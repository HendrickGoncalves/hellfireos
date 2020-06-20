#include "filter.h"
#include "image.h"

uint32_t i, j, k = 0;
uint32_t time;
uint32_t sequence = 0;

corePacket cores[4];
typedef void *(*state_func)();
core_type currentCore;

uint8_t gaussianOutput[TOTAL_HEIGHT*TOTAL_WIDTH];

uint8_t firstTime = 1;
uint8_t sent = 0;

corePacket rwBuffer;

/* -----------------------------------  THREADS ---------------------------  */

void core4(void) {

	if (hf_comm_create(hf_selfid(), MASTER_PORT, 0)) 
		panic(0xff);

	master_fsm();
}

void core3(void) {

	if (hf_comm_create(hf_selfid(), SLAVE_PORT, 0)) //criando ID da thread
		panic(0xff);

	slave_fsm();
}

void core2(void) {

	if (hf_comm_create(hf_selfid(), SLAVE_PORT, 0)) //criando ID da thread
		panic(0xff);

	slave_fsm();
}

void core1(void) {

	if (hf_comm_create(hf_selfid(), SLAVE_PORT, 0)) //criando ID da thread
		panic(0xff);

	slave_fsm();
}

void core0(void) {

	if (hf_comm_create(hf_selfid(), SLAVE_PORT, 0)) //criando ID da thread
		panic(0xff);

	slave_fsm();
}

/* -----------------------------------  MASTER STATE MACHINE ---------------------------  */

void * waitForBuffer(void) {
	int32_t i = 0;
	corePacket buffer;

	if(firstTime) {
		sent = 0;
		firstTime = 0;
		printf("Waiting for a buffer...\n");
	}

	i = hf_recvprobe(); //retorna o valor do channel que recebeu algo. se for < 0 é porque nenhum canal recebeu alguma coisa

	if (i >= 0) {
		currentCore =  (i <= 4) ? (core_type)i : 5;
	
		memset((uint8_t *)&buffer, 0, sizeof(corePacket));
		receive((uint8_t *)&buffer, i);

		switch (currentCore) {
			case CORE0:

				printf("\nReceived from core0\n");

				if(!startBuffer(buffer)) {
					appendBuffer(&buffer, gaussianOutput, currentCore);
					cores[0].sequence++;
					sequence++;
				} else {
					printf("Start buffer received!\n");
					cores[0].ready = 1;
				}

				return gaussian;
			case CORE1:
			
				printf("\nReceived from core1\n");

				if(!startBuffer(buffer)) {
					appendBuffer(&buffer, gaussianOutput, currentCore);
					cores[1].sequence++;
					sequence++;
				} else {
					printf("Start buffer received!\n");
					cores[1].ready = 1;
				}

				return gaussian;
			case CORE2:
				
				printf("\nReceived from core2\n");

				if(!startBuffer(buffer)) {
					appendBuffer(&buffer, gaussianOutput, currentCore);
					cores[2].sequence++;
					sequence++;
				} else {
					printf("Start buffer received!\n");
					cores[2].ready = 1;
				}

				return gaussian;
			case CORE3:

				printf("\nReceived from core3\n");

				if(!startBuffer(buffer)) {
					appendBuffer(&buffer, gaussianOutput, currentCore);
					cores[3].sequence++;
					sequence++;
				} else {
					printf("Start buffer received!\n");
					cores[3].ready = 1;
				}
		
				return gaussian;
			default:
				printf("Default: %d", i);
				return waitForBuffer;
		}
	}

	return waitForBuffer;
}

void * sendBuffer(void) {
	int32_t i;

	printf("Sending buffer...\n");

	sender((int8_t *)&rwBuffer, currentCore, (int16_t)CORE4, 1000); //envia para onde ele recebeu
	memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));

	sent = 1;

	return gaussian;
}

void * prepareBuffer(void) {
	printf("Preparing buffer...\n");

	memset(rwBuffer.buff, '2', BLOCK_SIZE);

	return sendBuffer;
}

void * gaussian(void) {
	printf("Gaussian step...\n");

	firstTime = 1;

	if(cores[currentCore].ready && !sent) return prepareBuffer;

	return waitForBuffer;
}

/* -----------------------------------  SLAVE STATE MACHINE ---------------------------  */

void * filter(void) {
	printf("Filtering...\n");
	strcpy((char *)rwBuffer.buff, "TEUCU");

	return sendPacket;
}

void * waitingForPacket(void) {
	printf("Waiting for packet...\n");

	memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));
	receive((uint8_t *)&rwBuffer, CORE4);

	return (rwBuffer.buff[0] != '\0' && strlen((char *)rwBuffer.buff) > 0) ? filter : waitingForPacket;
}

void * sendPacket(void) {
	if(firstTime) {
		memset(rwBuffer.buff, '1', BLOCK_SIZE);
		firstTime = 0;
		printf("Sending packet...\n");
	}

	sender((int8_t *)&rwBuffer, CORE4, (int16_t)hf_cpuid(), 5000); 
	memset(rwBuffer.buff, 0, BLOCK_SIZE);

	return waitingForPacket;
}

/* ------------------------------------------ FUNCTIONS ----------------------------- */

void slave_fsm(void) {
	state_func slaveState = sendPacket;

    while(1)
        slaveState = (state_func)(*slaveState)();
}

void master_fsm(void) {
	state_func masterState = gaussian;
    
	initCores();

    while(1)
        masterState = (state_func)(*masterState)();
}

void initCores(void) {
	cores[0].id = CORE0;
	cores[1].id = CORE1;
	cores[2].id = CORE2;
	cores[3].id = CORE3;
}

void sender(int8_t *buf, core_type targetCore, int16_t channel, uint16_t targetPort) {
	int16_t val; 
	
	printf("Sending a packet to: Core %d Port %d Channel %d\n", targetCore, targetPort, channel);

	val = hf_send((uint16_t)targetCore, targetPort, buf, sizeof(corePacket), channel);

	if (val)
		printf("hf_send(): error %d\n", val);
}

void receive(uint8_t *buf, int32_t src_channel) {
	uint16_t cpu, port, size;
	int16_t val;

	val = hf_recv(&cpu, &port, (int8_t *)buf, &size, src_channel);
			
	if (val)
		printf("hf_recv(): error %d\n", val);
}

uint8_t startBuffer(corePacket buffer) {
	return (buffer.buff[0] == '1' && buffer.buff[BLOCK_SIZE-1] == '1') ? 1 : 0;
}

void appendBuffer(corePacket *buffer, uint8_t *output, core_type coreId) {
	int32_t i, j;
	int32_t range, this_height, this_width;

	printf("Appending buffer...\n");

	range = (sequence * HEIGHT) - 1;
	this_height = HEIGHT + range;
	this_width = this_height;

	//printf("Debug append: \n");
	for(i = range; i < this_height; i++) {
		if (i > 1 && i < this_height-2) {
			for(j = 0; j < this_width; j++) {
				output[((i * this_width) + j)] = buffer->buff[((i * this_width) + j)];
				//printf("%X", output[((i * this_width) + j)]);
			}
		} else {
			output[((i * this_width) + j)] = buffer->buff[((i * this_width) + j)];	
			//printf("%X", output[((i * this_width) + j)]);
		}
	}
	//printf("\n");
}

void app_main(void) {
	if (hf_cpuid() == 4) //master
		hf_spawn(core4, 0, 0, 0, "master", 4096);
	else if(hf_cpuid() == 0) 
		hf_spawn(core0, 0, 0, 0, "slave0", 4096);
	else if(hf_cpuid() == 1) 
		hf_spawn(core1, 0, 0, 0, "slave1", 4096);
	else if(hf_cpuid() == 2) 
		hf_spawn(core2, 0, 0, 0, "slave2", 4096);
	else if(hf_cpuid() == 3) 
		hf_spawn(core3, 0, 0, 0, "slave3", 4096);
}
