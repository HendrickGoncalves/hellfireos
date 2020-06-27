#include "filter.h"
#include "image.h"

int32_t k = 0, l = 0;

uint32_t time;
uint32_t sequence = 0;

typedef void *(*state_func)();
core_type currentCore;

uint8_t *bufferAux;
uint8_t *gaussianOutput;
uint8_t *sobelOutput;

uint8_t firstTime = 1;
uint8_t sent = 0;
uint8_t filtered = 0;
uint8_t initBuffer = 1;
uint8_t ready = 0;
uint8_t ack_flags[4];
uint8_t notFinished = 0;
uint8_t coreReadyFlag[] = {0, 0, 0, 0};
uint8_t ack = 0;

corePacket rwBuffer;

filter_type currentFilter;

/* -----------------------------------  THREADS ---------------------------  */

void master(void) {
	if (hf_comm_create(hf_selfid(), MASTER_PORT, 0)) 
		panic(0xff);

	bufferAux = (uint8_t *)malloc(GAUSS_IMG_OUTPUT * sizeof(uint8_t));
	gaussianOutput = (uint8_t *)malloc(IMG_SIZE * sizeof(uint8_t));

	time = _readcounter();

	master_fsm();
}

void slave(void) {
	if (hf_comm_create(hf_selfid(), SLAVE_PORT, 0)) //criando ID da thread
		panic(0xff);

	slave_fsm();
}

/* -----------------------------------  MASTER STATE MACHINE ---------------------------  */

void * master_waitForBuffer(void) {
	int32_t i = -208;
	corePacket buffer;

	currentCore = 5;

	if(firstTime) {
		sent = 0;
		firstTime = 0;
	}

	i = hf_recvprobe();

	if (i >= 0) {
	
		memset((uint8_t *)&buffer, 0, sizeof(corePacket));
		receive((uint8_t *)&buffer, i);

		currentCore = (core_type)buffer.id;

		switch (buffer.packetType) {
		case READY:

			if(coreReadyFlag[currentCore])
				break;

			coreReadyFlag[currentCore] = 1;

			return master_prepareBuffer;
		case IMG_BLOCK:
			memcpy((uint8_t *)&rwBuffer, (uint8_t *)&buffer, sizeof(corePacket)); 

			sequence++;

			printf("Sequence: %d\n", sequence);

			return master_sendAck;
		
		case ACK: 

			if(buffer.packetType == ACK && i == currentCore) {
				currentCore = 5;
				memset((uint8_t *)&buffer, 0, sizeof(corePacket));
				break;
			} 
		default:
			break;
		}
	
	} 

	return master_waitForBuffer;
}

void * master_appendBuffer(void) {
	if(rwBuffer.k == 256 && rwBuffer.l >= 0) {
		sequence--;
		return (currentFilter == GAUSSIAN) ? master_gaussian : master_sobel;
	}

	appendBuffer(bufferAux, rwBuffer.buff, rwBuffer.l, rwBuffer.k, currentFilter);

	memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));
 
	return master_prepareBuffer;
}

void * master_sendAck(void) {
	corePacket buffer;

	ack_flags[currentCore] = 0;

	memset((uint8_t *)&buffer, 0, sizeof(corePacket));

	buffer.packetType = ACK;

	sender((int8_t *)&buffer, currentCore, (int16_t)CORE4, SLAVE_PORT); 

	return master_appendBuffer;
}

void * master_sendBuffer(void) {

	sender((int8_t *)&rwBuffer, currentCore, (int16_t)CORE4, SLAVE_PORT); //envia para onde ele recebeu

	firstTime = 1;

	return (currentFilter == GAUSSIAN) ? master_gaussian : master_sobel;
}

void * master_prepareBuffer(void) {
	memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));

	switch (currentFilter) {
	case GAUSSIAN:
		rwBuffer.filter = GAUSSIAN;
		splitGauss(image, rwBuffer.buff, l, k); //split matrix

		break;
	case SOBEL:

		rwBuffer.filter = SOBEL;
		splitSobel(gaussianOutput, rwBuffer.buff, l, k); //split matrix

		break;
	}
	
	rwBuffer.packetType = IMG_BLOCK;
	rwBuffer.k = k;
	rwBuffer.l = l;
	rwBuffer.sequence = sequence;

	ack_flags[currentCore] = 1;

	k = !((l+1) % 8) && l > 0 ? k+32 : k;
	l = !((l+1) % 8) && l > 0 ? 0 : l+1;

	return master_sendBuffer;
}

void * master_sobel(void) {

	firstTime = 1;

	currentFilter = SOBEL;

	if(sequence < MAX_SEQUENCE) return master_waitForBuffer;

	sendFinishPacket(FINISH_SOBEL);

	free(gaussianOutput);
	gaussianOutput = NULL;

	sobelOutput = (uint8_t *)malloc(IMG_SIZE * sizeof(uint8_t));

	cutImage(sobelOutput, bufferAux, SOBEL);

	time = _readcounter() - time;
	printf("done in %d clock cycles.\n\n", time);

	showImg(sobelOutput);

	while(1);
}

void * master_gaussian(void) {

	firstTime = 1;

	currentFilter = GAUSSIAN;

	if(sequence < MAX_SEQUENCE) return master_waitForBuffer;

	sendFinishPacket(FINISH_GAUSS);
	cleanRXBuffer();

	cutImage(gaussianOutput, bufferAux, GAUSSIAN);

	free(bufferAux);
	bufferAux = NULL;

	bufferAux = (uint8_t *)malloc(SOBEL_IMG_OUPUT * sizeof(uint8_t));
	memset(bufferAux, 0, SOBEL_IMG_OUPUT);

	sequence = 0;
	k = 0;
	l = 0;
	coreReadyFlag[0] = 0;
	coreReadyFlag[1] = 0;
	coreReadyFlag[2] = 0;
	coreReadyFlag[3] = 0;

	return master_sobel;
}

/* -----------------------------------  SLAVE STATE MACHINE ---------------------------  */

void * slave_filter(void) {
	uint8_t *filterOutput;
	int32_t size = 0;
	
	switch (rwBuffer.filter) {
	case GAUSSIAN:

		size = GAUSS_BLOCK_SIZE;
		filterOutput = (uint8_t *)malloc(GAUSS_BLOCK_SIZE * sizeof(uint8_t));

        do_gaussian(rwBuffer.buff, filterOutput, GAUSS_HEIGHT, GAUSS_WIDTH); //filtra com o buffer que ele recebeu

		break;
	case SOBEL:

		size = SOBEL_BLOCK_SIZE;
		filterOutput = (uint8_t *)malloc(SOBEL_BLOCK_SIZE * sizeof(uint8_t));

		memset(filterOutput, 0, SOBEL_BLOCK_SIZE);

        do_sobel(rwBuffer.buff, filterOutput, SOBEL_HEIGHT, SOBEL_WIDTH); //filtra com o buffer que ele recebeu

		break;
	}

	memcpy(rwBuffer.buff, filterOutput, size);

	free(filterOutput);
	filterOutput = NULL;

	return slave_sendPacket;
}

void * slave_waitAck(void) {
	corePacket buffer;
	uint8_t i = 0;

	i = hf_recvprobe();

	if(i >= 0) {
		memset((uint8_t *)&buffer, 0, sizeof(corePacket));
		receive((uint8_t *)&buffer, CORE4);
	}

	if(buffer.packetType == ACK && i >= 0) {
		ack = 1;
		return slave_waitingForPacket;
	}

	//delay_ms(50);

	return slave_sendPacket;
}

void * slave_waitingForPacket(void) {
	int32_t i = -208;

	i = hf_recvprobe();

	if(i >= 0) {
		memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));
		receive((uint8_t *)&rwBuffer, CORE4);
	}

	if(rwBuffer.packetType == FINISH_GAUSS && i>= 0) {
		memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));
		ready = 0;
		return slave_sendReady;
	} else if(rwBuffer.packetType == FINISH_SOBEL && i>= 0) {
		while(1);
	}

	if((rwBuffer.packetType == IMG_BLOCK) && i >= 0) return slave_sendAck;
	
	if(!ready) {
		delay_ms(50);
		return slave_sendReady;
	}

	if(rwBuffer.packetType == ACK && i >= 0) return slave_waitingForPacket;
	if(ack) return slave_waitingForPacket;

	return slave_sendPacket;
}

void * slave_sendAck(void) {

	corePacket buffer;

	ready =  1;

	memset((uint8_t *)&buffer, 0, sizeof(corePacket));
	buffer.packetType = ACK;
	buffer.id = hf_cpuid();

	delay_ms(50);

	sender((int8_t *)&buffer, CORE4, (int16_t)hf_cpuid(), MASTER_PORT); 
	memset(&buffer, 0, sizeof(corePacket));	

	return (rwBuffer.k == 256 && rwBuffer.l == 0) ? slave_sendReady : slave_filter; 
}

void * slave_sendPacket(void) {

	ack = 0;

	rwBuffer.packetType = IMG_BLOCK;
	rwBuffer.id = hf_cpuid();

	sender((int8_t *)&rwBuffer, CORE4, (int16_t)hf_cpuid(), MASTER_PORT); 

	return slave_waitAck;
}

void * slave_sendReady(void) {
	corePacket buffer;

	memset((uint8_t *)&buffer, 0, sizeof(corePacket));

	buffer.packetType = READY;
	buffer.id = hf_cpuid();

	sender((int8_t *)&buffer, CORE4, (int16_t)hf_cpuid(), 5000); 	

	return slave_waitingForPacket;
}

/* ------------------------------------------ FUNCTIONS ----------------------------- */

void cleanRXBuffer(void) {
	uint8_t i;
	corePacket buffer;

	for(i = 0; i < 4; i++) {
		receive((uint8_t *)&buffer, i);
	}
}

void sendFinishPacket(packet_type filter) {
	corePacket buffer;
	uint8_t i;

	ack_flags[currentCore] = 0;

	memset((uint8_t *)&buffer, 0, sizeof(corePacket));

	buffer.packetType = filter;

	for (i = 0; i < 4; i++) 
		sender((int8_t *)&buffer, (core_type)i, (int16_t)CORE4, SLAVE_PORT); 
	
}

void slave_fsm(void) {
	state_func slaveState = slave_sendReady;

    while(1)
        slaveState = (state_func)(*slaveState)();
}

void master_fsm(void) {
	state_func masterState = master_gaussian;

    while(1)
        masterState = (state_func)(*masterState)();
}

void sender(int8_t *buf, core_type targetCore, int16_t channel, uint16_t targetPort) {
	int16_t val; 

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

void app_main(void) {
	if (hf_cpuid() == 4) //master
		hf_spawn(master, 0, 0, 0, "master", 100000);
	else if(hf_cpuid() == 0) 
		hf_spawn(slave, 0, 0, 0, "slave0", 90000);
	else if(hf_cpuid() == 1) 
		hf_spawn(slave, 0, 0, 0, "slave1", 90000);
	else if(hf_cpuid() == 2) 
		hf_spawn(slave, 0, 0, 0, "slave2", 90000);
	else if(hf_cpuid() == 3) 
		hf_spawn(slave, 0, 0, 0, "slave3", 90000);
}
