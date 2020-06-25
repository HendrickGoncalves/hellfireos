#include "filter.h"
#include "image.h"

int32_t k = 0, l = 0;

uint32_t time;
uint32_t sequence = 0;
uint32_t core_sequence[4];
//uint32_t 

typedef void *(*state_func)();
core_type currentCore;

uint8_t *bufferAux;
uint8_t *gaussianOutput;

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

void core4(void) {

	if (hf_comm_create(hf_selfid(), MASTER_PORT, 0)) 
		panic(0xff);

	bufferAux = (uint8_t *)malloc(SOBEL_IMG_OUPUT * sizeof(uint8_t));
	gaussianOutput = (uint8_t *)malloc(GAUSS_IMG_OUTPUT * sizeof(uint8_t));

	time = _readcounter();

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

void * master_waitForBuffer(void) {
	int32_t i = -208;
	corePacket buffer;

	delay_ms(50);

	currentCore = 5;

	if(firstTime) {
		sent = 0;
		firstTime = 0;
		printf("Waiting for a buffer...\n");
	}

	i = hf_recvprobe(); //retorna o valor do channel que recebeu algo. se for < 0 é porque nenhum canal recebeu alguma coisa

	if (i >= 0) {
		printf("TX MODE\n");
	
		memset((uint8_t *)&buffer, 0, sizeof(corePacket));
		receive((uint8_t *)&buffer, i);

		currentCore = (core_type)buffer.id;

		printf("Received from core%d: k %d l %d\n", currentCore, buffer.k, buffer.l);

		switch (buffer.packetType) {
		case READY:
			printf("Received ready buffer...\n");

			if(coreReadyFlag[currentCore])
				break;

			coreReadyFlag[currentCore] = 1;

			return master_prepareBuffer;
		case IMG_BLOCK:
			printf("Received image block buffer...\n");
			memcpy((uint8_t *)&rwBuffer, (uint8_t *)&buffer, sizeof(corePacket)); //copia para dentro do buffer antes do append

			//if(core_sequence[currentCore] == rwBuffer.sequence)
				sequence++;

			printf("Sequence: %d\n", sequence);

			return master_sendAck;
		
		case ACK: 

			if(buffer.packetType == ACK && i == currentCore) {
				printf("Ack received from core%d\n", i);
				currentCore = 5;
				memset((uint8_t *)&buffer, 0, sizeof(corePacket));
				break;
			} 
		default:
			printf("Received ERROR: %d\n", buffer.packetType);
			break;
		}
	
	} 

	return master_waitForBuffer;
}

void * master_waitAck(void) {
	int32_t i = 0;
	corePacket buffer;

	if(firstTime) {
		firstTime = 0;
		printf("Waiting for ack...\n");
	}

	i = hf_recvprobe(); 

	if (i >= 0) {

		memset((uint8_t *)&buffer, 0, sizeof(corePacket));
		receive((uint8_t *)&buffer, i);

		if(buffer.packetType == ACK && i == currentCore) {
			printf("Ack received from core%d\n", i);
			currentCore = 5;
			memset((uint8_t *)&buffer, 0, sizeof(corePacket));
			return master_gaussian;
		} 
	}

	return master_gaussian;
}

void * master_appendBuffer(void) {
	printf("Appending buffer: k %d l %d\n", rwBuffer.l, rwBuffer.k);

	appendBuffer(bufferAux, rwBuffer.buff, rwBuffer.l, rwBuffer.k, currentFilter);

	memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));
 
	return master_prepareBuffer;
}

void * master_sendAck(void) {
	printf("Sending ack...\n");
	corePacket buffer;
	uint8_t i;

	ack_flags[currentCore] = 0;

	buffer.packetType = ACK;

	memset(buffer.buff, 0, sizeof(buffer.buff));

	//for (i = 0; i < 3; i++) {
		sender((int8_t *)&buffer, currentCore, (int16_t)CORE4, SLAVE_PORT); 
		//delay_ms(50);
	//}

	return master_appendBuffer;
}

void * master_sendBuffer(void) {
	uint8_t i;

	printf("Sending buffer...\n");

	//rwBuffer.packetType = IMG_BLOCK;
	
	//for (i = 0; i < 3; i++) {
		sender((int8_t *)&rwBuffer, currentCore, (int16_t)CORE4, SLAVE_PORT); //envia para onde ele recebeu
		//delay_ms(50);
	//}

	firstTime = 1;

	return master_gaussian;
}

void * master_prepareBuffer(void) {
	//uint8_t buffer[36*36];

	memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));

	switch (currentFilter) {
	case GAUSSIAN:
		printf("Preparing a gaussian buffer...\n");

		rwBuffer.filter = GAUSSIAN;
		splitGauss(image, rwBuffer.buff, l, k); //split matrix

		break;
	case SOBEL:
		printf("Preparing a sobel buffer...\n");

		rwBuffer.filter = SOBEL;
		splitSobel(image, rwBuffer.buff, l, k); //split matrix

		break;
	}
	
	rwBuffer.packetType = IMG_BLOCK;
	rwBuffer.k = k;
	rwBuffer.l = l;
	rwBuffer.sequence = sequence;

	core_sequence[currentCore] = sequence;
	ack_flags[currentCore] = 1;

	printf("\nK %d l %d enviado.\n", k, l);

	k = !((l+1) % 8) && l > 0 ? k+32 : k;
	l = !((l+1) % 8) && l > 0 ? 0 : l+1;


	return master_sendBuffer;
}

void * master_sobel(void) {
	return master_sobel;

	printf("Gaussian step...\n");

	firstTime = 1;

	currentFilter = SOBEL;

	if(sequence < MAX_SEQUENCE) return master_waitForBuffer;

	return master_sobel;
}

void * master_gaussian(void) {

	printf("Gaussian step...\n");

	firstTime = 1;

	currentFilter = GAUSSIAN;

	if(sequence < MAX_SEQUENCE) return master_waitForBuffer;
	
	sendFinishPacket();
	cleanRXBuffer();

	printf("\nGauss finished!\n");

	cutImage(gaussianOutput, bufferAux, GAUSSIAN);

	time = _readcounter() - time;
	printf("\nComp time: %d\n", time);

	showImg(gaussianOutput);

	memset(bufferAux, 0, sizeof(bufferAux));

	sequence = 0;

	return master_sobel;
}

/* -----------------------------------  SLAVE STATE MACHINE ---------------------------  */

void * slave_filter(void) {
	uint8_t *filterOutput;
	int32_t size;

	printf("Filtering: k %d l %d...\n", rwBuffer.k, rwBuffer.l);
	
	switch (rwBuffer.filter) {
	case GAUSSIAN:
		printf("Doing gaussian...\n");

		size = GAUSS_BLOCK_SIZE;
		filterOutput = (uint8_t *)malloc(GAUSS_BLOCK_SIZE * sizeof(uint8_t));

        do_gaussian(rwBuffer.buff, filterOutput, GAUSS_HEIGHT, GAUSS_WIDTH); //filtra com o buffer que ele recebeu

		break;
	case SOBEL:
		printf("Doing sobel...\n");
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

	printf("Waiting for ack...\n");

	delay_ms(50);

	i = hf_recvprobe();

	if(i >= 0) {
		memset((uint8_t *)&buffer, 0, sizeof(corePacket));
		receive((uint8_t *)&buffer, CORE4);
	}

	if(buffer.packetType == ACK && i >= 0) {
		printf("Ack received!\n");
		ack = 1;
		return slave_waitingForPacket;
	}

	delay_ms(100);

	return slave_sendPacket;
}

void * slave_waitingForPacket(void) {
	int32_t i = -208;

	printf("Waiting for a packet...\n");

	i = hf_recvprobe();

	if(i >= 0) {
		memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));
		receive((uint8_t *)&rwBuffer, CORE4);
	}

	if((rwBuffer.packetType == IMG_BLOCK) && i >= 0) return slave_sendAck;
	if(!ready) {
		delay_ms(300);
		return slave_sendReady;
	}
	if(rwBuffer.packetType == ACK && i >= 0) return slave_waitingForPacket;
	if(ack) return slave_waitingForPacket;

	return slave_sendPacket;
}

void * slave_sendAck(void) {
	int32_t i = 0;
	printf("Packet received!\n");
	printf("Sending ack...\n");

	corePacket buffer;

	ready =  1;

	memset(buffer.buff, 0, sizeof(buffer.buff));
	buffer.packetType = ACK;
	buffer.id = hf_cpuid();

	delay_ms(500);
	//for (i = 0; i < 3; i++) {
		sender((int8_t *)&buffer, CORE4, (int16_t)hf_cpuid(), 5000); 
		memset(&buffer, 0, sizeof(corePacket));	
		
	//}

	return slave_filter;
}

void * slave_sendPacket(void) {
	uint8_t i;

	if(firstTime) {
		firstTime = 0;
		printf("Sending packet...\n");
	}

	ack = 0;

	rwBuffer.packetType = IMG_BLOCK;
	rwBuffer.id = hf_cpuid();

	//for (i = 0; i < 5; i++) {
		delay_ms(100);
		sender((int8_t *)&rwBuffer, CORE4, (int16_t)hf_cpuid(), 5000); 
	//}

	return slave_waitAck;
}

void * slave_sendReady(void) {
	corePacket buffer;
	//uint8_t i;

	printf("Sending ready packet...\n");

	memset((uint8_t *)&buffer, 0, sizeof(corePacket));

	//delay_ms(200);

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

void sendFinishPacket(void) {
	corePacket buffer;
	uint8_t i;

	ack_flags[currentCore] = 0;

	buffer.packetType = ACK;

	memset(buffer.buff, 0, sizeof(buffer.buff));

	for (i = 4; i > 0; i--) {
		sender((int8_t *)&buffer, (core_type)i, (int16_t)CORE4, SLAVE_PORT); 
		delay_ms(50);
		sender((int8_t *)&buffer, (core_type)i, (int16_t)CORE4, SLAVE_PORT); 
		delay_ms(50);
		sender((int8_t *)&buffer, (core_type)i, (int16_t)CORE4, SLAVE_PORT); 
	}
}

uint8_t waitingPaket (void) {
	return (ack_flags[0] || ack_flags[1] || ack_flags[2] || ack_flags[3]) ? 1 : 0;
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
	
	printf("Sending a packet to: Core %d Port %d Channel %d\n", targetCore, targetPort, channel);

	//delay_ms(50);

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
		hf_spawn(core4, 0, 0, 0, "master", 100000);
	else if(hf_cpuid() == 0) 
		hf_spawn(core0, 0, 0, 0, "slave0", 90000);
	else if(hf_cpuid() == 1) 
		hf_spawn(core1, 0, 0, 0, "slave1", 90000);
	else if(hf_cpuid() == 2) 
		hf_spawn(core2, 0, 0, 0, "slave2", 90000);
	else if(hf_cpuid() == 3) 
		hf_spawn(core3, 0, 0, 0, "slave3", 90000);
}
