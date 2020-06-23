#include "filter.h"
#include "image.h"

int32_t k = 0, l = 0;

uint32_t time;
uint32_t sequence = 0;

typedef void *(*state_func)();
core_type currentCore;

uint8_t gaussianAux[GAUSS_IMG_OUTPUT];
//uint8_t sobelOuput[]

uint8_t firstTime = 1;
uint8_t sent = 0;
uint8_t filtered = 0;
uint8_t initBuffer = 1;
uint8_t ready = 0;

corePacket rwBuffer;

filter_type currentFilter;

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

void * master_waitForBuffer(void) {
	int32_t i = -1;
	corePacket buffer;

	currentCore = 5;

	if(firstTime) {
		sent = 0;
		firstTime = 0;
		printf("Waiting for a buffer...\n");
	}

	i = hf_recvprobe(); //retorna o valor do channel que recebeu algo. se for < 0 é porque nenhum canal recebeu alguma coisa

	if (i >= 0) {
		printf("\nTX MODE\n");

		currentCore = (core_type)i;
	
		memset((uint8_t *)&buffer, 0, sizeof(corePacket));
		receive((uint8_t *)&buffer, i);

		printf("Received from core%d\n", currentCore);

		switch (buffer.packetType) {
		case READY:
			printf("Received ready buffer...\n");
			return master_prepareBuffer;
		case IMG_BLOCK:
			printf("Received image block buffer...\n");
			memcpy((uint8_t *)&rwBuffer, (uint8_t *)&buffer, sizeof(corePacket)); //copia para dentro do buffer antes do append

			return master_sendAck;
		default:
			printf("Received ERROR: %d\n", buffer.packetType);
			return master_waitForBuffer;
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

		printf("\nReceived from core%d -- Packet Type: %d\n", i, buffer.packetType);

		if(buffer.packetType == ACK && i == currentCore) {
			printf("Ack detected!!\n");
			currentCore = 5;
			memset((uint8_t *)&buffer, 0, sizeof(corePacket));
			return master_gaussian;
		} 
		// else if (buffer.packetType == IMG_BLOCK && i == currentCore) {
		// 	printf("Ack/packet detected!!\n");
		// 	currentCore = 5;
		// 	memcpy(&rwBuffer, &buffer, sizeof(corePacket));
		// 	//memset((uint8_t *)&buffer, 0, sizeof(corePacket));
		// 	return master_appendBuffer;
		// }
		
	}

	return master_waitAck;
}

void * master_appendBuffer(void) {

	appendBuffer(gaussianAux, rwBuffer.buff, rwBuffer.l, rwBuffer.k, currentFilter);

	return master_prepareBuffer;
}

void * master_sendNext(void) {

}

void * master_sendAck(void) {
	printf("Sending ack...\n");
	corePacket buffer;
	uint8_t i;

	buffer.packetType = ACK;

	memset(buffer.buff, 0, sizeof(buffer.buff));

	for (i = 0; i < 3; i++) {
		sender((int8_t *)&buffer, currentCore, (int16_t)CORE4, 1000); 
		delay_ms(50);
	}

	return master_appendBuffer;
}

void * master_sendBuffer(void) {
	uint8_t i;

	printf("Sending buffer...\n");

	for (i = 0; i < 3; i++) {
		sender((int8_t *)&rwBuffer, currentCore, (int16_t)CORE4, 1000); //envia para onde ele recebeu
		delay_ms(50);
	}

	firstTime = 1;

	return master_waitAck;
}

void * master_prepareBuffer(void) {
	uint8_t buffer[36*36];

	memset(rwBuffer.buff, 0, sizeof(rwBuffer.buff));

	switch (currentFilter) {
	case GAUSSIAN:
		printf("Preparing a gaussian buffer...\n");

		rwBuffer.filter = GAUSSIAN;
		splitGauss(image, buffer, l, k); //split matrix

		memcpy(rwBuffer.buff, buffer, sizeof(buffer));

		//("Gaussian step done!\n");

		break;
	case SOBEL:
		//printf("Preparing a sobel buffer...\n");
		break;
	}
	
	rwBuffer.packetType = IMG_BLOCK;
	rwBuffer.k = k;
	rwBuffer.l = l;

	k = !((l+1) % 8) && l > 0 ? k+32 : k;
	l = !((l+1) % 8) && l > 0 ? 0 : l+1;

	sequence++;

	printf("\nSequence %d\n", sequence);

	return master_sendBuffer;
}

void * master_sobel(void) {
	return master_sobel;
}

void * master_gaussian(void) {
	uint8_t *gaussianOutput;

	printf("Gaussian step...\n");

	firstTime = 1;

	currentFilter = GAUSSIAN;

	if(sequence < MAX_SEQUENCE) return master_waitForBuffer;

	printf("\nGauss finished!\n");

	gaussianOutput = (uint8_t *)malloc(IMG_SIZE * sizeof(uint8_t));

	cutImage(gaussianOutput, gaussianAux, GAUSSIAN);
	showImg(gaussianOutput);

	free(gaussianOutput);

	/*adicionar lógica pra mudar de filtro */

	return master_sobel;
}

/* -----------------------------------  SLAVE STATE MACHINE ---------------------------  */

void * slave_filter(void) {
	uint8_t *filterOutput;
	int32_t size;

	printf("Filtering...\n");
	
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

	printf("Waiting for ack...\n");

	delay_ms(50);

	memset((uint8_t *)&buffer, 0, sizeof(corePacket));
	receive((uint8_t *)&buffer, CORE4);

	if(buffer.packetType == ACK)
		printf("Ack received!\n");

	return (buffer.packetType == ACK) ? slave_waitingForPacket : slave_waitAck;
}

void * slave_waitingForPacket(void) {
	int32_t i = -1;

	printf("Waiting for a packet...\n");

	i = hf_recvprobe();

	if(i >= 0) {
		memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));
		receive((uint8_t *)&rwBuffer, CORE4);
	}

	if((rwBuffer.packetType == IMG_BLOCK) && i >= 0) return slave_sendAck;

	if(!ready) return slave_sendReady;

	return slave_waitingForPacket;

	//return (rwBuffer.packetType == IMG_BLOCK) && i >= 0 ? slave_sendAck : slave_sendReady;
}

void * slave_sendAck(void) {
	int32_t i = 0;
	printf("Packet received!\n");
	printf("Sending ack...\n");

	corePacket buffer;

	ready =  1;

	memset(buffer.buff, 0, sizeof(buffer.buff));
	buffer.packetType = ACK;

	for (i = 0; i < 3; i++) {
		sender((int8_t *)&buffer, CORE4, (int16_t)hf_cpuid(), 5000); 
		memset(&buffer, 0, sizeof(corePacket));	
		delay_ms(50);
	}

	return slave_filter;
}

void * slave_sendPacket(void) {
	uint8_t i;

	if(firstTime) {
		firstTime = 0;
		printf("Sending packet...\n");
	}

	rwBuffer.packetType = IMG_BLOCK;

	for (i = 0; i < 5; i++) {
		sender((int8_t *)&rwBuffer, CORE4, (int16_t)hf_cpuid(), 5000); 
		delay_ms(50);
	}

	//delay_ms(50);

	return slave_waitAck;
}

void * slave_sendReady(void) {
	printf("Sending ready packet...\n");

	memset(rwBuffer.buff, 0, sizeof(rwBuffer.buff));

	rwBuffer.packetType = READY;

	sender((int8_t *)&rwBuffer, CORE4, (int16_t)hf_cpuid(), 5000); 	
	memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));

	delay_ms(50);

	return slave_waitingForPacket;
}

/* ------------------------------------------ FUNCTIONS ----------------------------- */

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
		hf_spawn(core0, 0, 0, 0, "slave0", 100000);
	// else if(hf_cpuid() == 1) 
	// 	hf_spawn(core1, 0, 0, 0, "slave1", 4096);
	// else if(hf_cpuid() == 2) 
	// 	hf_spawn(core2, 0, 0, 0, "slave2", 4096);
	// else if(hf_cpuid() == 3) 
	// 	hf_spawn(core3, 0, 0, 0, "slave3", 4096);
}
