#include "filter.h"
#include "image.h"

uint32_t i, j, k = 0;
uint32_t time;
uint32_t sequence = 0;

//corePacket cores[4];
typedef void *(*state_func)();
core_type currentCore;

uint8_t gaussianOutput[TOTAL_HEIGHT*TOTAL_WIDTH];

uint8_t firstTime = 1;
uint8_t sent = 0;
uint8_t filtered = 0;

corePacket rwBuffer;

filter_type currentFilter;

int32_t nAcks = 0;

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

		printf("\nReceived from core%d\n", currentCore);

		switch (buffer.packetType) {
		case READY:
			printf("Received ready buffer...\n");
			return master_prepareBuffer;
		case IMG_BLOCK:
			printf("Received image block buffer...\n");
			return master_sendAck;
		default:
			printf("Received ERROR...\n");
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

	//nAcks++;

	if (i >= 0) {

		memset((uint8_t *)&buffer, 0, sizeof(corePacket));
		receive((uint8_t *)&buffer, i);

		printf("\nReceived from core%d -- Packet Type: %d\n", currentCore, buffer.packetType);

		if(buffer.packetType == ACK) {
			printf("Ack detected!!\n");
			currentCore = 5;
			memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));
			return master_gaussian;
		}
	}

	return master_waitAck;
}

void * master_sendAck(void) {
	printf("Sending ack...\n");
	corePacket buffer;

	buffer.packetType = ACK;

	memset(buffer.buff, 0, sizeof(buffer.buff));

	sender((int8_t *)&buffer, currentCore, (int16_t)CORE4, 1000); 

	delay_ms(50);

	return master_prepareBuffer;
}

void * master_sendBuffer(void) {
	printf("Sending buffer...\n");

	sender((int8_t *)&rwBuffer, currentCore, (int16_t)CORE4, 1000); //envia para onde ele recebeu
	//memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));

	firstTime = 1;

	//delay_ms(50);

	return master_waitAck;
}

void * master_prepareBuffer(void) {
	uint32_t size = 0;

	switch (currentFilter)
	{
	case GAUSSIAN:
		printf("Preparing a gaussian buffer...\n");
		size = GAUSS_BLOCK_SIZE;
		break;
	case SOBEL:
		printf("Preparing a sobel buffer...\n");
		size = SOBEL_BLOCK_SIZE;
		break;
	}
	
	rwBuffer.packetType = IMG_BLOCK;

	memset(rwBuffer.buff, '2', size);

	return master_sendBuffer;
}

void * master_gaussian(void) {
	printf("Gaussian step...\n");

	firstTime = 1;
	nAcks = 0;

	currentFilter = GAUSSIAN;

	/*adicionar lógica pra mudar de filtro */

	return master_waitForBuffer;
}

/* -----------------------------------  SLAVE STATE MACHINE ---------------------------  */

void * slave_filter(void) {
	printf("Filtering...\n");
	
	switch (rwBuffer.filter)
	{
	case GAUSSIAN:
		printf("Doing gaussian...\n");
		break;
	case SOBEL:
		printf("Doing sobel...\n");
		break;
	}

	return slave_sendPacket;
}

void * slave_waitAck(void) {
	corePacket buffer;

	printf("Waiting for ack...\n");

	memset((uint8_t *)&buffer, 0, sizeof(corePacket));
	receive((uint8_t *)&buffer, CORE4);

	return (buffer.packetType == ACK) ? slave_waitingForPacket : slave_sendPacket;
}

void * slave_waitingForPacket(void) {
	int32_t i = -1;

	printf("Waiting for a packet...\n");

	i = hf_recvprobe();

	if(i >= 0) {
		memset((uint8_t *)&rwBuffer, 0, sizeof(corePacket));
		receive((uint8_t *)&rwBuffer, CORE4);
	}

	return (rwBuffer.packetType == IMG_BLOCK) && i >= 0 ? slave_sendAck : slave_sendReady;
}

void * slave_sendAck(void) {
	printf("Packet received!\n");
	printf("Sending ack...\n");

	corePacket buffer;

	memset(buffer.buff, 0, sizeof(buffer.buff));
	buffer.packetType = ACK;

	sender((int8_t *)&buffer, CORE4, (int16_t)hf_cpuid(), 5000); 
	memset(&buffer, 0, sizeof(corePacket));

	delay_ms(50);

	return slave_filter;
}

void * slave_sendPacket(void) {
	if(firstTime) {
		firstTime = 0;
		printf("Sending packet...\n");
	}

	rwBuffer.packetType = IMG_BLOCK;

	sender((int8_t *)&rwBuffer, CORE4, (int16_t)hf_cpuid(), 5000); 
	//memset(rwBuffer.buff, 0, BLOCK_SIZE);

	delay_ms(50);

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

	val = hf_send((uint16_t)targetCore, targetPort, buf, sizeof(corePacket), channel);

	if (val)
		printf("hf_send(): error %d\n", val);
}

void receive(uint8_t *buf, int32_t src_channel) {
	uint16_t cpu, port, size;
	int16_t val;

	//printf("\nTentando receber de channel %d\n", src_channel);

	val = hf_recv(&cpu, &port, (int8_t *)buf, &size, src_channel);
	//printf("\nDebug size: %d\n", size);
			
	if (val)
		printf("hf_recv(): error %d\n", val);
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
