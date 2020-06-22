
#ifndef FILTER_H
#define FILTER_H

#include <hellfire.h>
#include <noc.h>

#define GAUSS_BLOCK_SIZE 36*36
#define GAUSS_HEIGHT 36
#define GAUSS_WIDTH 36

#define SOBEL_BLOCK_SIZE 42*42
#define SOBEL_HEIGHT 42
#define SOBEL_WIDTH 42

#define MAX_SEQUENCE 64 

#define TOTAL_HEIGHT 256
#define TOTAL_WIDTH 256

#define MASTER_PORT 5000
#define SLAVE_PORT 1000

typedef enum {
    CORE0 = 0,
    CORE1,
    CORE2,
    CORE3,
    CORE4
} core_type;

typedef enum {
    ACK = 0,
    READY,
    IMG_BLOCK
} packet_type;

typedef enum {
    GAUSSIAN = 0,
    SOBEL
} filter_type;

typedef struct core_packet {
    uint8_t ready;
    int32_t sequence;
    core_type id;
    filter_type filter;
    packet_type packetType;
    uint8_t buff[SOBEL_BLOCK_SIZE]; 
} __attribute__((packed)) corePacket;

/* ---------------------- FUNCTIONS ---------------- */

void sender(int8_t *buf, core_type targetCore, int16_t channel, uint16_t targetPort);
void appendBuffer(corePacket *buffer, uint8_t *output, core_type coreId);
uint8_t startBuffer(corePacket buffer);
void receive(uint8_t *buf, int32_t src_channel);
uint8_t allReady(void);
void initCores(void);

/* ------------------------ THREADS -------------------*/

void core0(void);
void core1(void);
void core2(void);
void core3(void);
void core4(void);

/* -----------------  MASTER STATE MACHINE ---------  */

void master_fsm(void);

void * master_gaussian(void);
void * master_sobel(void);

void * master_prepareBuffer(void);
void * master_appendImage(void);
void * master_sendAck(void);
void * master_waitAck(void);
void * master_sendBuffer(void);
void * master_waitForBuffer(void);

/* ----------------- SLAVE STATE MACHINE ------------ */

void slave_fsm(void);

void * slave_filter(void);
void * slave_sendPacket(void);
void * slave_sendReady(void);
void * slave_sendAck(void);
void * slave_waitAck(void);
void * slave_waitingForPacket(void);

#endif