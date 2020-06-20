
#ifndef FILTER_H
#define FILTER_H

#include <hellfire.h>
#include <noc.h>

#define MAX_BLOCK_SIZE 1684 //bytes / parte
#define BLOCK_SIZE 1444 //38x38

#define MAX_SEQUENCE 64 //64 partes

#define TOTAL_HEIGHT 256
#define TOTAL_WIDTH 256
#define HEIGHT 38
#define WIDTH 38

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
    GAUSSIAN = 0,
    SOBEL
} filter_type;

typedef struct core_packet {
    uint8_t ready;
    uint8_t sequence;
    core_type id;
    filter_type filter;
    uint8_t buff[BLOCK_SIZE]; //38x38
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

void * gaussian(void);
void * sobel(void);

void * prepareBuffer(void);
void * sendBuffer(void);
void * waitForBuffer(void);

/* ----------------- SLAVE STATE MACHINE ------------ */

void slave_fsm(void);

void * filter(void);
void * sendPacket(void);
void * waitingForPacket(void);

#endif