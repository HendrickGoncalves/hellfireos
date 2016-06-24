/**
 * @file tcb.h
 * @author Sergio Johann Filho
 * @date February 2016
 * 
 * @section LICENSE
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file 'doc/license/gpl-2.0.txt' for more details.
 * 
 * @section DESCRIPTION
 * 
 * Task management primitives and auxiliary functions.
 * 
 */

/* task status definitions */
#define TASK_IDLE			0		/*!< task does not exist / not ready */
#define TASK_READY			1		/*!< task ready to run (on run queue) */
#define TASK_RUNNING			2		/*!< task running (only one task/core can be in this state, on run queue) */
#define TASK_BLOCKED			3		/*!< task blocked, can be resumed later (on run queue) */
#define TASK_DELAYED			4		/*!< task being delayed (on delay queue) */
#define TASK_WAITING			5		/*!< task waiting for an event (on event queue) */

/**
 * @brief Task control block (TCB) entry data structure.
 */
struct tcb_entry {
	uint16_t id;					/*!< task id */
	int8_t name[20];				/*!< task description (or name) */
	volatile uint8_t state;				/*!< 0 - idle,  1 - ready,  2 - running, 3 - blocked, 4 - delayed, 5 - waiting */
	volatile uint32_t delay;			/*!< delay to enter in the run/RT queue */
	volatile uint32_t jobs;				/*!< total task jobs executed */
	volatile uint32_t deadline_misses;		/*!< task realtime deadline misses */
	volatile uint16_t period;			/*!< task period */
	volatile uint16_t capacity;			/*!< task capacity */
	volatile uint16_t deadline;			/*!< task deadline */
	volatile uint16_t capacity_rem;			/*!< remaining capacity on period */
	volatile uint16_t deadline_rem;			/*!< remaining time slices on period */
	context task_context;				/*!< task context */
	void (*ptask)(void);				/*!< task entry point, pointer to function */
	int32_t *pstack;				/*!< task stack area (bottom) */
	uint32_t stack_size;				/*!< task stack size */
	void *other_data;				/*!< pointer to other data related to this task */
};

/**
 * @brief The task control block
 */
struct tcb_entry krnl_tcb[MAX_TASKS];

struct tcb_entry * volatile krnl_task;			/*!< pointer to a task control block entry */
volatile uint16_t krnl_tasks;				/*!< number of tasks in the system */
volatile uint16_t krnl_current_task;			/*!< the current running task id */
volatile uint16_t krnl_schedule;			/*!< scheduler enable / disable flag */
struct queue *krnl_run_queue;				/*!< pointer to a queue of best effort tasks */
struct queue *krnl_delay_queue;				/*!< pointer to a queue of delayed tasks */
struct queue *krnl_rt_queue;				/*!< pointer to a queue of real time tasks */
struct queue *krnl_event_queue;				/*!< pointer to a queue of tasks waiting for an event */
uint8_t krnl_heap[HEAP_SIZE];				/*!< contiguous heap memory area to be used as a memory pool. the memory allocator (malloc() and free()) controls this data structure */
uint32_t krnl_free;					/*!< amount of free heap memory, in bytes */