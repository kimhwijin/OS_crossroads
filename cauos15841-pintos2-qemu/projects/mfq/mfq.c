#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"
#include "threads/interrupt.h"

#include "devices/timer.h"

#include "projects/mfq/mfq.h"



struct semaphore sem_tick;

void thread_loop(void *aux)
{
    int i; 
    struct thread *t = thread_current ();
    printf("name : %s, prior : %d \n", t->name, t->priority);
    schedule();
    for(i=0; i<5; i++) {
        thread_print_stats();
        thread_tick();
        printf("%d", get_next_tick_to_wakeup());
        timer_msleep(1000);
    }
}

void run_mfqtest(char **argv)
{   
    int cnt;
	char *token, *save_ptr;

    enum intr_level old_level;
    old_level = intr_disable ();

    /// TODO: make your own test

    sema_init(&sem_tick, 1);

	cnt = 0;
	for (token = strtok_r (argv[1], ":", &save_ptr); token != NULL; token = strtok_r (NULL, ":", &save_ptr)) {
        char *subtoken, *save_ptr2;
        subtoken = strtok_r (token, ".", &save_ptr2);
        printf("thread name: %s\n", &subtoken[1]);

        char *t_name = &subtoken[1];
        subtoken = strtok_r (NULL, ".", &save_ptr2);
        printf("priority: %d\n", atoi(subtoken));
        // you can create threads here 
    
        thread_create(t_name, atoi(subtoken), thread_loop, NULL);
		cnt++;
        
	}
    intr_set_level (old_level);

    while (1) {
        timer_msleep(1000);
    }
}