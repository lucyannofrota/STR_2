#ifndef _STR_FUNCTIONS_H__
#define _STR_FUNCTIONS_H__

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>
#include <math.h>

#include "comp_defines.h"

// #include <sys/time.h>
// #include <stdio.h>


#include <stdlib.h>
#include <pthread.h>
#include <errno.h>

#include <semaphore.h>


// #include <sys/wait.h>

#define handle_error_en(en, msg) \
    do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

#define MAX_LINE_LEN (3-DBL_MIN_10_EXP+1)*3+10 // Tamanho maximo que uma linha pode ter. Assumindo 3xDLOUBLE + 10.
//(strlen("-0.")-DBL_MIN_10_EXP+1) Sendo o maior valor que uma variavel double pode assumir em ascii

typedef struct {
    double *x, *y, *z;
    int npoints;
} t_point_cloud;

void read_point_cloud(t_point_cloud **ptr, char *file_name);

void describe_point_cloud(t_point_cloud *ptr);

void free_t_point_cloud(t_point_cloud *ptr);

void filter_point_cloud(t_point_cloud **ptr);

void filter_roads(t_point_cloud **ptr);

void clk_wait(double m_sec);

void sub_timespec(struct timespec *tim_1,struct timespec *tim_2,struct timespec *result);

void calc_func_ripple(struct timespec dtime_spec[N_FUNCTIONS][N_SAMPLES]);

void display_thread_attr(pthread_t thread, char *prefix);

double timespec_to_double_ms(struct timespec *time);

void print_table(struct timespec *tab,int M, int N, char *prefix);

void thread_configs(pthread_attr_t *attr, int setaffinity, int sched_type,int priority_mod);



// Sem
void read_point_cloud_sem(t_point_cloud **out_ptr, char *file_name, sem_t *sem_b, sem_t *sem_a);

// void describe_point_cloud_mut(t_point_cloud *ptr);

void filter_point_cloud_sem(t_point_cloud **ptr, sem_t *sem_b, sem_t *sem_a);

void filter_roads_sem(t_point_cloud **ptr, sem_t *sem_b, sem_t *sem_a);


#endif //_STR_FUNCTIONS_H__