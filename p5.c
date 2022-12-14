#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <pthread.h>

#include "aux_libs/str_functions.h"

#include <semaphore.h>
#include <fcntl.h>
#include <sys/wait.h>

int cond_th[3] = {1,1,1};
sem_t sem_1, sem_2, sem_3;
t_point_cloud *pointCloud;

#define THREAD_PERIOD 100

#define ITERATIONS 500

struct timespec times[ITERATIONS+1];

static void *read_thread(void *arg){

    (void) arg;

    printf("thread attr:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    const struct timespec period = {0,THREAD_PERIOD*1e6};
    struct timespec next_time;

    clock_gettime(CLOCK_REALTIME, &(times[0]));

    int i = 0;

    add_timespec(&next_time,&period,&(times[0]));

    while(cond_th[0] && i < ITERATIONS+1){

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_time, NULL);

        read_point_cloud_sem(&pointCloud, "Data/point_cloud1.txt",&sem_3,&sem_1);

        clock_gettime(CLOCK_REALTIME, &(times[i]));

        add_timespec(&next_time,&period,&(next_time));
    
        i++;
    }

    cond_th[0] = 0; cond_th[1] = 0; cond_th[2] = 0;

    return NULL;

}

static void *remove_outliers_thread(void *arg){

    (void) arg;

    printf("thread attr:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    while(cond_th[1]){

        filter_point_cloud_sem(&pointCloud,&sem_1,&sem_2);
        
    }

    return NULL;
    
}

static void *filter_roads_thread(void *arg){

    (void) arg;

    printf("thread attr:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    while(cond_th[2]){

        filter_roads_sem(&pointCloud,12,&sem_2,&sem_3);
        
    }

    return NULL;
    
}

void catch_sig_int(int val){
    (void) val;
    for(int i = 0; i < 3; i++){
        cond_th[i] = 0;
    }

    if(pointCloud != NULL) free_t_point_cloud(pointCloud);

    sem_close(&sem_1); sem_close(&sem_2); sem_close(&sem_3);

    sem_destroy(&sem_1); sem_destroy(&sem_2); sem_destroy(&sem_3);

    exit(0);
}

int main(int argc, char *argv[]){
    (void) argc; (void) argv;

    signal(SIGTTIN, catch_sig_int);
    
    sem_init(&sem_1,0,0); sem_init(&sem_2,0,0); sem_init(&sem_3,0,1);
    if(&sem_1 == SEM_FAILED || &sem_2 == SEM_FAILED || &sem_3 == SEM_FAILED) perror("sem_init() error.");

    pointCloud = (t_point_cloud*) malloc(sizeof(t_point_cloud));

    pthread_t thr1, thr2, thr3;

    pthread_attr_t attr;

    thread_configs(&attr,12,SCHED_FIFO,0);

    printf("main_thread attr Changed:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    pthread_create(&thr1,&attr,read_thread, NULL);
    pthread_create(&thr2,&attr,remove_outliers_thread, NULL);
    pthread_create(&thr3,&attr,filter_roads_thread, NULL);

    pthread_join(thr1,NULL); pthread_join(thr2,NULL); pthread_join(thr3,NULL);

    pthread_attr_destroy(&attr);

    sem_destroy(&sem_1); sem_destroy(&sem_2); sem_destroy(&sem_3);

    int i;
    printf("Time table:\n");
    double max_time = -DBL_MIN;
    double d_time;
    for(i = 0; i < ITERATIONS; i++){
        d_time = dtime_ms(&(times[i+1]),&(times[i]));
        printf("\t%2i | delta_t = %9.3f (ms)\n",i+1,d_time);
        if(max_time < d_time) max_time = d_time;
    }
    printf("\tMax time: %9.3f (ms)\n",max_time);

    return 0;
    
}