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

// static void *read_thread(void *arg){

//     arg = NULL;

//     // struct timespec (*tab)[N_FUNCTIONS][N_SAMPLES] = arg;

//     printf("thread attr:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

//     while(cond_th[0]){
//         printf("1");
//         clk_wait(5);
//     }

//     return NULL;
// }

// static void *remove_outliers_thread(void *arg){

//     arg = NULL;

//     // struct timespec (*tab)[N_FUNCTIONS][N_SAMPLES] = arg;

//     printf("thread attr:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

//     while(cond_th[1]){
//         printf("2");
//         clk_wait(5);
//     }

//     return NULL;
// }

// static void *filter_roads_thread(void *arg){

//     arg = NULL;

//     // struct timespec (*tab)[N_FUNCTIONS][N_SAMPLES] = arg;

//     printf("thread attr:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

//     while(cond_th[0]){
//         printf("3");
//         clk_wait(5);
//     }

//     return NULL;
// }

void catch_sig_int(int val){
    val += 0;
    for(int i = 0; i < 3; i++){
        cond_th[i] = 0;
    }

    if(pointCloud != NULL) free_t_point_cloud(pointCloud);
    sem_close(&sem_1); sem_close(&sem_2); sem_close(&sem_3);

    exit(0);
}

int main(int argc, char *argv[]){
    if(argc > 1){
        printf("ArgV: %s\n",*argv);
    }

    // signal(SIGTTIN, catch_sig_int);
    
    // sem_init(&sem_1,0,0); sem_init(&sem_2,0,0); sem_init(&sem_2,0,1); 
    // if(&sem_1 == SEM_FAILED || &sem_2 == SEM_FAILED || &sem_3 == SEM_FAILED) perror("sem_open() error.");

    pointCloud = (t_point_cloud*) malloc(sizeof(t_point_cloud));

    // pthread_t thr1, thr2, thr3;

    // printf("main_thread attr Changed:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    // pthread_create(&thr1,NULL,read_thread, NULL);
    // pthread_create(&thr2,NULL,remove_outliers_thread, NULL);
    // pthread_create(&thr3,NULL,filter_roads_thread, NULL);

    // pthread_join(thr1,NULL); pthread_join(thr2,NULL); pthread_join(thr3,NULL);


    // for(int i = 0; i < 3; i++){

    read_point_cloud(&pointCloud, "Data/point_cloud1.txt");

    describe_point_cloud(pointCloud);

    filter_point_cloud(&pointCloud);

    describe_point_cloud(pointCloud);

    filter_roads(&pointCloud,12);

    describe_point_cloud(pointCloud);

    // }

    return 0;
}