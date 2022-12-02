#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <unistd.h>

#include "aux_libs/str_functions.h"

#include "aux_libs/comp_defines.h"

void calc_func_ripple(struct timespec dtime_spec[N_FUNCTIONS][N_SAMPLES]){
    int i,j;
    struct timespec time1 = {0,0}, time2 = {0,0};

    t_point_cloud *pointCloud1 = (t_point_cloud*) malloc(sizeof(t_point_cloud));

    for(j = 0; j < N_FUNCTIONS; j++){
        for(i = 0; i < N_SAMPLES; i++){
            if(j == 0){
                clock_gettime(CLOCK_REALTIME, &time1);
                read_point_cloud(&pointCloud1, "Data/point_cloud1.txt");
                clock_gettime(CLOCK_REALTIME, &time2);
            }
            if(j == 1){
                clock_gettime(CLOCK_REALTIME, &time1);
                filter_point_cloud(&pointCloud1);
                clock_gettime(CLOCK_REALTIME, &time2);
            }
            if(j == 2){
                clock_gettime(CLOCK_REALTIME, &time1);
                filter_roads(&pointCloud1,12);
                clock_gettime(CLOCK_REALTIME, &time2);
            }
            sub_timespec(&time2,&time1,&dtime_spec[j][i]);
            clk_wait(5); // 5 ms
        }
    }
    if(pointCloud1 != NULL) free_t_point_cloud(pointCloud1);
}


static void *thread_start(void *arg){

    struct timespec (*tab)[N_FUNCTIONS][N_SAMPLES] = arg;

    printf("thread attr:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    calc_func_ripple(*tab);

    return NULL;
}

int main(int argc, char *argv[]){
    (void) argc; (void) argv;





    printf("Process PID: %i\n",getpid());


    printf("##################\n");
    printf("##Thread configs##\n");
    printf("##################\n\n\n");

    printf("main_thread attr Initial:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    pthread_t thr1;

    printf("main_thread attr Changed:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    struct timespec time_table[N_FUNCTIONS][N_SAMPLES];

    pthread_create(&thr1,NULL,thread_start, time_table);


    // Aguardando o termino do thread
    pthread_join(thr1,NULL);

    printf("###### 1 ######\n");

    printf("\n\t###########\n");
    printf("\t##Results##\n");
    printf("\t###########\n\n\n");

    print_table((struct timespec*)time_table,N_FUNCTIONS,N_SAMPLES,"\t");

    printf("Ending main thread\n");    

    return 0;
}