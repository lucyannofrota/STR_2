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

    t_point_cloud *pointCloud1 = NULL;

    for(j = 0; j < N_FUNCTIONS; j++){
        for(i = 0; i < N_SAMPLES; i++){
            if(j == 0){
                clock_gettime(CLOCK_REALTIME, &time1);
                read_point_cloud(&pointCloud1, "Data/point_cloud1.txt");
                clock_gettime(CLOCK_REALTIME, &time2);
                if(pointCloud1 != NULL && i != N_SAMPLES-1) free_t_point_cloud(pointCloud1);
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
    if(argc > 1){
        printf("ArgV: %s\n",*argv);
    }

    // t_point_cloud *pointCloud1 = NULL;

    // read_point_cloud(&pointCloud1, "Data/point_cloud1.txt");

    // describe_point_cloud(pointCloud1);

    // filter_point_cloud(&pointCloud1);

    // describe_point_cloud(pointCloud1);

    // if(pointCloud1 != NULL) free_t_point_cloud(pointCloud1);





    printf("Process PID: %i\n",getpid());

    // Verificando atributos iniciais do thread main

    // int s;

    // pthread_t main_thread = pthread_self();

    // cpu_set_t cpu_set;


    // Alterando atributos do thread main

    // CPU_ZERO(&cpu_set);

    // CPU_SET(0,&cpu_set);

    printf("##################\n");
    printf("##Thread configs##\n");
    printf("##################\n\n\n");

    printf("main_thread attr Initial:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    // s = pthread_setaffinity_np(main_thread,sizeof(cpu_set), &cpu_set);
    // if (s != 0) handle_error_en(s, "pthread_setaffinity_np");

    // pthread_attr_t attr;
    
    // thread_configs(&attr,1,SCHED_FIFO,0);

    pthread_t thr1;

    printf("main_thread attr Changed:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    struct timespec time_table[N_FUNCTIONS][N_SAMPLES];

    calc_func_ripple(time_table);

    // pthread_create(&thr1,&attr,thread_start, time_table);
    pthread_create(&thr1,NULL,thread_start, time_table);

    // s = pthread_attr_destroy(&attr);
    // if (s != 0) handle_error_en(s, "pthread_attr_destroy");

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