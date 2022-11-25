#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "aux_libs/str_functions.h"

int main(int argc, char *argv[]){
    if(argc > 1){
        printf("ArgV: %s\n",*argv);
    }

    t_point_cloud *pointCloud1;

    read_point_cloud(&pointCloud1, "Data/point_cloud1.txt");

    describe_point_cloud(pointCloud1);

    free_t_point_cloud(pointCloud1);
    return 0;
}