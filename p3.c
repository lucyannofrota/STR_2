#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "aux_libs/str_functions.h"

int main(int argc, char *argv[]){
    (void) argc; (void) argv;

    t_point_cloud *pointCloud1 = (t_point_cloud*) malloc(sizeof(t_point_cloud));

    read_point_cloud(&pointCloud1, "Data/point_cloud1.txt");

    describe_point_cloud(pointCloud1);

    filter_point_cloud(&pointCloud1);

    describe_point_cloud(pointCloud1);

    filter_roads(&pointCloud1,12);

    describe_point_cloud(pointCloud1);

    if(pointCloud1 != NULL) free_t_point_cloud(pointCloud1);

    return 0;
}