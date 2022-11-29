#include <cstdio>

#include <stdlib.h>

// #include "str_ROS2/str_functions.hpp"

#include "../include/str_ROS2/str_functions.hpp"

sem_t sem_1, sem_2, sem_3;

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  t_point_cloud *pointCloud1 = (t_point_cloud*) malloc(sizeof(t_point_cloud));

  char str[] = "src/str_ROS2/Data/point_cloud1.txt";

  sem_init(&sem_1,0,0); sem_init(&sem_2,0,0); sem_init(&sem_3,0,1);
  // if(&sem_1 == SEM_FAILED || &sem_2 == SEM_FAILED || &sem_3 == SEM_FAILED) perror("sem_init() error.");

  read_point_cloud_sem(&pointCloud1, str,&sem_3,&sem_1);

  describe_point_cloud(pointCloud1);

  filter_point_cloud_sem(&pointCloud1,&sem_1,&sem_2);

  describe_point_cloud(pointCloud1);

  filter_roads_sem(&pointCloud1,12,&sem_2,&sem_3);

  describe_point_cloud(pointCloud1);

  if(pointCloud1 != NULL) free_t_point_cloud(pointCloud1);

  printf("asd 3\n");
  return 0;
}
