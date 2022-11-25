#include "ros/ros.h"
#include "std_msgs/String.h"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "sensor_msgs/msg/point_field.hpp"
// #include "sensor_msgs/point_cloud2_iterator.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>
#include <math.h>

typedef struct {
    float *x, *y, *z;
    int npoints;
} t_point_cloud;


#define MAX_LINE_LEN (3-FLT_MIN_10_EXP+1)*3+10 // Tamanho maximo que uma linha pode ter. Assumindo 3xFloat + 10.
//(strlen("-0.")-FLT_MIN_10_EXP+1) Sendo o maior valor que uma variavel float pode assumir em ascii

t_point_cloud* read_point_cloud(char *file_name){
    // printf("File name: %s\n",file);
    t_point_cloud *ret_ptr = new t_point_cloud;//malloc(sizeof(t_point_cloud));
    ret_ptr->npoints = 0;
    ret_ptr->x = NULL;
    ret_ptr->y = NULL;
    ret_ptr->z = NULL;
    
    FILE* file = NULL;
    if(file == NULL) file = fopen(file_name, "r"); // Abre o ficheiro na primeira vez que é chamada
    else perror("Missing input file.");

    printf("File name: %s\n",file_name);
    int count = 0;
    char line_buffer[MAX_LINE_LEN];
    while(fgets(line_buffer,sizeof(line_buffer),file) != NULL) count++;
    fseek(file, 0, SEEK_SET);
    ret_ptr->npoints = count;
    ret_ptr->x = new float[ret_ptr->npoints];// malloc(sizeof(double)*ret_ptr->npoints);
    ret_ptr->y = new float[ret_ptr->npoints];// malloc(sizeof(double)*ret_ptr->npoints);
    ret_ptr->z = new float[ret_ptr->npoints];// malloc(sizeof(double)*ret_ptr->npoints);
    char *ptr;
    count = 0;
    while(fgets(line_buffer,sizeof(line_buffer),file) != NULL){
        printf("Buff: %s\n",line_buffer);
        ptr = strtok(line_buffer, " ");
        ret_ptr->x[count] = atof(ptr);
        ptr = strtok(NULL, " ");
        ret_ptr->y[count] = atof(ptr);
        ptr = strtok(NULL, " ");
        ret_ptr->z[count] = atof(ptr);
        count++;
    }

    printf("Count: %i\n",count);
    fclose(file);
    return ret_ptr;
}

void describe_point_cloud(t_point_cloud *ptr){
    double max[3] = {ptr->x[0],ptr->y[0],ptr->z[0]}, min[3] = {ptr->x[0],ptr->y[0],ptr->z[0]}, mean[3] = {0,0,0}, std[3] = {0,0,0};

    for(int i = 0; i < ptr->npoints; i++){
        printf("Pt: [%10.6f|%10.6f|%10.6f]\n",ptr->x[i],ptr->y[i],ptr->z[i]);
        // Max
        if(ptr->x[i] > max[0]) max[0] = ptr->x[i];
        if(ptr->y[i] > max[1]) max[1] = ptr->y[i];
        if(ptr->z[i] > max[2]) max[2] = ptr->z[i];
        // Min
        if(ptr->x[i] < min[0]) min[0] = ptr->x[i];
        if(ptr->y[i] < min[1]) min[1] = ptr->y[i];
        if(ptr->z[i] < min[2]) min[2] = ptr->z[i];
        // Mean
        mean[0] += ptr->x[i];
        mean[1] += ptr->y[i];
        mean[2] += ptr->z[i];
    }
    mean[0] /= ptr->npoints;
    mean[1] /= ptr->npoints;
    mean[2] /= ptr->npoints;


    for(int i = 0; i < ptr->npoints; i++){
        std[0] = pow(ptr->x[i] - mean[0],2);
        std[1] = pow(ptr->y[i] - mean[1],2);
        std[2] = pow(ptr->z[i] - mean[2],2);
    }
    std[0] /= ptr->npoints;
    std[1] /= ptr->npoints;
    std[2] /= ptr->npoints;
    std[0] = sqrt(std[0]);
    std[1] = sqrt(std[1]);
    std[2] = sqrt(std[2]);



    printf("\n\nReport:");
    printf("\t      [         x,         y,         z]\n");
    printf("\tMax:  [%10.6f,%10.6f,%10.6f]\n",max[0],max[1],max[2]);
    printf("\tMin:  [%10.6f,%10.6f,%10.6f]\n",min[0],min[1],min[2]);
    printf("\tMean: [%10.6f,%10.6f,%10.6f]\n",mean[0],mean[1],mean[2]);
    printf("\tSTD:  [%10.6f,%10.6f,%10.6f]\n",std[0],std[1],std[2]);
}

void free_t_point_cloud(t_point_cloud *ptr){
    free(ptr->x);
    free(ptr->y);
    free(ptr->z);
    free(ptr);
}




void pointCloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg,"x");

  for(; iter_x != iter_x.end();++iter_x){
    printf("X val: %f\n",*iter_x);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("ptc", 1000);
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("ptc",1000,pointCloud_callback);
  // <std_msgs::String>("chatter", 1000);

  char dataSTR[] = "src/cpp_pubsub/src/Data/point_cloud1_alt.txt";
  t_point_cloud *pointCloud1 = read_point_cloud(dataSTR);

  sensor_msgs::PointCloud2 msg;

  msg.header.frame_id = "Points";
  msg.header.stamp = ros::Time::now();
  msg.height = 1;
  msg.width = pointCloud1->npoints;
  msg.point_step = (sizeof(float) * 3);
  msg.is_dense = true;
  msg.row_step = msg.width;
  msg.is_bigendian = false;
  msg.data.resize(msg.height*msg.width);

  sensor_msgs::PointCloud2Modifier mod(msg);

  mod.setPointCloud2FieldsByString(2,"xyz","rgb");
  mod.resize(msg.height*msg.width);


  sensor_msgs::PointCloud2Iterator<float> iter_x(msg,"x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg,"y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg,"z");

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg,"r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg,"g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg,"b");

  int i = 0;
  printf("Size: (h: %i,w: %i)\n",msg.height,msg.width);
  while(iter_x != iter_x.end()){
  // while(i < this->pointCloud1->npoints){
    *iter_x = pointCloud1->x[i];
    printf("Def X: %f\n",pointCloud1->x[i]);
    *iter_y = pointCloud1->y[i];
    *iter_z = pointCloud1->z[i];
    printf("X: %f\n",*iter_x);

    *iter_r = 255;
    *iter_g = 0;
    *iter_b = 0;
    ++iter_x; ++iter_y; ++iter_z; i++;
    ++iter_r; ++iter_g; ++iter_b;
  }

  pub.publish(msg);



  ros::Rate loop_rate(2);

  int count = 0;
  while (ros::ok())
  {

    // std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    // msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);

    // pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  free_t_point_cloud(pointCloud1);

  return 0;
}




