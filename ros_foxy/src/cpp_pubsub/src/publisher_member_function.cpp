#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"


#include <visualization_msgs/msg/marker.hpp>

// #include "ros_foxy/src/cpp_pubsub/src/publisher_member_function.cpp"
// #include "../../../../aux_libs/str_functions.h"

// #include "../include/cpp_pubsub/str_functions.h"


#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>
#include <math.h>

typedef struct {
    double *x, *y, *z;
    int npoints;
} t_point_cloud;


#define MAX_LINE_LEN (3-DBL_MIN_10_EXP+1)*3+10 // Tamanho maximo que uma linha pode ter. Assumindo 3xDLOUBLE + 10.
//(strlen("-0.")-DBL_MIN_10_EXP+1) Sendo o maior valor que uma variavel double pode assumir em ascii

void read_point_cloud(t_point_cloud **ptr, char *file_name){
    // printf("File name: %s\n",file);
    // t_point_cloud *ret_ptr = (t_point_cloud*) malloc(sizeof(t_point_cloud));
    // (*ptr)->npoints
    (*ptr) = (t_point_cloud*) malloc(sizeof(t_point_cloud));
    (*ptr)->npoints = 0;
    (*ptr)->x = NULL;
    (*ptr)->y = NULL;
    (*ptr)->z = NULL;
    
    FILE* file = NULL;
    if(file == NULL) file = fopen(file_name, "r"); // Abre o ficheiro na primeira vez que é chamada
    else perror("Missing input file.");

    printf("File name: %s\n",file_name);
    int count = 0;
    char line_buffer[MAX_LINE_LEN];
    while(fgets(line_buffer,sizeof(line_buffer),file) != NULL) count++;
    fseek(file, 0, SEEK_SET);
    (*ptr)->npoints = count;
    (*ptr)->x = (double*) malloc(sizeof(double)*(*ptr)->npoints);
    (*ptr)->y = (double*) malloc(sizeof(double)*(*ptr)->npoints);
    (*ptr)->z = (double*) malloc(sizeof(double)*(*ptr)->npoints);
    char *ptr_c;
    count = 0;
    while(fgets(line_buffer,sizeof(line_buffer),file) != NULL){
        printf("Buff: %s\n",line_buffer);
        ptr_c = strtok(line_buffer, " ");
        (*ptr)->x[count] = atof(ptr_c);
        ptr_c = strtok(NULL, " ");
        (*ptr)->y[count] = atof(ptr_c);
        ptr_c = strtok(NULL, " ");
        (*ptr)->z[count] = atof(ptr_c);
        count++;
    }

    printf("Count: %i\n",count);
    fclose(file);
    // return ret_ptr;
}

void describe_point_cloud(t_point_cloud *ptr){
    double max[3] = {ptr->x[0],ptr->y[0],ptr->z[0]}, min[3] = {ptr->x[0],ptr->y[0],ptr->z[0]}, mean[3] = {0,0,0}, std[3] = {0,0,0};

    for(int i = 0; i < ptr->npoints; i++){
        // printf("Pt: [%10.6f|%10.6f|%10.6f]\n",ptr->x[i],ptr->y[i],ptr->z[i]);
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

void filter_point_cloud(t_point_cloud **ptr){
    const int n_rules = 3;
    int i, count = 0;
    int *valid_pts = (int*) malloc(sizeof(int)*(*ptr)->npoints);

    // looking for valid poiunts
    for(i = 0; i < (*ptr)->npoints; i++){
        valid_pts[i] = n_rules;

        // (a) remove x < 0 points
        if((*ptr)->x[i] >= 0) valid_pts[i]--;

        // (b) remove r <= 1.6 points
        if(sqrt(pow((*ptr)->x[i],2) + pow((*ptr)->y[i],2) + pow((*ptr)->z[i],2)) > 1.6) valid_pts[i]--;

        // (c) remove z < 0 points
        if((*ptr)->z[i] <= -0.3) valid_pts[i]--;


        if(!valid_pts[i]) count++;
    }

    // redefining the point_cloud
    int aux_count = 0;
    // double *aux_x, *aux_y, *aux_z;
    double *new_x = (double*) malloc(sizeof(double)*count); 
    double *new_y = (double*) malloc(sizeof(double)*count); 
    double *new_z = (double*) malloc(sizeof(double)*count); 
    
    // printf("ptr: %p\n",new_x);
    // printf("ptr: %p\n",new_y);
    // printf("ptr: %p\n",new_z);

    for(i = 0; i < (*ptr)->npoints; i++){
        if(!valid_pts[i]){
            new_x[aux_count] = (*ptr)->x[i];
            new_y[aux_count] = (*ptr)->y[i];
            new_z[aux_count] = (*ptr)->z[i];
            // printf("PT+: [%f,%f,%f]\n",new_x[aux_count],new_y[aux_count],new_z[aux_count]);
            // printf("PT_: [%f,%f,%f]\n",(*ptr)->x[aux_count],(*ptr)->y[aux_count],(*ptr)->z[aux_count]);
            aux_count++;
        }
    }
    printf("Old Size: %i\n",aux_count);
    (*ptr)->npoints = aux_count;
    free((*ptr)->x); free((*ptr)->y); free((*ptr)->z);
    (*ptr)->x = new_x; (*ptr)->y = new_y; (*ptr)->z = new_z;


    // for(i = 0; i < (*ptr)->npoints; i++){
    //   printf("PT+: [%f,%f,%f]\n",new_x[i],new_y[i],new_z[i]);
    //   printf("PT_: [%f,%f,%f]\n",(*ptr)->x[i],(*ptr)->y[i],(*ptr)->z[i]);
    // }



    printf("New Size: %i\n",count);

    free(valid_pts);
}





using namespace std::placeholders;


// https://github.com/mikeferguson/ros2_cookbook
class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("Pcloud", 10);
    publisher_2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("Pcloud_filt", 10);

    marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("Origin",10);


    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock().get()->now();
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0; marker.pose.position.y = 0; marker.pose.position.z = 0;
    marker.pose.orientation.x = 0; marker.pose.orientation.y = 0; marker.pose.orientation.z = 0; marker.pose.orientation.w = 0;
    marker.scale.x = 2.5; marker.scale.y = 2.5; marker.scale.z = 2.5;
    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 0.5;





    char dataSTR[] = "src/cpp_pubsub/src/Data/point_cloud1.txt";


    read_point_cloud(&this->pointCloud1,dataSTR);
    read_point_cloud(&this->pointCloud2,dataSTR);
    filter_point_cloud(&this->pointCloud2);
    describe_point_cloud(this->pointCloud2);

    // for(int i = 0; i < this->pointCloud2->npoints; i++){
    //   printf("PT: [%f,%f,%f]\n",pointCloud2->x[i],pointCloud2->y[i],pointCloud2->z[i]);
    // }

    ptc_msg(this->msg1,this->pointCloud1,0,100,0);
    ptc_msg(this->msg2,this->pointCloud2,255,0,200);

    publisher_1->publish(this->msg1);
    publisher_2->publish(this->msg2);
    marker_pub->publish(marker);



    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2000), std::bind(&MinimalPublisher::timer_callback, this));
  }
  ~MinimalPublisher(){
    free_t_point_cloud(this->pointCloud1);
  }

private:
  void timer_callback()
  {

    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker_pub->publish(marker);
    this->msg1.header.stamp = this->get_clock().get()->now();
    publisher_1->publish(this->msg1);
    this->msg2.header.stamp = this->get_clock().get()->now();
    publisher_2->publish(this->msg2);
  }
  void ptc_msg(sensor_msgs::msg::PointCloud2 &msg, t_point_cloud *data,uint8_t r,uint8_t g,uint8_t b){
    msg.header.frame_id = "map";
    msg.header.stamp = this->get_clock().get()->now();
    msg.height = 1;
    msg.width = data->npoints;
    msg.point_step = (sizeof(float) * 3);
    msg.is_dense = true;
    msg.row_step = msg.width;
    msg.is_bigendian = false;
    msg.data.resize(msg.height*msg.width);

    sensor_msgs::PointCloud2Modifier mod(msg);

    mod.setPointCloud2FieldsByString(2,"xyz","rgb");
    // mod.setPointCloud2Fields(6,
    //   "x",1,sensor_msgs::msg::PointField::FLOAT32,
    //   "y",1,sensor_msgs::msg::PointField::FLOAT32,
    //   "z",1,sensor_msgs::msg::PointField::FLOAT32,
    //   "r",1,sensor_msgs::msg::PointField::UINT8,
    //   "g",1,sensor_msgs::msg::PointField::UINT8,
    //   "b",1,sensor_msgs::msg::PointField::UINT8
    //   // "d",1,sensor_msgs::msg::PointField::FLOAT32
    // );
    // mod.setPointCloud2Fields(6,
    //   "x",1,sensor_msgs::msg::PointField::FLOAT32,
    //   "y",1,sensor_msgs::msg::PointField::FLOAT32,
    //   "z",1,sensor_msgs::msg::PointField::FLOAT32,
    //   "rgb",3,sensor_msgs::msg::PointField::FLOAT32
    //   // "d",1,sensor_msgs::msg::PointField::FLOAT32
    // );
    mod.resize(msg.height*msg.width);
    // mod.setPointCloud2FieldsByString()



    sensor_msgs::PointCloud2Iterator<float> iter_x(msg,"x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg,"y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg,"z");

    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg,"r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg,"g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg,"b");

    // sensor_msgs::PointCloud2Iterator<float> iter_d(msg,"d");

    int i = 0;
    while(iter_x != iter_x.end()){
      *iter_x = (float) data->x[i];
      *iter_y = (float) data->y[i];
      *iter_z = (float) data->z[i];

      *iter_r = (uint8_t) r;
      *iter_g = (uint8_t) g;
      *iter_b = (uint8_t) b;

      // *iter_d = (float) 0;

      ++iter_x; ++iter_y; ++iter_z; i++;
      ++iter_r; ++iter_g; ++iter_b; //++iter_d;
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_1;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_2;
  size_t count_;
  t_point_cloud *pointCloud1;
  t_point_cloud *pointCloud2;
  sensor_msgs::msg::PointCloud2 msg1;
  sensor_msgs::msg::PointCloud2 msg2;
  visualization_msgs::msg::Marker marker;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
