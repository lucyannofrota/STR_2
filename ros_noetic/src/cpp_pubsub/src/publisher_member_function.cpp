#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// #include "sensor_msgs/point_cloud_conversion.hpp"


#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>
#include <math.h>

// using namespace std::chrono_literals;
using namespace std::placeholders;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

typedef struct {
    float *x, *y, *z;
    int npoints;
} t_point_cloud;


#define MAX_LINE_LEN (3-FLT_MIN_10_EXP+1)*3+10 // Tamanho maximo que uma linha pode ter. Assumindo 3xFloat + 10.
//(strlen("-0.")-FLT_MIN_10_EXP+1) Sendo o maior valor que uma variavel float pode assumir em ascii

// https://github.com/mikeferguson/ros2_cookbook
// https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    publisher_1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("Pcloud", 10);

    subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
    // this->create_subscription<sensor_msgs::msg::PointCloud2>()
    subscription_1 = this->create_subscription<sensor_msgs::msg::PointCloud2>("Pcloud", 10, 
                                std::bind(&MinimalPublisher::pc_callback, this, _1));

    char dataSTR[] = "src/cpp_pubsub/src/Data/point_cloud1_alt.txt";
    this->pointCloud1 = this->read_point_cloud(dataSTR);

    // msg.header.frame_id = std_msgs::msg::String("Base");

    this->msg.header.frame_id = "Points";
    this->msg.header.stamp = this->get_clock().get()->now();
    this->msg.height = 1;
    this->msg.width = this->pointCloud1->npoints;
    this->msg.point_step = (sizeof(float) * 3);
    this->msg.is_dense = true;
    this->msg.row_step = this->msg.width;
    this->msg.is_bigendian = false;
    this->msg.data.resize(msg.height*msg.width);

    sensor_msgs::PointCloud2Modifier mod(this->msg);

    mod.setPointCloud2FieldsByString(1,"xyz");
    mod.resize(msg.height*msg.width);

    // sensor_msgs::convertPointCloudToPointCloud2()



    sensor_msgs::PointCloud2Iterator<float> iter_x(this->msg,"x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(this->msg,"y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(this->msg,"z");

    int i = 0;
    printf("Size: (h: %i,w: %i)\n",msg.height,msg.width);
    while(iter_x != iter_x.end()){
    // while(i < this->pointCloud1->npoints){
      *iter_x = this->pointCloud1->x[i];
      printf("Def X: %f\n",this->pointCloud1->x[i]);
      *iter_y = this->pointCloud1->y[i];
      *iter_z = this->pointCloud1->z[i];
      printf("X: %f\n",*iter_x);
      ++iter_x; ++iter_y; ++iter_z; i++;
    }

    publisher_1->publish(msg);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&MinimalPublisher::timer_callback, this));
  }
  ~MinimalPublisher(){
    free_t_point_cloud(this->pointCloud1);
  }

private:
  void timer_callback()
  {

    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // publisher_->publish(message);
    publisher_1->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_1;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_1;
  size_t count_;
  t_point_cloud *pointCloud1;
  sensor_msgs::msg::PointCloud2 msg;

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const{
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg,"x");

    for(; iter_x != iter_x.end();++iter_x){
      printf("X val: %f\n",*iter_x);
      // printf("X val: %f\n",iter_x);
    }
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  t_point_cloud* read_point_cloud(char *file_name){
      t_point_cloud *ret_ptr = new t_point_cloud;
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
      ret_ptr->x = new float[ret_ptr->npoints];
      ret_ptr->y = new float[ret_ptr->npoints];
      ret_ptr->z = new float[ret_ptr->npoints];
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

  void free_t_point_cloud(t_point_cloud *ptr){
      free(ptr->x);
      free(ptr->y);
      free(ptr->z);
      free(ptr);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
