#include <cstdio>

#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "rclcpp/qos.hpp"

#include "../include/str_ROS2/str_functions.hpp"


#define THREAD_PERIOD 100

t_point_cloud *pointCloud;

sensor_msgs::msg::PointCloud2 msg_ptc;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_ptc;

bool runflag = false;

void send_ptc(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub, t_point_cloud *data,uint8_t r,uint8_t g,uint8_t b){
  msg_ptc.width = data->npoints;
  msg_ptc.row_step = msg_ptc.width;
  msg_ptc.data.resize(msg_ptc.height*msg_ptc.width);
  msg_ptc.header.stamp = rclcpp::Clock().now();

  sensor_msgs::PointCloud2Modifier mod(msg_ptc);

  mod.setPointCloud2FieldsByString(2,"xyz","rgb");
  mod.resize(msg_ptc.height*msg_ptc.width);

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg_ptc,"x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg_ptc,"y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg_ptc,"z");

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg_ptc,"r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg_ptc,"g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg_ptc,"b");


  int i = 0;
  while(iter_x != iter_x.end()){
    *iter_x = (float) data->x[i];
    *iter_y = (float) data->y[i];
    *iter_z = (float) data->z[i];

    *iter_r = (uint8_t) r;
    *iter_g = (uint8_t) g;
    *iter_b = (uint8_t) b;

    ++iter_x; ++iter_y; ++iter_z; i++;
    ++iter_r; ++iter_g; ++iter_b;
  }

  pub->publish(msg_ptc);
}

using std::placeholders::_1;

class pub_view : public rclcpp::Node{
  public:
  pub_view() : Node("pub_view"){
    

    // Initializing Publishers
    publisher_ptc = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_results", 10);
    publisher_marker = this->create_publisher<visualization_msgs::msg::Marker>("Origin",10);

    // Initializing Subscription
    subscription_velodyne = this->create_subscription<sensor_msgs::msg::PointCloud2>("velodyne_points",10,std::bind(&pub_view::PointCloud2_msg_to_t_point_cloud,this,_1));
    
    // Initializing Msgs
    msg_marker.header.frame_id = "velodyne";
    msg_marker.header.stamp = this->get_clock().get()->now();
    msg_marker.id = 0;
    msg_marker.type = visualization_msgs::msg::Marker::SPHERE;
    msg_marker.action = visualization_msgs::msg::Marker::ADD;
    msg_marker.pose.position.x = 0; msg_marker.pose.position.y = 0; msg_marker.pose.position.z = 0;
    msg_marker.pose.orientation.x = 0; msg_marker.pose.orientation.y = 0; msg_marker.pose.orientation.z = 0; msg_marker.pose.orientation.w = 0;
    msg_marker.scale.x = 2.5; msg_marker.scale.y = 2.5; msg_marker.scale.z = 2.5;
    msg_marker.color.r = 1.0; msg_marker.color.g = 0.0; msg_marker.color.b = 0.0; msg_marker.color.a = 0.5;


    msg_ptc.header.frame_id = "velodyne";
    msg_ptc.header.stamp = this->get_clock().get()->now();
    msg_ptc.height = 1;
    msg_ptc.point_step = (sizeof(float) * 3);
    msg_ptc.is_dense = true;
    msg_ptc.is_bigendian = false;


    pointCloud = (t_point_cloud*) malloc(sizeof(t_point_cloud));
    pointCloud->npoints = 0;
    pointCloud->x = NULL;
    pointCloud->y = NULL;
    pointCloud->z = NULL;


    timer = this->create_wall_timer(
      std::chrono::milliseconds(5000), std::bind(&pub_view::timer_callback, this));


  }
  ~pub_view(){
    free_t_point_cloud(pointCloud);
  }

  

  private:
  // Timer
  rclcpp::TimerBase::SharedPtr timer;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_marker;

  // Msgs
  visualization_msgs::msg::Marker msg_marker;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_velodyne;

  void PointCloud2_msg_to_t_point_cloud(sensor_msgs::msg::PointCloud2::SharedPtr shr_msg){
    sensor_msgs::PointCloud2Iterator<float> iter_x(*(shr_msg),"x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*(shr_msg),"y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*(shr_msg),"z");


    pointCloud->npoints = shr_msg.get()->width;
    free(pointCloud->x);
    pointCloud->x = (double*) malloc(sizeof(double)*pointCloud->npoints);
    free(pointCloud->y);
    pointCloud->y = (double*) malloc(sizeof(double)*pointCloud->npoints);
    free(pointCloud->z);
    pointCloud->z = (double*) malloc(sizeof(double)*pointCloud->npoints);

    int i = 0;
    while(iter_x != iter_x.end()){
      pointCloud->x[i] = (double) *iter_x;
      pointCloud->y[i] = (double) *iter_y;
      pointCloud->z[i] = (double) *iter_z;

      ++iter_x; ++iter_y; ++iter_z; i++;
    }

    runflag = true;
  }

  void timer_callback()
  {
    msg_marker.action = visualization_msgs::msg::Marker::MODIFY;
    publisher_marker->publish(msg_marker);
  }

};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<pub_view>();
  rclcpp::Rate rate(100.0);
  while(rclcpp::ok()){
    if(runflag){
      filter_point_cloud(&pointCloud);
      filter_roads(&pointCloud,12);
      send_ptc(publisher_ptc,pointCloud,255,255,255);
      runflag = false;
    }
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}
