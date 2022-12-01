#include <cstdio>

#include <stdlib.h>

// #include "str_ROS2/str_functions.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "rclcpp/qos.hpp"

#include "../include/str_ROS2/str_functions.hpp"

// sem_t sem_1, sem_2, sem_3;

using std::placeholders::_1;

class pub_view : public rclcpp::Node{
  public:
  pub_view() : Node("pub_view"){
    

    // Initializing Publishers
    publisher_ptc = this->create_publisher<sensor_msgs::msg::PointCloud2>("Pcloud", 10);
    publisher_ptc_filt = this->create_publisher<sensor_msgs::msg::PointCloud2>("Pcloud_filt", 10);
    
    publisher_ptc_roads = this->create_publisher<sensor_msgs::msg::PointCloud2>("Pcloud_roads", 10);

    publisher_marker = this->create_publisher<visualization_msgs::msg::Marker>("Origin",10);

    // Initializing Subscription
    subscription_velodyne = this->create_subscription<sensor_msgs::msg::PointCloud2>("velodyne_points",10,std::bind(&pub_view::subscriber_callback,this,_1));
    
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
    // this->initialize_ptc_msg(msg_ptc,"")

    msg_ptc.header.frame_id = "velodyne";
    msg_ptc.header.stamp = this->get_clock().get()->now();
    msg_ptc.height = 1;
    msg_ptc.point_step = (sizeof(float) * 3);
    msg_ptc.is_dense = true;
    msg_ptc.is_bigendian = false;

    // char str[] = "src/str_ROS2/Data/point_cloud1.txt";
    pointCloud = (t_point_cloud*) malloc(sizeof(t_point_cloud));
    pointCloud->npoints = 0;
    pointCloud->x = NULL;
    pointCloud->y = NULL;
    pointCloud->z = NULL;
    // t_point_cloud *pointCloud = NULL;//(t_point_cloud*) malloc(sizeof(t_point_cloud));

    // read_point_cloud(&pointCloud,str);


    // this->send_ptc(publisher_ptc,pointCloud,0,100,0);


    // ptc_msg(this->msg1,this->pointCloud1,0,100,0);
    // ptc_msg(this->msg2,this->pointCloud2,255,0,200);
    // ptc_msg(this->msg3,this->pointCloud3,0,0,200);

    publisher_marker->publish(msg_marker);


    timer = this->create_wall_timer(
      std::chrono::milliseconds(2000), std::bind(&pub_view::timer_callback, this));


    sem_init(&sem_1,0,0); sem_init(&sem_2,0,0); sem_init(&sem_3,0,1); sem_init(&sem_4,0,0), sem_init(&sem_5,0,0), sem_init(&sem_6,0,1);

  }
  ~pub_view(){
    free_t_point_cloud(pointCloud);
  }



  private:
  // Timer
  rclcpp::TimerBase::SharedPtr timer;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_ptc;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_ptc_filt;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_ptc_roads;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_marker;

  // Msgs
  sensor_msgs::msg::PointCloud2 msg_ptc;
  sensor_msgs::msg::PointCloud2 msg_ptc_filt;
  sensor_msgs::msg::PointCloud2 msg_ptc_roads;
  visualization_msgs::msg::Marker msg_marker;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_velodyne;

  t_point_cloud *pointCloud;

  sem_t sem_1, sem_2, sem_3, sem_4, sem_5, sem_6;

  void subscriber_callback(sensor_msgs::msg::PointCloud2::SharedPtr shr_msg){
    // sem_post(&sem_3);
    this->PointCloud2_msgtot_point_cloud(pointCloud,shr_msg,&sem_3,&sem_1);
    // sem_wait(&sem_1);
    printf("Size Start: %i\n",pointCloud->npoints);
    // describe_point_cloud(pointCloud);
    this->send_ptc(publisher_ptc,pointCloud,0,150,0);


    // filter_point_cloud(&pointCloud);
    // filter_point_cloud_sem(&pointCloud,&sem_1,&sem_2);
    // printf("Size Filt: %i\n",pointCloud->npoints);
    // this->send_ptc(publisher_ptc_filt,pointCloud,0,0,150);
    sem_post(&sem_3);
    // sem_wait(&sem_1);






    // // filter_point_cloud_sem(&pointCloud,&sem_2,&sem_3);
    // filter_point_cloud(&pointCloud);
    // // sem_wait(&sem_3);
    // printf("Size Filt: %i\n",pointCloud->npoints);
    // describe_point_cloud(pointCloud);
    // this->send_ptc(publisher_ptc_filt,pointCloud,0,0,150);
    // // sem_post(&sem_4);

    // // filter_roads_sem(&pointCloud,12,&sem_4,&sem_5);
    // filter_roads(&pointCloud,12);
    // // sem_wait(&sem_5);
    // printf("Size Roads: %i\n",pointCloud->npoints);
    // describe_point_cloud(pointCloud);
    // this->send_ptc(publisher_ptc_roads,pointCloud,150,0,150);
    // sem_post(&sem_3);

    // free_t_point_cloud(pointCloud);
  }

  void PointCloud2_msgtot_point_cloud(t_point_cloud *ptc,sensor_msgs::msg::PointCloud2::SharedPtr &shr_msg,sem_t *sem_b,sem_t *sem_a){
    static sensor_msgs::PointCloud2Iterator<float> iter_x(*(shr_msg),"x");
    static sensor_msgs::PointCloud2Iterator<float> iter_y(*(shr_msg),"y");
    static sensor_msgs::PointCloud2Iterator<float> iter_z(*(shr_msg),"z");


    sem_wait(sem_b);
    pointCloud->npoints = shr_msg.get()->width;
    free(pointCloud->x);
    pointCloud->x = (double*) malloc(sizeof(double)*pointCloud->npoints);
    free(pointCloud->y);
    pointCloud->y = (double*) malloc(sizeof(double)*pointCloud->npoints);
    free(pointCloud->z);
    pointCloud->z = (double*) malloc(sizeof(double)*pointCloud->npoints);
    // printf("Width: %i\n",shr_msg.get()->width);

    int i = 0;
    while(iter_x != iter_x.end()){
      pointCloud->x[i] = (double) *iter_x;
      pointCloud->y[i] = (double) *iter_y;
      pointCloud->z[i] = (double) *iter_z;
      // printf("X: %f\n",*iter_x );
      // *iter_x = (float) data->x[i];
      // *iter_y = (float) data->y[i];
      // *iter_z = (float) data->z[i];

      ++iter_x; ++iter_y; ++iter_z; i++;
    }
    // describe_point_cloud(pointCloud);
    sem_post(sem_a);
  }

  void timer_callback()
  {
    msg_marker.action = visualization_msgs::msg::Marker::MODIFY;
    publisher_marker->publish(msg_marker);
    // marker.action = visualization_msgs::msg::Marker::MODIFY;
    // marker_pub->publish(marker);
    // this->msg1.header.stamp = this->get_clock().get()->now();
    // publisher_1->publish(this->msg1);
    // this->msg2.header.stamp = this->get_clock().get()->now();
    // publisher_2->publish(this->msg2);
    // this->msg3.header.stamp = this->get_clock().get()->now();
    // publisher_3->publish(this->msg3);
  }

  void initialize_ptc_msg(sensor_msgs::msg::PointCloud2 &msg){
    msg.header.frame_id = "map";
    msg.header.stamp = this->get_clock().get()->now();
    msg.height = 1;
    msg.point_step = (sizeof(float) * 3);
    msg.is_dense = true;
    msg.is_bigendian = false;

  }

  void send_ptc(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub, t_point_cloud *data,uint8_t r,uint8_t g,uint8_t b){
    msg_ptc.width = data->npoints;
    msg_ptc.row_step = msg_ptc.width;
    msg_ptc.data.resize(msg_ptc.height*msg_ptc.width);
    msg_ptc.header.stamp = this->get_clock().get()->now();

    static sensor_msgs::PointCloud2Modifier mod(msg_ptc);

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
    mod.resize(msg_ptc.height*msg_ptc.width);
    // mod.setPointCloud2FieldsByString()



    static sensor_msgs::PointCloud2Iterator<float> iter_x(msg_ptc,"x");
    static sensor_msgs::PointCloud2Iterator<float> iter_y(msg_ptc,"y");
    static sensor_msgs::PointCloud2Iterator<float> iter_z(msg_ptc,"z");

    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg_ptc,"r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg_ptc,"g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg_ptc,"b");

    // sensor_msgs::PointCloud2Iterator<float> iter_d(msg,"d");

    static int i = 0;
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

    pub->publish(msg_ptc);
  }

  // void ptc_msg(sensor_msgs::msg::PointCloud2 &msg, t_point_cloud *data,uint8_t r,uint8_t g,uint8_t b){
  //   msg.header.frame_id = "map";
  //   msg.header.stamp = this->get_clock().get()->now();
  //   msg.height = 1;
  //   msg.width = data->npoints;
  //   msg.point_step = (sizeof(float) * 3);
  //   msg.is_dense = true;
  //   msg.row_step = msg.width;
  //   msg.is_bigendian = false;
  //   msg.data.resize(msg.height*msg.width);

  //   sensor_msgs::PointCloud2Modifier mod(msg);

  //   mod.setPointCloud2FieldsByString(2,"xyz","rgb");
  //   // mod.setPointCloud2Fields(6,
  //   //   "x",1,sensor_msgs::msg::PointField::FLOAT32,
  //   //   "y",1,sensor_msgs::msg::PointField::FLOAT32,
  //   //   "z",1,sensor_msgs::msg::PointField::FLOAT32,
  //   //   "r",1,sensor_msgs::msg::PointField::UINT8,
  //   //   "g",1,sensor_msgs::msg::PointField::UINT8,
  //   //   "b",1,sensor_msgs::msg::PointField::UINT8
  //   //   // "d",1,sensor_msgs::msg::PointField::FLOAT32
  //   // );
  //   // mod.setPointCloud2Fields(6,
  //   //   "x",1,sensor_msgs::msg::PointField::FLOAT32,
  //   //   "y",1,sensor_msgs::msg::PointField::FLOAT32,
  //   //   "z",1,sensor_msgs::msg::PointField::FLOAT32,
  //   //   "rgb",3,sensor_msgs::msg::PointField::FLOAT32
  //   //   // "d",1,sensor_msgs::msg::PointField::FLOAT32
  //   // );
  //   mod.resize(msg.height*msg.width);
  //   // mod.setPointCloud2FieldsByString()



  //   sensor_msgs::PointCloud2Iterator<float> iter_x(msg,"x");
  //   sensor_msgs::PointCloud2Iterator<float> iter_y(msg,"y");
  //   sensor_msgs::PointCloud2Iterator<float> iter_z(msg,"z");

  //   sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg,"r");
  //   sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg,"g");
  //   sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg,"b");

  //   // sensor_msgs::PointCloud2Iterator<float> iter_d(msg,"d");

  //   int i = 0;
  //   while(iter_x != iter_x.end()){
  //     *iter_x = (float) data->x[i];
  //     *iter_y = (float) data->y[i];
  //     *iter_z = (float) data->z[i];

  //     *iter_r = (uint8_t) r;
  //     *iter_g = (uint8_t) g;
  //     *iter_b = (uint8_t) b;

  //     // *iter_d = (float) 0;

  //     ++iter_x; ++iter_y; ++iter_z; i++;
  //     ++iter_r; ++iter_g; ++iter_b; //++iter_d;
  //   }
  // }
};





#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <sys/wait.h>






int cond_th[3] = {1,1,1};
sem_t sem_1, sem_2, sem_3;
// sem_t sem_1, sem_2, sem_3, sem_4, sem_5;
t_point_cloud *pointCloud;

#define THREAD_PERIOD 100

// #define I_TIME_S 3

static void *read_thread(void *arg){

    (void) arg;


    // printf("thread attr:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    const struct timespec period = {0,THREAD_PERIOD*1000000};
    struct timespec next_time,currrent_time;

    clock_gettime(CLOCK_REALTIME, &(next_time));

    char str[] = "src/str_ROS2/Data/point_cloud1.txt";

    while(cond_th[0]){

        #if __VERBOSE
        clock_gettime(CLOCK_REALTIME, &(currrent_time));
        
        printf("Th1: %f\n",dtime_ms(&currrent_time,&next_time)*1000);// print_timespec(currrent_time,"\t");
        #endif

        add_timespec(&next_time,&period,&next_time);

        read_point_cloud_sem(&pointCloud, "src/str_ROS2/Data/point_cloud1.txt",&sem_3,&sem_1);

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_time, NULL);
        
    }

    return NULL;

}

static void *remove_outliers_thread(void *arg){

    (void) arg;

    // printf("thread attr:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    const struct timespec period = {0,THREAD_PERIOD*1000000};
    struct timespec next_time;

    clock_gettime(CLOCK_REALTIME, &(next_time));

    while(cond_th[1]){

        add_timespec(&next_time,&period,&next_time);

        filter_point_cloud_sem(&pointCloud,&sem_1,&sem_2);

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_time, NULL);
        
    }

    return NULL;
    
}

static void *filter_roads_thread(void *arg){

    (void) arg;

    // printf("thread attr:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    const struct timespec period = {0,THREAD_PERIOD*1000000};
    struct timespec next_time, currrent_time;

    clock_gettime(CLOCK_REALTIME, &(next_time));

    while(cond_th[2]){

        add_timespec(&next_time,&period,&next_time);

        filter_roads_sem(&pointCloud,12,&sem_2,&sem_3);

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_time, NULL);
        
    }

    return NULL;
    
}

void catch_sig_int(int val){

    val += 0;
    for(int i = 0; i < 3; i++){
        cond_th[i] = 0;
    }

    if(pointCloud != NULL) free_t_point_cloud(pointCloud);

    sem_close(&sem_1); sem_close(&sem_2); sem_close(&sem_3);

    sem_destroy(&sem_1); sem_destroy(&sem_2); sem_destroy(&sem_3);

    exit(0);
}



static void *test(void *arg){

    pthread_setschedprio(pthread_self(),sched_get_priority_max(SCHED_OTHER));

    printf("thread attr:\n"); display_thread_attr(pthread_self(), "\t"); printf("\n");

    return NULL;
    
}












int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;

  t_point_cloud *pointCloud1 = (t_point_cloud*) malloc(sizeof(t_point_cloud));

  char str[] = "src/str_ROS2/Data/point_cloud1.txt";

  sem_init(&sem_1,0,0); sem_init(&sem_2,0,0); sem_init(&sem_3,0,1);
  // if(&sem_1 == SEM_FAILED || &sem_2 == SEM_FAILED || &sem_3 == SEM_FAILED) perror("sem_init() error.");

  // read_point_cloud_sem(&pointCloud1, str,&sem_3,&sem_1);

  // describe_point_cloud(pointCloud1);

  // filter_point_cloud_sem(&pointCloud1,&sem_1,&sem_2);

  // describe_point_cloud(pointCloud1);

  // filter_roads_sem(&pointCloud1,12,&sem_2,&sem_3);

  // describe_point_cloud(pointCloud1);

  // if(pointCloud1 != NULL) free_t_point_cloud(pointCloud1);


  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pub_view>());
  rclcpp::shutdown();






























  // signal(SIGTTIN, catch_sig_int);
    
  // sem_init(&sem_1,0,0); sem_init(&sem_2,0,0); sem_init(&sem_3,0,1);

  // pointCloud = (t_point_cloud*) malloc(sizeof(t_point_cloud));

  // pthread_t thr1, thr2, thr3;

  // pthread_attr_t attr;

  // thread_configs(&attr,12,SCHED_FIFO,0);

  // pthread_create(&thr1,NULL,read_thread, NULL);
  // pthread_create(&thr2,NULL,remove_outliers_thread, NULL);
  // pthread_create(&thr3,NULL,filter_roads_thread, NULL);

  // pthread_attr_destroy(&attr);

  // sem_destroy(&sem_1); sem_destroy(&sem_2); sem_destroy(&sem_3);


  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<pub_view>());
  // rclcpp::shutdown();


  return 0;
}
