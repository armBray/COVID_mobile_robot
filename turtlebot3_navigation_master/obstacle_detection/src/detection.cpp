#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "obstacle_detection/Obstacle.h"

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);


obstacle_detection::Obstacle ob;
std_msgs::Float32 msg;


void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {

  float inf = 10000;

  for(int i = 0; i < scan->ranges.size(); i++) {
    if ((scan->ranges[i] <= inf) && (scan->ranges[i] >= scan->range_min)){
      inf = scan->ranges[i];
      ob.direction = i;
      ob.distance = inf;
      }
   }

}

int main(int argc, char **argv) {


  ros::init(argc,argv,"Obstacles_sensing");

  ros::NodeHandle n;

  ros::Publisher berto = n.advertise<obstacle_detection::Obstacle>("/berto", 1000);
  //ros::Publisher berto = n.advertise<std_msgs::Float32>("/berto", 1000);
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan",50,laserCallBack);

  while(ros::ok()) {
    berto.publish(ob);
    ros::spinOnce();
  }


  return 0;
}
