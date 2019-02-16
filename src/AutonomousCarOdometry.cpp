#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include <math.h>
#include <string>
#include <vector>

ros::Time messageTimestamp;

float hingeOfFrontLeft = 0;
float hingeOfFrontRight = 0;
float encoderOfFrontLeftWheel = 0;
float encoderOfFrontRightWheel = 0;
float encoderOfRearLeftWheel = 0;
float encoderOfRearRightWheel = 0;

double current_x = 0;
double current_y = 0;
double current_th = 0;

double prev_left = 0;
double prev_right = 0;
ros::Time prev_time;

double current_v = 0;
double current_w = 0;

double r_rear = 0.215;
double ticksPerMeter = 1/(2*M_PI*r_rear/24); // 1 / (2 * pi * rear_radius / tic_per_turn)
double axle_length = 1.8;


bool update = false;
bool first_data_came = false;

int seq = 0;

int getIndex(std::vector<std::string> names, std::string name) {
  for(size_t index = 0; index < name.size(); index++) {
    if(name.compare(names.at(index)) == 0) {
      return index;
    }
  }
  return -1;
}

void callbackSubscriber(const sensor_msgs::JointStateConstPtr& message) {
  messageTimestamp = ros::Time::now();

  int indexOfFrontLeftHinge = getIndex(message->name, "front_left_hinge_joint");
  int indexOfFrontRightHinge = getIndex(message->name, "front_right_hinge_joint");
  int indexOfFrontLeftWheel = getIndex(message->name, "front_left_wheel_joint");
  int indexOfFrontRightWheel = getIndex(message->name, "front_right_wheel_joint");
  int indexOfRearLeftWheel = getIndex(message->name, "rear_right_wheel_joint");
  int indexOfRearRightWheel = getIndex(message->name, "rear_right_wheel_joint");


  if(indexOfFrontLeftWheel != -1 && indexOfFrontRightWheel != -1 && indexOfRearLeftWheel != -1 && indexOfRearRightWheel != -1) {
    hingeOfFrontLeft = message->position[indexOfFrontLeftHinge];
    hingeOfFrontRight = message->position[indexOfFrontRightHinge];

    encoderOfFrontLeftWheel = message->position[indexOfFrontLeftWheel];
    encoderOfFrontRightWheel = message->position[indexOfFrontLeftWheel];
    encoderOfRearLeftWheel = message->position[indexOfFrontLeftWheel];
    encoderOfRearRightWheel = message->position[indexOfRearRightWheel];

    encoderOfFrontLeftWheel = (encoderOfFrontLeftWheel / (2*M_PI)) * (24);
    encoderOfFrontRightWheel = (encoderOfFrontRightWheel / (2*M_PI)) * (24);
    encoderOfRearLeftWheel = (encoderOfRearLeftWheel / (2*M_PI)) * (24);
    encoderOfRearRightWheel = (encoderOfRearRightWheel / (2*M_PI)) * (24);


    if(!first_data_came){
        prev_left = encoderOfRearLeftWheel;
        prev_right = encoderOfRearRightWheel;
        prev_time = messageTimestamp;

        first_data_came = true;

    }
    else{
        double d_left = encoderOfRearLeftWheel - prev_left;
        double d_right = encoderOfRearRightWheel - prev_right;

        double d = 0.5*(d_left + d_right);
        double d_diff = d_right - d_left;
        double t_diff = (messageTimestamp - prev_time).toSec();


        double dr = d / ticksPerMeter;
        double dw = d_diff / ticksPerMeter / r_rear;


        current_x += dr * cos(current_th + dw/2);
        current_y += dr * sin(current_th + dw/2);
        current_th += dw;

        current_v = dr / t_diff;
        current_w = dw * axle_length / t_diff;

        prev_left = encoderOfRearLeftWheel;
        prev_right = encoderOfRearRightWheel;
        prev_time = ros::Time::now();

    }

    if(encoderOfFrontLeftWheel != 0 || encoderOfFrontRightWheel != 0 || encoderOfRearLeftWheel != 0 || encoderOfRearRightWheel != 0) {
      update = true;
    }
  }
}


int main(int argc, char **argv) {
  ros::init(argc,argv,"AutonomousCarOdometry");

  ros::NodeHandle node;

  messageTimestamp = ros::Time::now();

  ros::Subscriber subscriber = node.subscribe<sensor_msgs::JointState>("/autonomous_car/joint_states",1,callbackSubscriber);

  ros::Publisher encoderFrontPublisher = node.advertise<std_msgs::Int32>("autonomous_car/encoder/front",1);
  ros::Publisher encoderRearPublisher = node.advertise<std_msgs::Int32>("autonomous_car/encoder/rear",1);
  ros::Publisher timestampPublisher = node.advertise<std_msgs::Int32>("autonomous_car/encoder/timestamp",1);

  ros::Publisher hingeLeftPublisher = node.advertise<std_msgs::Float32>("autonomous_car/hinge/left",1);
  ros::Publisher hingeRightPublisher = node.advertise<std_msgs::Float32>("autonomous_car/hinge/right",1);
  ros::Publisher odomPublisher = node.advertise<nav_msgs::Odometry>("autonomous_car/odom",1);

  ros::Rate loop(20);
  while(ros::ok()) {

    if(update == true) {

      std_msgs::Float32 hingeLeftMessage;
      hingeLeftMessage.data = hingeOfFrontLeft;
      hingeLeftPublisher.publish(hingeLeftMessage);

      std_msgs::Float32 hingeRightMessage;
      hingeLeftMessage.data = hingeOfFrontRight;
      hingeRightPublisher.publish(hingeRightMessage);

      nav_msgs::Odometry odomMessage;

      odomMessage.header.seq = seq;
      seq++;
      odomMessage.header.frame_id = "odom";
      odomMessage.header.stamp = ros::Time::now();

      odomMessage.pose.pose.position.x = current_x;
      odomMessage.pose.pose.position.y = current_y;
      odomMessage.pose.pose.position.z = 0;

      odomMessage.pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_th);

      odomMessage.twist.twist.angular.z = current_w;
      odomMessage.twist.twist.linear.x = current_v;

      odomPublisher.publish(odomMessage);

      update = false;
    }

    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}

