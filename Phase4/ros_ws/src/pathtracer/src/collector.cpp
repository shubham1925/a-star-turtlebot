
#include <ros/ros.h>
#include <fstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include "spawnCollect.hpp"
#include "randomizer.hpp"
#include "collector.hpp"
#include <string>
#include <time.h>

typedef actionlib::SimpleActionClient
<move_base_msgs::MoveBaseAction> MoveBaseClient;

Collector::Collector() {
}
Collector::~Collector() {
}

std::vector<std::pair<double,double>> Collector::readTextFile() {
  std::string line;
  std::ifstream infile;
  std::ifstream count;
  std::vector< std::pair <double,double> > path; 
  std::string a;
  std::string temp; 
  double p;
  double q;

  infile.open ("/home/rajshinde/Documents/mew/src/pathtracer/waypoints.txt");
int coun=1;
  do{
          getline(infile,line);
            p=std::stoi(line.substr(2,1));
            p=p+(0.01*std::stoi(line.substr(4,2)));
            if(line.substr(1,1)=="-")
            {
              p=-1*p;
            }
            ROS_INFO_STREAM("Line="<<p);

            q=std::stoi(line.substr(8,1));
            q=q+(0.01*std::stoi(line.substr(10,2)));
            if(line.substr(7,1)=="-")
            {
              q=-1*q;
            }
            ROS_INFO_STREAM("Line="<<q);

            path.push_back(std::make_pair(p,q));
      coun++;
       }while(infile.good());
  infile.close();
  return path;
}

bool Collector::collector() {

std::vector< std::pair <double,double> > path=readTextFile();
double xn=0;
double yn=0;

  MoveBaseClient ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(10.0))) {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  int i=path.size()-1;
  while(i!=-1)
  {
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = -(4+path[i].first);
  goal.target_pose.pose.position.y = -(3+path[i].second);
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  ROS_INFO_STREAM("x="<<path[i].first);
  ROS_INFO_STREAM("y="<<path[i].second);


  ROS_INFO_STREAM("Sending goal");
  ac.sendGoal(goal);
  

  i--;
  while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
  }
  ROS_INFO_STREAM("Waypoint");
  }



 ROS_INFO_STREAM("Goal Reached");

return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_navigation_goals");
  Collector trash;
  trash.collector();
  return 0;
}
