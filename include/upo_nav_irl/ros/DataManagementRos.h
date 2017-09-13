#ifndef UPO_IRL_DATA_ROS_
#define UPO_IRL_DATA_ROS_

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>
#include <time.h>
#include <dirent.h> //mkdir
#include <sys/types.h>
#include <sys/stat.h>
#include <numeric>      // std::accumulate


#include <ros/ros.h>

//TF
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//Bag datatypes
#include <geometry_msgs/PoseStamped.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

//rosbag 
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

//open a directory
#include <boost/filesystem.hpp>


using namespace std;


namespace upo_irl_ros
{
	class DataManagementRos
	{

		public:
		
			DataManagementRos(string dir);
			
			~DataManagementRos();

			bool loadData();

			inline int getDataSize() {
				return data_size_;
			};

			void setDataSetDir(string dir){
				data_dir_ = dir;
			};

			geometry_msgs::PoseStamped mapDataExists(int i);

			geometry_msgs::PoseStamped getRobotPose(int i);

			
			struct trajectory_t {
				vector<geometry_msgs::PoseStamped> goal_data;
				vector<upo_msgs::PersonPoseArrayUPO> people_data;
				vector<sensor_msgs::PointCloud2> obs_data;
				vector<geometry_msgs::PoseStamped> robot_data;
				vector<geometry_msgs::Twist> vel_data;
				vector<geometry_msgs::PoseStamped> robot_map_data;
			};

			trajectory_t getTrajectory(int i) {
				if(i>=0 && i<data_size_)
					return dataset_[i];
			};


			inline vector<geometry_msgs::PoseStamped> getGoalData(int i) {
				return dataset_[i].goal_data;
			}

			inline vector<upo_msgs::PersonPoseArrayUPO> getPeopleData(int i) {
				return dataset_[i].people_data;
			}

			inline vector<sensor_msgs::PointCloud2> getObsData(int i) {
				return dataset_[i].obs_data;
			}

			inline vector<geometry_msgs::PoseStamped> getPathData(int i) {
				return dataset_[i].robot_data;
			}
			
			inline vector<geometry_msgs::Twist> getVelData(int i) {
				return dataset_[i].vel_data;
			}

			inline vector<geometry_msgs::PoseStamped> getPathMapData(int i) {
				return dataset_[i].robot_map_data;
			}

 
		private:

			string					data_dir_;

			vector<trajectory_t>	dataset_;

			int						data_size_;


	};
	
}
#endif


