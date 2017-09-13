#ifndef UPO_IRL_PLANNING_ROS_
#define UPO_IRL_PLANNING_ROS_


#include <ros/ros.h>

//Bag datatypes
#include <geometry_msgs/PoseStamped.h>
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

//TF
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//Visualization
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//Service messages to call
#include <upo_social_layer/Features.h>
#include <upo_social_layer/Cost.h>
#include <upo_social_layer/SetGoal.h>
#include <upo_social_layer/SetDemoPath.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>


//ROS services
#include <nav_msgs/GetPlan.h> 					//A* planning
#include <upo_rrt_planners/MakePlan.h>  		//RRT* planning
#include <navigation_features/SetLossCost.h>	//Enable/Disable loss function for RLT and MMP algorithms
#include <navigation_features/InitWeights.h>
#include <navigation_features/SetWeights.h>
#include <navigation_features/GetFeatureCount.h>


#include <upo_nav_irl/Planning.h>

#include <upo_nav_irl/ros/DataManagementRos.h>


using namespace std;


namespace upo_irl_ros
{
	class PlanningRos : public upo_irl::Planning
	{
		public:
			PlanningRos(string data_dir);
			PlanningRos(string data_dir, bool fcn);
			~PlanningRos();

			/*struct trajectory_t {
				vector<geometry_msgs::PoseStamped> goal_data;
				vector<upo_msgs::PersonPoseArrayUPO> people_data;
				vector<sensor_msgs::PointCloud2> obs_data;
				vector<geometry_msgs::PoseStamped> robot_data;
				vector<geometry_msgs::Twist> vel_data;
				vector<geometry_msgs::PoseStamped> robot_map_data;
			};*/


			bool loadData();

			//int getFeatSize();

			int getDataSize();

			void setScenario(int i);

			void setWeights(vector<float> w);

			vector<float> initializeWeights(bool random, bool normalize);

			//path_point getGoal(int i);

			vector<float> makePlanAndCount(int i, int planner_type);

			vector<float> getFeatureCountDemoPath(int i);

			vector<float> getFeatureCountPath(vector<geometry_msgs::PoseStamped>* mypath);

			vector< vector<float> > calculateFeatureCountDemos(int type);

			vector<geometry_msgs::PoseStamped> makePlan(int i, int planner_type);	

			void enableLossFunction(bool e, int i);

			void publish_robot_position(int i);	

			void visualize_demo_path(vector<geometry_msgs::PoseStamped> path);
			void visualize_people(upo_msgs::PersonPoseArrayUPO peop);	

			geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped pose_in, string frame_out, bool usetime);
			bool isQuaternionValid(geometry_msgs::Quaternion q);
			vector<geometry_msgs::PoseStamped> pathAdaptation(std::vector<geometry_msgs::PoseStamped> path);


			vector<geometry_msgs::PoseStamped> getGoalData(int i) {
				return data_->getGoalData(i);
			}

			vector<upo_msgs::PersonPoseArrayUPO> getPeopleData(int i) {
				return data_->getPeopleData(i);
			}

			vector<sensor_msgs::PointCloud2> getObsData(int i) {
				return data_->getObsData(i);
			}

			vector<geometry_msgs::PoseStamped> getPathData(int i) {
				return data_->getPathData(i);
			}
			
			vector<geometry_msgs::Twist> getVelData(int i) {
				return data_->getVelData(i);
			}

			geometry_msgs::PoseStamped getRobotPose(int i) {
				return data_->getRobotPose(i);
			}

			


		private:

			DataManagementRos*		data_;

			tf::TransformBroadcaster* tf_broadcaster_;
			tf::TransformListener*	tf_;

			bool					path_adapt_;

			std::string 			base_frame_id_;
			std::string 			odom_frame_id_;
			std::string 			map_frame_id_;


			//ROS Publishers
			ros::NodeHandle			nh_;
			ros::Publisher			goal_pub_;
			ros::Publisher			people_pub_;
			ros::Publisher			obs_pub_;
			ros::Publisher 			odom_pub_; 
			//ROS publishers for visualization
			ros::Publisher			demo_path_pub_;
			ros::Publisher			people_arrow_pub_;
			ros::Publisher			people_cyl_pub_;

			
			//ROS Service clients
			ros::ServiceClient		astar_client_;
			ros::ServiceClient		rrt_client_;
			ros::ServiceClient		rrt_client_fcn_;
			ros::ServiceClient		features_client_;
			ros::ServiceClient		setWeights_client_;
			ros::ServiceClient		loss_client_;	
			ros::ServiceClient		init_w_client_;

			bool					fcn_;

	};
	
}
#endif
