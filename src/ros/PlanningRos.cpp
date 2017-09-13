
#include <upo_nav_irl/ros/PlanningRos.h>



upo_irl_ros::PlanningRos::PlanningRos(string data_dir) : Planning()
{
	data_dir_ = data_dir;
	data_ = new DataManagementRos(data_dir_);

 	path_adapt_ = true;

	tf_ = new tf::TransformListener();

	fcn_ = false;

	base_frame_id_ = "/base_link";
	odom_frame_id_ = "/odom";
	map_frame_id_ = "/map";


	tf_broadcaster_ = new tf::TransformBroadcaster();

	
	//Topic publication
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
	obs_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/scan360/point_cloud", 1);
	people_pub_ = nh_.advertise<upo_msgs::PersonPoseArrayUPO>("/people/navigation", 1);
	goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/rrt_goal",1);
	//Visualization
	demo_path_pub_ = nh_.advertise<visualization_msgs::Marker>("demo_path_learning", 1);
	people_arrow_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/people/people_arrow_markers", 1);
	people_cyl_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/people/people_cylinders_markers", 1);

	//Declare ROS service clients
	astar_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/upo_navigation_node/make_plan");
	rrt_client_ = nh_.serviceClient<upo_rrt_planners::MakePlan>("/RRT_ros_wrapper/makeRRTPlan");
	//rrt_client_fcn_ = nh_.serviceClient<upo_rrt_planners::MakePlan>("/RRT_ros_wrapper3/makeRRTPlan");
	features_client_ = nh_.serviceClient<navigation_features::GetFeatureCount>("/navigation_features/getPathFeatureCount");
	setWeights_client_ = nh_.serviceClient<navigation_features::SetWeights>("/navigation_features/setWeights");
	loss_client_ = nh_.serviceClient<navigation_features::SetLossCost>("/navigation_features/set_use_loss_func");	
	init_w_client_ = nh_.serviceClient<navigation_features::InitWeights>("/navigation_features/initWeights");
}


upo_irl_ros::PlanningRos::PlanningRos(string data_dir, bool fcn) : Planning()
{
	fcn_ = fcn;

	data_dir_ = data_dir;
	data_ = new DataManagementRos(data_dir_);

 	path_adapt_ = true;

	tf_ = new tf::TransformListener();
	base_frame_id_ = "/base_link";
	odom_frame_id_ = "/odom";
	map_frame_id_ = "/map";


	tf_broadcaster_ = new tf::TransformBroadcaster();

	
	//Topic publication
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
	obs_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/scan360/point_cloud", 1);
	people_pub_ = nh_.advertise<upo_msgs::PersonPoseArrayUPO>("/people/navigation", 1);
	goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/rrt_goal",1);
	//Visualization
	demo_path_pub_ = nh_.advertise<visualization_msgs::Marker>("demo_path_learning", 1);
	people_arrow_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/people/people_arrow_markers", 1);
	people_cyl_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/people/people_cylinders_markers", 1);

	//Declare ROS service clients
	astar_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/upo_navigation_node/make_plan");
	//rrt_client_ = nh_.serviceClient<upo_rrt_planners::MakePlan>("/RRT_ros_wrapper/makeRRTPlan");
	rrt_client_fcn_ = nh_.serviceClient<upo_rrt_planners::MakePlan>("/RRT_ros_wrapper3/makeRRTPlan");
	features_client_ = nh_.serviceClient<navigation_features::GetFeatureCount>("/navigation_features/getPathFeatureCount");
	setWeights_client_ = nh_.serviceClient<navigation_features::SetWeights>("/navigation_features/setWeights");
	loss_client_ = nh_.serviceClient<navigation_features::SetLossCost>("/navigation_features/set_use_loss_func");	
	init_w_client_ = nh_.serviceClient<navigation_features::InitWeights>("/navigation_features/initWeights");
}


upo_irl_ros::PlanningRos::~PlanningRos()
{
	delete data_;
}


bool upo_irl_ros::PlanningRos::loadData() {
	return data_->loadData();
}


int upo_irl_ros::PlanningRos::getDataSize() {
	return data_->getDataSize();
}


void upo_irl_ros::PlanningRos::setScenario(int i)
{
	
	//Publish tf and odom
	publish_robot_position(i);

	ros::Time time = ros::Time::now();

	//Publish obstacle data
	sensor_msgs::PointCloud2 pc = (data_->getObsData(i))[0];
	pc.header.stamp = time;
	obs_pub_.publish(pc);

	//Publish people data
	upo_msgs::PersonPoseArrayUPO people = (data_->getPeopleData(i))[0];
	people.header.stamp = time;
	people_pub_.publish(people);
	//Visualize people
	visualize_people(people);

	//Publish goal data
	geometry_msgs::PoseStamped g = (data_->getGoalData(i))[0];
	geometry_msgs::PoseStamped goal = transformPoseTo(g, string("/base_link"), false);
	goal.header.stamp = time;
	goal_pub_.publish(goal);

	//Visualize demo path
	vector<geometry_msgs::PoseStamped> path = data_->getPathData(i);
	visualize_demo_path(path);
	
	
}

void upo_irl_ros::PlanningRos::visualize_people(upo_msgs::PersonPoseArrayUPO peop)
{
	visualization_msgs::MarkerArray arrowArray;	
	visualization_msgs::MarkerArray cylinderArray;
	ros::Time time = ros::Time::now();
	for(unsigned int i=0; i<peop.size; i++)
	{
		visualization_msgs::Marker markerDel;
		//Delete previous markers
		markerDel.action = 3; //visualization_msgs::Marker::DELETEALL;
		arrowArray.markers.push_back(markerDel);
		cylinderArray.markers.push_back(markerDel);
		people_arrow_pub_.publish(arrowArray);
		people_cyl_pub_.publish(cylinderArray);

		// Fill up arrow marker information
		visualization_msgs::Marker marker;
		marker.header.frame_id = peop.header.frame_id;
		marker.header.stamp = time; 
		marker.ns = "people_pose";
		marker.id = (i+1);
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position = peop.personPoses[i].position;
		marker.pose.orientation = peop.personPoses[i].orientation;
		marker.pose.position.z = 1.0; 
		marker.scale.x = 0.6; 
		marker.scale.y = 0.1; 
		marker.scale.z = 0.4; 
		marker.color.a = 0.8;
		marker.color.r = 0.0; 
		marker.color.g = 1.0; 
		marker.color.b = 0.0; 
		marker.lifetime = ros::Duration();
		arrowArray.markers.push_back(marker);

		// Fill up cylinder marker information
		marker.header.frame_id = peop.header.frame_id;
		marker.header.stamp = time; 
		marker.ns = "people_pose";
		marker.id = (i+1);
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position = peop.personPoses[i].position;
		marker.pose.orientation = peop.personPoses[i].orientation;
		marker.pose.position.z = 0.50;
		marker.scale.x = 0.5; 
		marker.scale.y = 0.5; 
		marker.scale.z = 1.5; 
		marker.color.a = 0.8;
		marker.color.r = 0.0; 
		marker.color.g = 1.0; 
		marker.color.b = 0.0; 
		marker.lifetime = ros::Duration();
		cylinderArray.markers.push_back(marker);

	}
	people_arrow_pub_.publish(arrowArray);
	people_cyl_pub_.publish(cylinderArray);
}




void upo_irl_ros::PlanningRos::visualize_demo_path(vector<geometry_msgs::PoseStamped> path)
{

	visualization_msgs::Marker points;
		  
	points.header.frame_id = path[0].header.frame_id; 
	points.header.stamp = ros::Time::now();
	points.ns = "basic_shapes";
	points.id = 0;
	points.type = visualization_msgs::Marker::SPHERE_LIST;
	points.action = visualization_msgs::Marker::ADD;
	points.pose.position.x = 0.0;
	points.pose.position.y = 0.0;
	points.pose.position.z = 0.1; 
	points.scale.x = 0.12;
	points.scale.y = 0.12;
	points.color.r = 1.0f;
	points.color.g = 0.0f;
	points.color.b = 0.0f;
	points.color.a = 0.8;
	points.lifetime = ros::Duration();
			
	for(unsigned int i=0; i<path.size(); i++)
	{
		geometry_msgs::Point p = path[i].pose.position;
		points.points.push_back(p);
	}
	demo_path_pub_.publish(points);

}





vector<float> upo_irl_ros::PlanningRos::makePlanAndCount(int i, int planner_type)
{
	vector<geometry_msgs::PoseStamped> path;
	vector<float> fc;

	geometry_msgs::PoseStamped g = (data_->getGoalData(i))[0];
	geometry_msgs::PoseStamped goal = transformPoseTo(g, string("/base_link"), false);

	//Plan a path
	if(planner_type == 1)  //RRT*
	{
		upo_rrt_planners::MakePlan mp;
		mp.request.goal = goal;

		if(fcn_) {
			if(!rrt_client_fcn_.call(mp))
			{
				ROS_ERROR("Failed to call service '/RRT_ros_wrapper3/makeRRTPlan'");
				return fc;
			}
		} else {
			if(!rrt_client_.call(mp))
			{
				ROS_ERROR("Failed to call service '/RRT_ros_wrapper/makeRRTPlan'");
				return fc;
			}
		}

		bool ok = mp.response.ok;
		path = mp.response.path;
		if(path_adapt_)
			path = pathAdaptation(path);

	}else if(planner_type == 2) //A*
	{

	}

	//Calcule the feature count of the path
	navigation_features::GetFeatureCount gfc;
	gfc.request.path = path;
	if(!features_client_.call(gfc))
	{
		ROS_ERROR("Failed to call service '/navigation_features/getPathFeatureCount'");
		return fc;
	}
	fc = gfc.response.fc;

	return fc;

}


vector<geometry_msgs::PoseStamped> upo_irl_ros::PlanningRos::makePlan(int i, int planner_type)
{
	vector<geometry_msgs::PoseStamped> path;

	geometry_msgs::PoseStamped g = (data_->getGoalData(i))[0];
	geometry_msgs::PoseStamped goal = transformPoseTo(g, string("/base_link"), false);

	//Plan a path
	if(planner_type == 1)  //RRT*
	{
		upo_rrt_planners::MakePlan mp;
		mp.request.goal = goal;

		if(fcn_) {
			if(!rrt_client_fcn_.call(mp))
			{
				ROS_ERROR("Failed to call service '/RRT_ros_wrapper3/makeRRTPlan'");
				return path;
			}
		} else {
			if(!rrt_client_.call(mp))
			{
				ROS_ERROR("Failed to call service '/RRT_ros_wrapper/makeRRTPlan'");
				return path;
			}
		}




		bool ok = mp.response.ok;
		path = mp.response.path;
		if(path_adapt_)
			path = pathAdaptation(path);

	}
	return path;
}




vector<float> upo_irl_ros::PlanningRos::getFeatureCountDemoPath(int i)
{
	vector<float> demo_fc;
	//Calcule the feature count of the path
	navigation_features::GetFeatureCount gfc;
	vector<geometry_msgs::PoseStamped> path = data_->getPathData(i);
	if(path_adapt_)
		path = pathAdaptation(path);
	gfc.request.path = path;

	if(!features_client_.call(gfc))
	{
		ROS_ERROR("Failed to call service '/navigation_features/getPathFeatureCount'");
		return demo_fc;
	}
	demo_fc = gfc.response.fc;
	return demo_fc;
}



vector<float> upo_irl_ros::PlanningRos::getFeatureCountPath(vector<geometry_msgs::PoseStamped>* mypath)
{
	vector<float> demo_fc;
	//Calcule the feature count of the path
	navigation_features::GetFeatureCount gfc;
	vector<geometry_msgs::PoseStamped> path = *mypath;
	if(path_adapt_)
		path = pathAdaptation(path);
	gfc.request.path = path;

	if(!features_client_.call(gfc))
	{
		ROS_ERROR("Failed to call service '/navigation_features/getPathFeatureCount'");
		return demo_fc;
	}
	demo_fc = gfc.response.fc;
	return demo_fc;
}





vector< vector<float> > upo_irl_ros::PlanningRos::calculateFeatureCountDemos(int type)
{
	vector< vector<float> > demo_fc;
	for(unsigned int i=0; i<data_->getDataSize(); i++)
	{
		printf("PlanningRos. calculeFeatureCountDemo %i of %i\n", (i+1), (int)data_->getDataSize());
		setScenario(i);

		//Calcule the feature count of the path
		navigation_features::GetFeatureCount gfc;
		vector<geometry_msgs::PoseStamped> path = data_->getPathData(i);
		if(path_adapt_)
			path = pathAdaptation(path);
		gfc.request.path = path;

		vector<float> fc;
		if(!features_client_.call(gfc))
		{
			ROS_ERROR("Failed to call service '/navigation_features/getPathFeatureCount'");
			return demo_fc;
		}
		fc = gfc.response.fc;
		demo_fc.push_back(fc);
	}

	return demo_fc;
}




void upo_irl_ros::PlanningRos::setWeights(vector<float> w)
{
	navigation_features::SetWeights srv;
	srv.request.weights = w;

  	if(!setWeights_client_.call(srv))
    	ROS_ERROR("Failed to call service '/navigation_features/setWeights'");
}




vector<float> upo_irl_ros::PlanningRos::initializeWeights(bool random, bool normalize)
{
		//Initialize the weights
		navigation_features::InitWeights iw;
		iw.request.random = random;
		iw.request.normalize = normalize;
		vector<float> fc;
		if(!init_w_client_.call(iw))
		{
			ROS_ERROR("Failed to call service '/navigation_features/initWeights'");
			return fc;
		}
		vector<float> weights = iw.response.weights;
		return weights;
}



void upo_irl_ros::PlanningRos::enableLossFunction(bool e, int i) 
{
	navigation_features::SetLossCost srv;
	srv.request.demo_path = data_->getPathData(i);
	srv.request.use_loss_func = e;
	if(!loss_client_.call(srv))
	{
		ROS_ERROR("Failed to call service '/navigation_features/set_use_loss_func'");
	}
}






void upo_irl_ros::PlanningRos::publish_robot_position(int i)
{
	ros::Time current = ros::Time::now();

	geometry_msgs::PoseStamped podom = data_->getRobotPose(i);
	//printf("PlanningRos. Publishing robot position in frame: %s\n", podom.header.frame_id.c_str());
	//geometry_msgs::PoseStamped podom = transformPoseTo(aux, string("/base_link"), false);

	//first, we'll publish the transforms over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current;
	odom_trans.header.frame_id = odom_frame_id_;
	odom_trans.child_frame_id = base_frame_id_;
	odom_trans.transform.translation.x = podom.pose.position.x;
	odom_trans.transform.translation.y = podom.pose.position.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(tf::getYaw(podom.pose.orientation));
	tf_broadcaster_->sendTransform(odom_trans);

	//Publish also the tf regarding the map frame
	geometry_msgs::PoseStamped pmap = data_->mapDataExists(i);	
	geometry_msgs::TransformStamped map_trans;
	map_trans.header.stamp = current;
	map_trans.header.frame_id = map_frame_id_;
	map_trans.child_frame_id = odom_frame_id_;
	map_trans.transform.translation.x = pmap.pose.position.x;
	map_trans.transform.translation.y = pmap.pose.position.y;
	map_trans.transform.translation.z = 0.0;
	map_trans.transform.rotation = tf::createQuaternionMsgFromYaw(tf::getYaw(pmap.pose.orientation));
	tf_broadcaster_->sendTransform(map_trans);
	

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current;
	odom.header.frame_id = odom_frame_id_;
		
	//set the position
	odom.pose.pose.position.x = podom.pose.position.x;
	odom.pose.pose.position.y = podom.pose.position.y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(podom.pose.orientation));
		
	//set the velocity
	odom.child_frame_id = base_frame_id_;
	odom.twist.twist.linear.x = 0.0;
	odom.twist.twist.linear.y = 0.0; 
	odom.twist.twist.angular.z = 0.0;
		
	//publish the odometry
	odom_pub_.publish(odom);
}






geometry_msgs::PoseStamped upo_irl_ros::PlanningRos::transformPoseTo(geometry_msgs::PoseStamped pose_in, string frame_out, bool usetime)
{
	geometry_msgs::PoseStamped in = pose_in;
	if(!usetime)
		in.header.stamp = ros::Time();
		
	geometry_msgs::PoseStamped pose_out;
	
	geometry_msgs::Quaternion q = in.pose.orientation;
	if(!isQuaternionValid(q))
	{
		ROS_WARN("PlanningRos. transformPoseTo. Quaternion no valid. Creating new quaternion with yaw=0.0");
		in.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	}
	try {
		tf_->transformPose(frame_out.c_str(), in, pose_out);
	}catch (tf::TransformException ex){
		ROS_WARN("PlanningRos. TransformException in method transformPoseTo. TargetFrame: %s : %s", frame_out.c_str(), ex.what());
	}
	//printf("Tranform pose. frame_in: %s, x:%.2f, y:%.2f, frame_out: %s, x:%.2f, y:%.2f\n", in.header.frame_id.c_str(), in.pose.position.x, in.pose.position.y, frame_out.c_str(), pose_out.pose.position.x, pose_out.pose.position.y);
	return pose_out;
}




bool upo_irl_ros::PlanningRos::isQuaternionValid( geometry_msgs::Quaternion q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
		ROS_ERROR("Quaternion has infs!!!!");
		return false;
    }
    if(std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
		ROS_ERROR("Quaternion has nans !!!");
		return false;
	}
	
	if(std::fabs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01) {
		ROS_ERROR("Quaternion malformed, magnitude: %.3f should be 1.0", (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w));
		return false;
	}

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding.");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
}


/**
 * Procedure to adapt the path, including or removing points
 * in order to have a point list with the same distance between points.
 * The distance between points is determined by the value of "point_distance".
 */
std::vector<geometry_msgs::PoseStamped> upo_irl_ros::PlanningRos::pathAdaptation(std::vector<geometry_msgs::PoseStamped> path)
{
	std::vector <geometry_msgs::PoseStamped> trajectory_aux;
	
	geometry_msgs::Point current = path[0].pose.position;
	trajectory_aux.push_back(path[0]);
	
	double range = 0.005;

	double point_distance = 0.1;
	
	for(unsigned int i=1; i<path.size(); i++)
	{
		geometry_msgs::Point next = path.at(i).pose.position;
		double dist = sqrt(((next.x-current.x)*(next.x-current.x)) + ((next.y-current.y)*(next.y-current.y)));

		if(dist > point_distance)	{	// Interpolation
			
			geometry_msgs::Point cp = current;
			int newnodes = (int)floor(dist/point_distance + 0.5);
			for(unsigned int j=0; j<newnodes; j++) {
					
				float theta = atan2((next.y-cp.y), (next.x-cp.x));
				float newx = cp.x + point_distance*cos(theta);
				float newy = cp.y + point_distance*sin(theta);
				geometry_msgs::PoseStamped newpose;
				newpose.header = path.at(0).header;
				newpose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				newpose.pose.position.x = newx;
				newpose.pose.position.y = newy;
				newpose.pose.position.z = 0.0;
				trajectory_aux.push_back(newpose);
				cp = newpose.pose.position;
			}

		} else if(dist < (point_distance-range)) // Discretization. 
			continue;
		/*if(dist > (point_distance+range)) {
			ROS_WARN("THERE IS NOT A POINT IN THE DISTANCE RANGE [%.2f, %.2f]", point_distance-range, point_distance+range);
		}*/
		
		trajectory_aux.push_back(path.at(i));
		
		current = next;
	}
	//printf("New trajectory of %u nodes.\n\n", (int)trajectory_aux.size());
	return trajectory_aux;
}








