
#include <upo_nav_irl/ros/DataManagementRos.h>



upo_irl_ros::DataManagementRos::DataManagementRos(string dir) {
	data_dir_ = dir;
}

upo_irl_ros::DataManagementRos::~DataManagementRos() {
}


bool upo_irl_ros::DataManagementRos::loadData()
{
	printf("\nReading demonstration bags...\n\n");		

	// we check if the directory is valid
	boost::filesystem::path my_path(data_dir_.c_str());
	if(!boost::filesystem::exists(my_path) || !boost::filesystem::is_directory(my_path))
	{
		ROS_ERROR("ERROR. Directory '%s' of demonstration samples does not exists or is not a directory", data_dir_.c_str());
		return false;
	}else if(boost::filesystem::is_empty(my_path)) {
		ROS_ERROR("ERROR. Directory of demonstration samples is empty.");
		return false;	
	}

	//The directory should contain bag files with trajectories.
	int ntrajs = 0;
	boost::filesystem::directory_iterator it_end;
	for(boost::filesystem::directory_iterator it_sc(my_path); it_sc != it_end; it_sc++ )
		ntrajs++;

	printf("Trajectories detected: %u\n", ntrajs);	
	ntrajs = 1;

	try{
		// Get the files in the main directory
		boost::filesystem::directory_iterator end_it;
		for(boost::filesystem::directory_iterator it_files(my_path); it_files != end_it; it_files++ ) 
		{
			trajectory_t  trajectory;
			if( boost::filesystem::is_regular_file(it_files->status())) 
			{
				printf("Loading trajectory %u. File name: %s\n", ntrajs, it_files->path().filename().c_str());
				std::string direc_bag = string(it_files->path().c_str()); //ros::package::getPath("navigation_experiments")
				rosbag::Bag bag;
				try {
					bag.open(direc_bag, rosbag::bagmode::Read);
							
				} catch (rosbag::BagException& ex) {
					ROS_ERROR("Error opening bag file %s : %s", it_files->path().filename().c_str(), ex.what());
					return false;
				}

				std::vector<std::string> topics;
				topics.push_back(std::string("robot"));
				topics.push_back(std::string("people"));
				topics.push_back(std::string("goal"));
				topics.push_back(std::string("obstacles"));
				topics.push_back(std::string("vels"));
				topics.push_back(std::string("robot_map"));
				rosbag::View view(bag, rosbag::TopicQuery(topics));
						
						
				foreach(rosbag::MessageInstance const m, view)
				{
					if(m.getTopic()=="robot"){
						geometry_msgs::PoseStamped::Ptr robo= m.instantiate<geometry_msgs::PoseStamped>();
						trajectory.robot_data.push_back(*robo.get());
					}
					else if(m.getTopic()=="people"){
						upo_msgs::PersonPoseArrayUPO::Ptr ppl = m.instantiate<upo_msgs::PersonPoseArrayUPO>();
						trajectory.people_data.push_back(*ppl.get());
					}
					else if (m.getTopic()=="goal"){
						geometry_msgs::PoseStamped::Ptr gol = m.instantiate<geometry_msgs::PoseStamped>();
						trajectory.goal_data.push_back(*gol.get());
					}
					else if (m.getTopic()=="obstacles"){
						sensor_msgs::PointCloud2::Ptr obs = m.instantiate<sensor_msgs::PointCloud2>();
						trajectory.obs_data.push_back(*obs.get());
					}
					else if (m.getTopic()=="vels"){
						geometry_msgs::Twist::Ptr vel = m.instantiate<geometry_msgs::Twist>();
						trajectory.vel_data.push_back(*vel.get());
					}
					else if (m.getTopic()=="robot_map"){
						geometry_msgs::PoseStamped::Ptr rm = m.instantiate<geometry_msgs::PoseStamped>();
						trajectory.robot_map_data.push_back(*rm.get());
					}
				}

				bag.close();
						
				ntrajs++;
				dataset_.push_back(trajectory);

			} else
			{
				ROS_ERROR("File %s, is not a regular file", it_files->path().filename().c_str());
				return false;
			}
			
		} //end of the for loop demonstration files 
				
				

	} catch(const boost::filesystem::filesystem_error& ex){
		std::cout << "ERROR opening demonstrations files: " << ex.what() << '\n';
		return false;
	}

	data_size_ = (int)dataset_.size();
	return true;
}



geometry_msgs::PoseStamped upo_irl_ros::DataManagementRos::mapDataExists(int i) {
	
	geometry_msgs::PoseStamped p;
	p.header.frame_id = "map";
	p.header.stamp = ros::Time::now();
	p.pose.position.z = 0.0;
	
	if(dataset_[i].robot_map_data.empty()) {
		p.pose.position.x = 0.0;
		p.pose.position.y = 0.0;
		p.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);	
	}else {
		p.pose.position = dataset_[i].robot_map_data[0].pose.position;
		p.pose.orientation = dataset_[i].robot_map_data[0].pose.orientation;
	}

	return p;
}


geometry_msgs::PoseStamped upo_irl_ros::DataManagementRos::getRobotPose(int i)
{
	return dataset_[i].robot_data[0];
}

