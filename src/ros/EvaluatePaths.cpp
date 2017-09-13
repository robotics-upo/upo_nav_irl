
//Read/write a file
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
#include <functional>   // std::plus, std::minus
#include <algorithm>    // std::transform

//open a directory
#include <boost/filesystem.hpp>

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

//ROS services
#include <nav_msgs/GetPlan.h> 					//A* planning
#include <upo_rrt_planners/MakePlan.h>  		//RRT* planning
#include <navigation_features/SetLossCost.h>	//Enable/Disable loss function for RLT and MMP algorithms
#include <navigation_features/InitWeights.h>
#include <navigation_features/SetWeights.h>
#include <navigation_features/GetFeatureCount.h>

//rosbag 
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <upo_nav_irl/ros/PlanningRos.h>


using namespace std;



// Parameters and variables
//--------------------------------------
tf::TransformListener* 	tf_;

//Store parameters
string					demo_dir_;
string					store_dir_;
string					store_frame_;
bool					store_demo_path_;
bool					store_people_;
bool					store_rrt_path_;
bool					store_obs_;
bool					store_fc_diff_;
int						rrt_repetitions_;
bool 					adapt_paths_;
bool					use_weights_;
vector<float>			weights_;
vector<float>			dissimilarity_;
vector<float>			metric2_;
vector< vector<float> >	fc_diff_;
float					diss_threshold_;
float					m2_threshold_low_;
float					m2_threshold_med_;
float					m2_threshold_high_;
bool					fcn_;

//Create the proper instance of the planning interface
upo_irl_ros::PlanningRos* planning_;





void initParameters()
{

	ros::NodeHandle n("~");
	n.param("adapt_paths", adapt_paths_, true);
	n.param("use_weights", use_weights_, false);
	n.param("store_demo_paths", store_demo_path_, false);
	n.param("store_rrt_paths", store_rrt_path_, false);
	n.param("store_people", store_people_, false);
	n.param("store_fc_diff", store_fc_diff_, true);
	n.param("fcn", fcn_, false);
	n.param("demo_bag_dir", demo_dir_, string("/home/"));
	n.param("store_dir", store_dir_, string("/home/"));
	n.param("store_frame", store_frame_, string("/odom"));
	n.param("rrt_repetitions", rrt_repetitions_, 4);
	n.param("dissimilarity_threshold", diss_threshold_, (float)0.02);
	n.param("metric2_threshold_low", m2_threshold_low_, (float)0.10);
	n.param("metric2_threshold_med", m2_threshold_med_, (float)0.25);
	n.param("metric2_threshold_high", m2_threshold_high_, (float)0.5);

	bool ok = true;
	unsigned int i = 1;
	//Load RRT* weights values
	if(use_weights_) {
		while(ok)
		{
			char buf[10];
			sprintf(buf, "rrt_w%u", i);
			std::string st = std::string(buf);
			
			if(n.hasParam(st.c_str())){
				double wg = 0.0;
				n.getParam(st.c_str(), wg);
				weights_.push_back((float)wg);	
				printf("Loading rrt_w %u: %.3f\n", i, wg);
			} else {
				//printf("param '%s' not found\n", st.c_str());
				ok = false;
			}
			i++;
		}
	}

	if(fcn_)
		planning_ = new upo_irl_ros::PlanningRos(demo_dir_, true);
	else
		planning_ = new upo_irl_ros::PlanningRos(demo_dir_);


	tf_ = new tf::TransformListener();


}






bool store_path(vector<geometry_msgs::PoseStamped>* path, int sc, int rep)
{
	//Create directory if necessary
	string directory;
	if(rep == -1)
		directory = store_dir_ + "demo_paths";
	else
		directory = store_dir_ + "rrt_paths";

	if(sc==0 && rep==-1) {
		mkdir(directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	} else if(sc == 0 && rep == 0) {
		mkdir(directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); 
	}

	char b[60];
	string filename;
	if(rep == -1) {
 		sprintf(b, "demo_sc%i.txt", (sc+1));
		std::string st = std::string(b);
		filename = directory + "/" + st;
	} else {
		sprintf(b, "rrt_sc%i_%i.txt", (sc+1), (rep+1));
		std::string st = std::string(b);
		filename = directory + "/" + st;
	}
	
	ofstream file;
	try {
		file.open(filename.c_str());
	} catch(...) {
		printf("\nERROR creating path file: %s\n\n", filename.c_str());
		return false;
	}
	for(unsigned int i=0; i<path->size(); i++)
	{
		geometry_msgs::PoseStamped p = path->at(i);
		file << p.pose.position.x << "\t" << p.pose.position.y << "\t" << tf::getYaw(p.pose.orientation) << std::endl;
	}
	file.close();
	return true;
}



bool store_people(vector<upo_msgs::PersonPoseArrayUPO>* p, int sc)
{
	return true;
}






vector<geometry_msgs::PoseStamped> transformPathTo(string frame, vector<geometry_msgs::PoseStamped>* path)
{
	vector<geometry_msgs::PoseStamped> p;
	for(unsigned int i=0; i<path->size(); i++) {
		geometry_msgs::PoseStamped pose = planning_->transformPoseTo(path->at(i), frame, false);
		p.push_back(pose);	
	}
	return p;
}




float dissimilarity(vector<geometry_msgs::PoseStamped>* p1, vector<geometry_msgs::PoseStamped>* p2)
{
	float xp1, yp1;
	float xp2, yp2;
	float accum = 0;
	for(unsigned int i=0; i<p1->size()-1; i++)
	{
		//Take point i
		xp1 = p1->at(i).pose.position.x;
		yp1 = p1->at(i).pose.position.y;
		//Take point i+1
		xp2 = p1->at(i+1).pose.position.x;
		yp2 = p1->at(i+1).pose.position.y;

		// ||p2 - p1||
		float dx = xp2 - xp1;
		float dy = yp2 - yp1;
		float norm = hypot(dx, dy);

		//look for the closest points in p2
		float dist1 = 9999;
		float dist2 = 9999;
		for(unsigned int j=0; j<p2->size(); j++)
		{
			//point i
			dx = p2->at(j).pose.position.x - xp1;
			dy = p2->at(j).pose.position.y - yp1;
			float d = hypot(dx, dy);
			if(d < dist1)
				dist1 = d;	

			//point i+1
			dx = p2->at(j).pose.position.x - xp2;
			dy = p2->at(j).pose.position.y - yp2;
			d = hypot(dx, dy);
			if(d < dist2)
				dist2 = d;				
		}
		
		accum += (((dist1 + dist2)/2.0) * norm);

	}
	float dis = accum/(float)p1->size();

	return dis;
}




//Store the dissimilarity values in a txt file and print statistics
bool storeMetrics()
{
	string filename = store_dir_ + "Metrics.txt";
	ofstream file;
	try {
		file.open(filename.c_str());
	} catch(...) {
		printf("\nERROR creating path file: %s\n\n", filename.c_str());
		return false;
	}


	// Dissimilarity
	float mean = (accumulate(dissimilarity_.begin(), dissimilarity_.end(), 0.0))/dissimilarity_.size();

	float match = 0.0;
	float stddev = 0.0;
	for(unsigned int i = 0; i < dissimilarity_.size(); i++) {
        stddev += pow(dissimilarity_[i] - mean, 2);
		if(dissimilarity_[i] < diss_threshold_)
			match = match + 1.0;
	}
	match = match/dissimilarity_.size();

    stddev = sqrt(stddev / dissimilarity_.size());

	float stderror = stddev/sqrt(dissimilarity_.size());

	

	// Distance metric 2
	float meanm2 = -1.0;
	float stddevm2 = -1.0;
	float stderrorm2 = 1.0;
	float m2_low = 0.0;
	float m2_med = 0.0;
	float m2_high = 0.0;
	if(!metric2_.empty())
	{
		meanm2 = (accumulate(metric2_.begin(), metric2_.end(), 0.0))/metric2_.size();
		stddevm2 = 0.0;
		for(unsigned int i = 0; i < metric2_.size(); i++) {
       		stddevm2 += pow(metric2_[i] - meanm2, 2);
			if(metric2_[i] < m2_threshold_low_)
				m2_low=m2_low+1.0;
			if(metric2_[i] < m2_threshold_med_)
				m2_med=m2_med+1.0;
			if(metric2_[i] < m2_threshold_high_)
				m2_high=m2_high+1.0;
		}
		m2_low = m2_low/metric2_.size();
		m2_med = m2_med/metric2_.size();
		m2_high = m2_high/metric2_.size();

    	stddevm2 = sqrt(stddevm2 / metric2_.size());

		stderrorm2 = stddevm2/sqrt(metric2_.size());

	}

	//Feature count difference
	vector<float> fc_means;
	float mean_fc = 0.0;
	float stddev_fc = 0.0;
	float stderror_fc = 0.0;
	if(store_fc_diff_) {
		for(unsigned int i=0; i<fc_diff_.size(); i++) {
			vector<float> fc_traj = fc_diff_[i];
			float fc_mean = 0.0;
			for(unsigned int j=0; j<fc_traj.size(); j++)
				fc_mean += fabs(fc_traj[j]);

			fc_mean = fc_mean/fc_traj.size();
			fc_means.push_back(fc_mean);

		}

		mean_fc = (accumulate(fc_means.begin(), fc_means.end(), 0.0))/fc_means.size();
		for(unsigned int d = 0; d < fc_means.size(); d++) 
       		stddev_fc += pow(fc_means[d] - mean_fc, 2);

		stddev_fc = sqrt(stddev_fc / fc_means.size());

		stderror_fc = stddev_fc/sqrt(fc_means.size());
	}




	// Show results
	printf("\n---METRICS RESULTS---\n\n");
	for(unsigned int j=0; j<dissimilarity_.size(); j++) {
		printf("Scenario %u:\t%.4f\t%.4f\n", (j+1), dissimilarity_[j], (metric2_.empty()?-1.0:metric2_[j]));
		file << (j+1) << "\t" << dissimilarity_[j] << "\t" << (metric2_.empty()?-1.0:metric2_[j]);
		if(store_fc_diff_) {
			for(unsigned k=0; k<fc_diff_[0].size(); k++)
				file << "\t" << fc_diff_[j][k]; 

			file << "\t" << fc_means[j];
		}
		file << std::endl;
	}

	printf("\n---METRICS SUMMARY---\n\n");
	printf("          dissim\tmu    \tfc_diff\n");
	printf("mean:     %.4f\t%.4f\t%.4f\n", mean, meanm2, mean_fc);
	printf("stddev:   %.4f\t%.4f\t%.4f\n", stddev, stddevm2, stddev_fc);
	printf("stderror: %.4f\t%.4f\t%.4f\n\n", stderror, stderrorm2, stderror_fc);
	printf("Diss match percentage (thres:%.3f): %.3f\n", diss_threshold_, match);
	printf("Met2 match low (thres:%.3f):  %.3f\n", m2_threshold_low_, m2_low);
	printf("Met2 match med (thres:%.3f):  %.3f\n", m2_threshold_med_, m2_med);
	printf("Met2 match high (thres:%.3f): %.3f\n\n", m2_threshold_high_, m2_high);

	/*file << std::endl << std::endl;
	file << "mean:\t" << mean << "\t" << meanm2 << std::endl;
	file << "stddev:\t" << stddev << "\t" << stddevm2 << std::endl;
	file << "stderror:\t" << stderror << "\t" << stderrorm2 << std::endl;
	file << "Diss match (thres:" << diss_threshold_ << "):\t" << match << std::endl;
	file << "M2 match low (thres:" << m2_threshold_low_ << "):\t" << m2_low << std::endl;
	file << "M2 match med (thres:" << m2_threshold_med_ << "):\t" << m2_med << std::endl;
	file << "M2 match high (thres:" << m2_threshold_high_ << "):\t" << m2_high << std::endl;
	file << std::endl;*/

	file.close();
	return true;
}




float distancemin(vector<geometry_msgs::PoseStamped>* p1, vector<geometry_msgs::PoseStamped>* p2)
{
	float xp1, yp1;
	float xp2, yp2;
	float accum = 0;
	for(unsigned int i=0; i<p1->size(); i++)
	{
		//Take point i
		xp1 = p1->at(i).pose.position.x;
		yp1 = p1->at(i).pose.position.y;
		
		//look for the closest points in p2
		float dist = 9999;
		for(unsigned int j=0; j<p2->size(); j++)
		{
			//point i
			float dx = p2->at(j).pose.position.x - xp1;
			float dy = p2->at(j).pose.position.y - yp1;
			float d = hypot(dx, dy);
			if(d < dist)
				dist = d;				
		}
		
		accum += dist;

	}
	float dis = accum/(float)p1->size();

	return dis;
}


float metric2(vector<geometry_msgs::PoseStamped>* p1, vector<geometry_msgs::PoseStamped>* p2)
{
	float avg1 = distancemin(p1, p2);
	float avg2 = distancemin(p2, p1);

	return ((avg1+avg2)/2.0);
}






void runEvaluation()
{
	
	if(use_weights_) {
		//Set the weight for the RRT* cost function
		planning_->setWeights(weights_);
	}

	for(unsigned int i=0; i<planning_->getDataSize(); i++)
	{
		printf("\nEvaluation scenario %u\n\n", (i+1));
		planning_->setScenario(i);

		//Get demo path
		vector<geometry_msgs::PoseStamped> dpath = planning_->getPathData(i);
		dpath = transformPathTo(store_frame_, &dpath);
		
		if(adapt_paths_)
			dpath = planning_->pathAdaptation(dpath);
		
		if(store_demo_path_)
			store_path(&dpath, i, -1);
		

		if(store_people_) {
			vector<upo_msgs::PersonPoseArrayUPO> p = planning_->getPeopleData(i);
			store_people(&p, i);
		}

		vector<float> demo_fc;
		if(store_fc_diff_) {
			demo_fc = planning_->getFeatureCountDemoPath(i);
		}


		
		vector<geometry_msgs::PoseStamped> rpath;
		for(unsigned int j=0; j<rrt_repetitions_; j++)
		{
			planning_->setScenario(i);

			rpath = planning_->makePlan(i, 1);

			if(store_fc_diff_) {
				vector<float> diff_fc;
				//diff_fc.assign((int)demo_fc.size(), 0.0);
				vector<float> path_fc = planning_->getFeatureCountPath(&rpath);
				//transform(demo_fc.begin(), demo_fc.end(), path_fc.begin(), diff_fc.begin(), std::minus<float>()); //diff_fc = demo_fc - path_fc;
				for(unsigned int k=0; k<demo_fc.size(); k++)
					diff_fc.push_back(fabs(demo_fc[k] - path_fc[k]));

				fc_diff_.push_back(diff_fc);
			}

			rpath = transformPathTo(store_frame_, &rpath);

			if(store_rrt_path_)
				store_path(&rpath, i, j);

			//float dis += dissimilarity(dpath, rpath);
			float dis = dissimilarity(&dpath, &rpath);
			dissimilarity_.push_back(dis);

			float m2 = metric2(&dpath, &rpath);
			metric2_.push_back(m2);
		}
		//dis = (float)dis/rrt_repetitions_;
		

		//dissimilarity_.push_back(dis);

	}

	storeMetrics();
}





int main(int argc, char** argv){

	ros::init(argc, argv, "Evaluate_paths");

	//Init parameters and variables
	initParameters();

	sleep(20.0); //waiting a bit for upo_rrt_planners setup

	//Load demo data
	planning_->loadData();

	//Run the store process
	runEvaluation();

}














