
#include <upo_nav_irl/Learning.h>
#include <upo_nav_irl/Learner.h>
#include <upo_nav_irl/Algorithms/RTIRL.h>
#include <upo_nav_irl/Algorithms/RLT.h>
//#include <upo_nav_irl/Algorithms/MMP.h>

#include <upo_nav_irl/ros/PlanningRos.h>

using namespace std;

int main(int argc, char** argv)
{

	ros::init(argc, argv, "RunLearningRos");

	//Load the learning parameters	
	ros::NodeHandle nh("~");

	//Learning parameters
	string data_dir;
	nh.param("demo_dir", data_dir, string("/home/noe/"));
	string save_data_dir;
	nh.param("save_data_dir", save_data_dir, string("/home/noe/"));
	int epochs;
	nh.param("epochs", epochs, 40);
	int batch_size;
	nh.param("batch_size", batch_size, 10);
	bool norm_weights;
	nh.param("normalize_weights", norm_weights, true);
	bool random_init;
	nh.param("initial_random_weights", random_init, true);
	string algorithm;
	nh.param("algorithm", algorithm, string("RTIRL"));


	//Create the proper instance of the planning interface
	upo_irl::Planning* planning = new upo_irl_ros::PlanningRos(data_dir);

	//Create the learning algorithm
	upo_irl::Learner* learner;

	if(algorithm == "RTIRL") {

		//Load parameters for RTIRL algorithm
		float alpha;
		nh.param("rtirl_alpha", alpha, (float)0.5);
		float stab_incr;
		nh.param("rtirl_stab_incr", stab_incr, (float)0.0);
		int rrt_repetitions;
		nh.param("rtirl_rrt_repetitions", rrt_repetitions, 5);

		learner = new upo_irl::RTIRL(planning, norm_weights, alpha, stab_incr, rrt_repetitions);
	}
	else if(algorithm == "RLT") {

		//Load parameters for RLT algorithm
		float alpha;
		nh.param("rlt_alpha", alpha, (float)0.5);
		float stab_incr;
		nh.param("rlt_stab_incr", stab_incr, (float)0.0);
		int rrt_repetitions;
		nh.param("rlt_rrt_repetitions", rrt_repetitions, 5);
		learner = new upo_irl::RLT(planning, norm_weights, alpha, stab_incr, rrt_repetitions);

	/*else if(algorithm == "MMP") {
		learner = new upo_irl::MMP(planning);*/
	} else {
		printf("\nERROR. Wrong name of the learning algorithm. Exiting program...\n\n"); 
		return(0);
	}

	//Establish the learning process
	upo_irl::Learning irl(learner, epochs, batch_size, norm_weights, random_init, save_data_dir);


	sleep(5.0); //waiting a bit for upo_rrt_planners setup
	//run learning
	irl.runLearning(); 

	return(0);

}
