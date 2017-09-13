
#include <upo_nav_irl/Algorithms/RTIRL.h>



upo_irl::RTIRL::RTIRL(upo_irl::Planning* p, bool norm, float alpha, float inc, int rep) : Learner()
{
	planning_ = p;
	normalize_weights_ = norm;
	alpha_ = alpha;
	stabilizer_ = 1.0;
	stab_incr_ = inc;
	rrt_repetitions_ = rep;
}


upo_irl::RTIRL::~RTIRL() {
	delete planning_;
}





vector<float> upo_irl::RTIRL::run_trajectory_diff(int i)
{
	//get the feature count of the demo
	//vector<float> fc_demo = fc_demos_[i]; 

	//Set the scenario and the goal
	planning_->setScenario(i);

	//get the feature count of the demo path
	vector<float> fc_demo = planning_->getFeatureCountDemoPath(i);

	//Get the goal
	//goal = planning_->getGoal(i);

	vector<float> fc;
	fc.assign(features_number_, 0.0);
	for(unsigned int r=0; r < rrt_repetitions_; r++)
	{
		//Set the scenario and the goal
		//printf("RTIRL.setting scenario %i\n", (i+1));
		planning_->setScenario(i);

		//printf("RTIRL. Planning and getting fc\n");
		vector<float> aux = makePlanAndCount(i, 1);  //type=1->RRT*, type=2->A*
		
		std::transform(fc.begin(), fc.end(), aux.begin(), fc.begin(), std::plus<float>());	//fc = fc + aux;	
		
	}
	for(unsigned int i=0; i<fc.size(); i++)
		fc[i] = fc[i]/rrt_repetitions_;

	vector<float> fc_diff;  //[(int)fc.size()];
	fc_diff.assign((int)fc.size(), 0.0);
	
	transform(fc_demo.begin(), fc_demo.end(), fc.begin(), fc_diff.begin(), std::minus<float>()); //fc_diff = fc_demo - fc;
	
	return fc_diff;
}






vector<float> upo_irl::RTIRL::updateWeights(vector<float> current_weights, vector<float> fc_diff)
{
	//Obtain the gradients (in this case the gradient is equal to the fc_diff)
	vector<float> gradients = fc_diff;

	vector<float> weights;
	weights.assign(gradients.size(), 0.0);
			
	//Update the weights
	float total = 0.0;
	for(unsigned int i=0; i<gradients.size(); i++)
	{
		/*if(exp_gradient) {
			double exponent = exp((alpha/stabilizer)*gradient[w]);
			weights[w] = weights[w]*exponent;
		} else {*/
			weights[i] = current_weights[i] - ((alpha_/stabilizer_)*gradients[i]); 
			if(weights[i] < 0.0) {
				weights[i] = 0.0;
			}
		//}
			
		total += weights[i];
	}

	//Normalize weights
	if(normalize_weights_)
	{
		for(unsigned int i=0; i<weights.size(); i++)
			weights[i] = weights[i]/total;
	}

	return weights;
}


void upo_irl::RTIRL::updateParamsIter() {
	stabilizer_ += stab_incr_;
}




/*void upo_irl::RTIRL::calculateFeatureCountDemos()
{
	fc_demos_ = planning_->calculateFeatureCountDemos(1);
	features_number_ = (int)fc_demos_[0].size();
}*/




