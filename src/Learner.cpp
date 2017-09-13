
#include <upo_nav_irl/Learner.h>


/*upo_irl::Learner::Learner(upo_irl::Planning* p)
{
	planning_ = p;
}*/

upo_irl::Learner::Learner()
{
}


upo_irl::Learner::~Learner() {

}



vector<float> upo_irl::Learner::initializeWeights(bool random, bool normalize)
{
	vector<float> w = planning_->initializeWeights(random, normalize);
	features_number_ = (int)w.size();
	return w;

}



bool upo_irl::Learner::loadData()
{
	return planning_->loadData();
}



vector<float> upo_irl::Learner::makePlanAndCount(int i, int planner_type) {
	return planning_->makePlanAndCount(i, planner_type);
}





