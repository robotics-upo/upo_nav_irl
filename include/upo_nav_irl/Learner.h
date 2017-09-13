#ifndef UPO_IRL_LEARNER_
#define UPO_IRL_LEARNER_


#include <vector>
#include <queue>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include <functional>   // std::plus, std::minus
#include <algorithm>    // std::transform


#include <upo_nav_irl/Planning.h>




using namespace std;

namespace upo_irl
{
	class Learner
	{

		public:
		
			//Learner(upo_irl::Planning* p);
			Learner();
			
			virtual ~Learner(); 

			bool loadData();

			//virtual void setScenario(int i);

			virtual vector<float> run_trajectory_diff(int i) = 0;

			vector<float> makePlanAndCount(int i, int planner_type);

			/*void setGoal(path_point g) {
				goal_  = g;
			};*/

			void setWeights(vector<float> w) {
				planning_->setWeights(w);
			};

			//virtual vector<float> getFeatureCount(vector<path_point> path);

			/*vector< vector<float> > getFeatureCountDemos() {
				return fc_demos_;
			};*/

			int getDataSize() {
				planning_->getDataSize();
			}

			//virtual void calculateFeatureCountDemos() = 0;

			//virtual vector<float> getGradients(vector<float> fc_diff);  

			virtual	vector<float> updateWeights(vector<float> current_weights, vector<float> fc_diff) = 0;

			virtual void updateParamsIter() = 0;

			vector<float> initializeWeights(bool random, bool normalize);

			//virtual void enableLossFunction(bool e);

			//virtual void showPath();

			//virtual void showDemoPath();
			
			

		protected:

			upo_irl::Planning*			planning_;

			//int							planner_type_;

			//vector< vector<float> >		fc_demos_;

			int							features_number_;

			bool						normalize_weights_;
			

	};
	
}
#endif



