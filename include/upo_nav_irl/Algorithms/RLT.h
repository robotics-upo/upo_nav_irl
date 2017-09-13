#ifndef UPO_IRL_RLT_
#define UPO_IRL_RLT_


#include <vector>
#include <queue>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>


#include <upo_nav_irl/Learner.h>


using namespace std;


namespace upo_irl
{
	class RLT : public upo_irl::Learner
	{

		public:
			RLT(upo_irl::Planning* p, bool norm, float alpha, float inc, int rep);
			~RLT();

			//void setup();

			//bool loadData(string dir);

			//void setScenario(int i);

			//vector<path_point> makePlan(path_point goal, int planner_type);

			vector<float> run_trajectory_diff(int i); 

			/*virtual setGoal(path_point g) {
				goal_  = g;
			};*/

			//void setWeights(vector<float> w);

			//void calculateFeatureCountDemos();

			vector<float> updateWeights(vector<float> current_weights, vector<float> fc_diff);

			void updateParamsIter();

			//vector<float> getFeatureCount(vector<path_point> path);

			//vector<float> getFeatureCount(int i);

			//vector<float> getGradients(float base);



		private:

			float			alpha_;

			float			stabilizer_;
			float			stab_incr_;

			int				rrt_repetitions_;		

	};
	
}
#endif
