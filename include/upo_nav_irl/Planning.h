#ifndef UPO_IRL_PLANNING_
#define UPO_IRL_PLANNING_


#include <vector>
#include <queue>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>

//#include <upo_nav_irl/utils/DataStructure.h>


using namespace std;

namespace upo_irl
{
	class Planning
	{

		public:

			/*struct path_point
			{
				float x;
				float y;
				float yaw;
				string frame;
			};*/
		
			
			Planning(){};
			
			virtual ~Planning(){}; 

			virtual bool loadData() = 0;

			virtual int getDataSize() = 0;

			virtual void setScenario(int i) = 0;

			virtual void setWeights(vector<float> w) = 0;

			//virtual path_point getGoal(int i);

			virtual vector<float> makePlanAndCount(int i, int planner_type) = 0;

			virtual vector<float> getFeatureCountDemoPath(int i) = 0;

			virtual vector< vector<float> > calculateFeatureCountDemos(int type) = 0;

			virtual vector<float> initializeWeights(bool random, bool normalize) = 0;

			virtual void enableLossFunction(bool e, int i) = 0;


		protected:

			string				data_dir_;
	};
	
}
#endif



