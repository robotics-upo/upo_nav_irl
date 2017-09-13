
#include <upo_nav_irl/Learning.h>


upo_irl::Learning::Learning() {

}

upo_irl::Learning::Learning(upo_irl::Learner* l, int ep, int bs, bool nw, bool ri, string save_dir) {
	learner_ = l;
	epochs_ = ep;
	batch_size_ = bs;
	normalize_weights_ = nw;
	random_initialization_ = ri;
	save_dir_ = save_dir;
}


upo_irl::Learning::~Learning() {

}


/*void upo_irl::Learning::setup(int ep, int bs)
{
	
}*/


void upo_irl::Learning::runLearning()
{
	printf("\n----RUNNING LEARNING-----\n\n");

	
	//Load data
	printf("\tLOADING DEMONSTRATION DATA...\n"); 
	learner_->loadData();
	int data_size = learner_->getDataSize();
	int nbatchs = floor(data_size/batch_size_);
	int res = data_size % batch_size_;
	if(res != 0)
		printf("WARNING. The data_size is not multiple of the batch_size. The last %i samples will be discarded!\n", res);
	printf("Data loaded!\n\ttotal trajectories: %i\n", data_size);
	printf("\tbatch size: %i\n\ttotal batches: %i\n", batch_size_, nbatchs);


	//Calculate feature counts of the loaded demo trajectories
	//learner_->calculateFeatureCountDemos();

	//file to write the output of the learning process
	string output_w = save_dir_ + "/output_weights.txt";
	string output_fc = save_dir_ + "/output_fc_diff.txt";
	printf("Storing learning weights in: %s\n", output_w.c_str());
	printf("Storing learning fc diff in: %s\n", output_fc.c_str());
	ofstream outputw;
	ofstream outputfc;
	try {
		outputw.open(output_w.c_str());
		outputfc.open(output_fc.c_str());
	} catch(...) {
		printf("ERROR creating output file: %s\n\n", output_w.c_str());
		return;
	}


	//Initialize weights
	weights_ = learner_->initializeWeights(random_initialization_, normalize_weights_);
	printf("Initial weights:\n");
	for(unsigned int i=0; i<weights_.size(); i++) {
		printf("w%u: %.5f\n", (i+1), weights_[i]);
		outputw << weights_[i] << "\t";
	}
	outputw << std::endl;
	printf("\n");
	learner_->setWeights(weights_);


	//Begin Learning loop
	printf("\tBEGINNING LEARNING LOOP...\n");
	for(unsigned int i=0; i<epochs_; i++)
	{
		
		int ind_traj=0;
		//Iterate over the examples
		for(unsigned int j=0; j<nbatchs; j++)
		{

			vector<float> fc_diff_batch;
			fc_diff_batch.assign(weights_.size(), 0.0);
			for(unsigned int k=0; k<batch_size_; k++)
			{
				//Plan a trajectory and get the feature count difference regarding the demo path
				printf("Executing trajectory %i\n", (ind_traj+1));
				vector<float> fc_diff = learner_->run_trajectory_diff(ind_traj); //fc_diff = (fc - fc_demo)

				transform(fc_diff_batch.begin(), fc_diff_batch.end(), fc_diff.begin(), fc_diff_batch.begin(), std::plus<float>());	
	 	
				ind_traj++;
			}

			//End of the batch, average the values
			for(unsigned int f=0; f<fc_diff_batch.size(); f++) {
				fc_diff_batch[f] = fc_diff_batch[f]/batch_size_;
				outputfc << fc_diff_batch[f] << "\t";
			}
			outputfc << std::endl;

			//Calcule gradients and update weights
			weights_ = learner_->updateWeights(weights_, fc_diff_batch);
			printf("Epoch %i. Batch %i finished! new Weights:\n", (i+1), (j+1));
			for(unsigned int w=0; w<weights_.size(); w++) {
				printf("w%u: %.5f\n", (w+1), weights_[w]);
				outputw << weights_[w] << "\t";
			}
			outputw << std::endl;
			printf("\n");
			learner_->setWeights(weights_);

		}
		learner_->updateParamsIter();
		printf("\nEpoch %u finished!\n\n", (i+1));
	}

	printf("\tLEARNING FINISHED!!!\n");
	outputw.close();
	outputfc.close();
}




