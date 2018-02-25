#ifndef MAORIS_EVALUATIONPARAM_31072017
#define MAORIS_EVALUATIONPARAM_31072017

#include "Evaluation.hpp"

namespace AASS{
	
	namespace maoris{
		
		class EvaluationParam{
		protected:
			
			std::vector<double> _mean_p;
			std::vector<double> _mean_r;
			std::vector<double> _f1_score;
			std::vector<double> _mean_ir;
			std::vector<double> _sd_p;
			std::vector<double> _sd_r;
			std::vector<double> _sd_ir;
			std::vector<double> _t_value;
//			std::vector<double> _m_value;
			std::vector<double> _matthewCC;
			std::vector<double> _matthewCC_median;
			std::vector<double> _matthewCC_perzone;
			std::vector<double> _matthewCC_median_perzone;
			std::vector<double> _sd_matthewCC;
			std::vector<double> _gscore;
			std::vector<double> _dor;
			std::vector<double> _accuracy;
            std::vector<double> _max;
            std::vector<double> _min;
            std::vector<double> _max_perzone;
            std::vector<double> _min_perzone;

            std::vector<Evaluation> _evaluation;

			
		public:
			EvaluationParam(){}
			
//			void add(double p, double r, double ir, double sdp, double sdr, double sdir, double mCC, double smCC, double gscore, double f1score, double acc, double t, double max, double min){
//				_mean_p.push_back(p);
//			    _mean_r.push_back(r);
//			    _mean_ir.push_back(ir);
//			    _sd_p.push_back(sdp);
//				_sd_r.push_back(sdr);
//				_sd_ir.push_back(sdir);
//				_t_value.push_back(t);
////				_m_value.push_back(m);
//
//				_matthewCC.push_back(mCC);
//				_sd_matthewCC.push_back(smCC);
//				_gscore.push_back(gscore);
//				_f1_score.push_back(f1score);
//				_accuracy.push_back(acc);
//
//                _max.push_back(max);
//                _min.push_back(min);
//
//                _evaluation.push_back(eval);
//			}
			
			void add(Evaluation eval, double t){
				
				eval.calculate();
				
				_mean_p.push_back(eval.getMeanPrecision());
			    _mean_r.push_back(eval.getMeanRecall());
			    _mean_ir.push_back(eval.getMeanInverseRecall());
			    _sd_p.push_back(eval.getSDPrecision());
				_sd_r.push_back(eval.getSDRecall());
				_sd_ir.push_back(eval.getSDInverseRecall());
				_t_value.push_back(t);
//				_m_value.push_back(m);
				
				_matthewCC.push_back(eval.getMatthewsCC());
				_sd_matthewCC.push_back(eval.getSDMatthewCC());
				_matthewCC_median.push_back(eval.getMatthewsCCMedian());
				_gscore.push_back(eval.getGscore());
				_f1_score.push_back(eval.getFscore());
				_dor.push_back(eval.getDOR());
				_accuracy.push_back(eval.getAccuracy());

                _max.push_back(eval.getMax());
                _min.push_back(eval.getMin());
                _max_perzone.push_back(eval.getMaxPerZone());
                _min_perzone.push_back(eval.getMinPerZone());
                
                _matthewCC_perzone.push_back(eval.getMatthewsCCPerZone());
			    _matthewCC_median_perzone.push_back(eval.getMatthewsCCMedianPerZone());

                _evaluation.push_back(eval);

			}
			
			size_t size(){return _mean_p.size();}
			
			void exportAll(const std::string &file_out, double start, double step){
								
				std::string result_file = file_out;
				std::ofstream myfile;
				if(!exists_test3(result_file)){
					myfile.open (result_file);

                    //Basic info
                    myfile << "# start and step: " << start << " " << step << "\n";
					myfile << "# mean_p mean_r mean_ir sd_p sd_r sd_ir f1_score g_score dor matthewsCC sd_mCC matthewsCCmedian accuracy variable max min mcc_perzone mcc_median_perzone\n";
				}
				else{
					myfile.open (result_file, std::ios::out | std::ios::app);
				}
				
				if (myfile.is_open())
				{




					for(int i = 0 ; i < _mean_p.size() ; ++i){
						
						myfile << _mean_p[i] << " " << _mean_r[i] << " " << _mean_ir[i] << " " << _sd_p[i] << " " << _sd_r[i] << " " << _sd_ir[i] << " " << _f1_score[i] << " " << _gscore[i] << " " << _dor[i] << " " << _matthewCC[i] << " " << _sd_matthewCC[i] << " " << _matthewCC_median[i] << " " << _accuracy[i] << " " << _t_value[i] << " " << _max[i] << " " << _min[i] << " " << _matthewCC_perzone[i] << " " << _matthewCC_median_perzone[i] << " " << _max_perzone[i] << " " << _min_perzone[i] << "\n";
						
					}


                    //Box plots
					myfile << "\n\n";

                    double count = start;
                    for(auto eval : _evaluation) {
                        for (auto mcc : eval.getAllMatthewsCC()) {
                            myfile << count << " " << mcc << "\n";
						}
                        count = count + step;
                    }

					
					myfile.close();
					
					
				}
				else std::cout << "Unable to open file";
				
			}
				
			
		};
		
	}
}


#endif
