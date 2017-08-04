#ifndef MAORIS_EVALUATION_31072017
#define MAORIS_EVALUATION_31072017

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <sys/stat.h>

#include <boost/filesystem.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/filesystem/path.hpp>

#include "Measures.hpp"
#include "Utils.hpp"

namespace AASS{
	
	namespace maoris{
		
		typedef std::map <std::vector<int>, std::vector <cv::Point> > match2points;
		typedef std::map <int, std::vector <cv::Point> > tag2points;
		typedef std::map <int, tag2points> tag2tagMapper;
		
		struct results{
			double time;
			double precision;
			double recall;
			double inverse_recall;
			double tp;
			double tn;
			double fp;
			double fn;
		};
		
		inline bool exists_test3 (const std::string& name) {
			struct stat buffer;   
			return (stat (name.c_str(), &buffer) == 0); 
		}

		inline void exportResultsGnuplot(const std::string& file_out, const std::string& file_studied, const AASS::maoris::results& Regions, double max, float proper_size){
			boost::filesystem::path p(file_studied);
			std::string name = p.filename().stem().string();
			
			std::string result_file = file_out;
			std::ofstream myfile;
			if(!exists_test3(result_file)){
				myfile.open (result_file);
				myfile << "precision recall inverse_recall time labels size\n";
			}
			else{
				myfile.open (result_file, std::ios::out | std::ios::app);
			}
			
			if (myfile.is_open())
			{
				myfile << name << " " << Regions.precision << " " << Regions.recall << " " << Regions.inverse_recall << " " << Regions.time << " " << max << " " << proper_size << "\n";
				myfile.close();
			}
			else std::cout << "Unable to open file";
		}
		
		
		bool checkAllValue(const cv::Mat& mat_in, const std::vector<int>& all_vals);
		cv::Mat segment_Ground_Truth(cv::Mat GroundTruth_BW);
		void compare_images(cv::Mat GT_segmentation_in, cv::Mat DuDe_segmentation_in, results& res_out);
		
		void extract_results(results& Regions, std::vector<float>& Precisions, std::vector<float>& Recalls, std::vector<float>& inverse_recall);
		
		
		
		class Evaluation{
		protected:
			std::vector<double> _precision;
			std::vector<double> _recall;
			std::vector<double> _inverse_recall;
			std::vector<double> _time;
			std::vector<std::string> _name;
			std::vector<float> _proper_size;
			std::vector<double> _labels;
			std::vector<double> _tp;
			std::vector<double> _fp;
			std::vector<double> _tn;
			std::vector<double> _fn;
			
			
			//Measures
			std::vector<double> _f1_score_individual;
			//Also known as the fowlkes-mallows index
			std::vector<double> _g_score_individual;
			std::vector<double> _accuracy_individual;
			std::vector<double> _matthewCC_individual;
			//Diagnostic odds ratio
			std::vector<double> _dor_individual;
			
			double _mean_p;
			double _mean_r;
			double _mean_ir;
			double _sd_p;
			double _sd_r;
			double _sd_ir;
			
			double _f1score;
			double _gscore;
			double _accuracy;
			double _matthewCC;
			double _dor;
			
			
			
		public:
			Evaluation(){}
			
			void addPrecision(double p){_precision.push_back(p);}
			void addRecall(double p){_recall.push_back(p);}
			void addInverseRecall(double p){_inverse_recall.push_back(p);}
			void addTime(double p){_time.push_back(p);}
			void addName(std::string& m){_name.push_back(m);}
			void clear(){_precision.clear(); _recall.clear(); _inverse_recall.clear();}
			
			double getMeanPrecision(){return _mean_p;}
			double getMeanRecall(){return _mean_r;}
			double getMeanInverseRecall(){return _mean_ir;}
			double getSDPrecision(){return _sd_p;}
			double getSDRecall(){return _sd_r;}
			double getSDInverseRecall(){return _sd_ir;}
			
			void calculate();
			
			size_t size(){return _precision.size();}
			void compare(const cv::Mat& seg, const cv::Mat& GT_segmentation, double time, const std::string& file);
			void exportAll(const std::string& file_out);
			double getMatthewsCC(){return _matthewCC;};
			double getFscore(){return _f1score;}
			double getGscore(){return _gscore;}
			double getDOR(){return _dor;}
			double getAccuracy(){return _accuracy;}
			
		private:
			void computeMeasures();
			void compareImagesUnbiased(cv::Mat GT_segmentation_in, cv::Mat DuDe_segmentation_in);
			
		};

		
	}
}

#endif