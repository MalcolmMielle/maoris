#ifndef MAORIS_EVALUATION_31072017
#define MAORIS_EVALUATION_31072017

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <sys/stat.h>
#include <cmath>
#include <algorithm>

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
		
		///Represent all point in a GT in contact with a segmented zone
		class ZoneTest{
		public:
			std::vector<cv::Point> all_point;
			int pixel_value;
		public:
			ZoneTest(int pix) : pixel_value(pix){}
			void addPoint(const cv::Point & p){all_point.push_back(p);}
			size_t size() const {return all_point.size();}
			void push_back(const cv::Point& p){all_point.push_back(p);}
			void push_back(const std::vector<cv::Point>& p){
				for(auto it = p.begin() ; it < p.end(); ++it){
					all_point.push_back(*it);
				}
			}
			void print(){
				for( auto it = all_point.begin() ; it != all_point.end() ; ++it ){	
					std::cout << " " << pixel_value<< ":" << all_point.size() << " - ";
				}
			}
			
		};
		
		///Store all GT zone in contact with a given Segmented Zone
		class ZoneAsso{
		public:
			std::vector<ZoneTest> other_zones;
			int pixel_value;
		public:
			ZoneAsso(int pixe) : pixel_value(pixe){}
			bool addPoint(int pix, cv::Point p){
				for(auto it = other_zones.begin() ; it != other_zones.end() ; ++it){
					if(it->pixel_value == pix){
						it->push_back(p);
						return 0;
					}
				}
				ZoneTest z(pix);
				z.push_back(p);
				other_zones.push_back(z);
			}
			void addTag(const tag2points& tag){
				for( auto it2 = tag.begin(); it2!= tag.end(); it2++ ){
					ZoneTest z(it2->first);
					z.push_back(it2->second);
					other_zones.push_back(z);
				}
			}
			
			
			void sort(){
				std::sort(other_zones.begin(), other_zones.end(), []( const ZoneTest& a, const ZoneTest& b)
				{						
					return a.size() > b.size();
				});
				
		
			}
			
			void print(){
				std::cout << "Zone " << pixel_value << " ";
				for( auto it = other_zones.begin() ; it != other_zones.end() ; ++it ){	
					it->print();
				}
				std::cout << std::endl;
			}
			
			
			
		};
		
		//Keep all association of Segmented and GT
		class AllZoneAsso{
		public:

			std::vector<int> non_asso_gt;
			std::vector<int> non_asso_seg;

			std::vector<ZoneAsso> zones;
			
			std::vector <std::pair<int, int> > associations;
			
			AllZoneAsso(){};
			
			size_t size() const {return associations.size();}
			void FromTag(const tag2tagMapper& tag){
				for( auto it2 = tag.begin(); it2!= tag.end(); it2++ ){
//					std::cout << "Adding " << it2-> first << std::endl;
					ZoneAsso z(it2->first);
					z.addTag(it2->second);
					zones.push_back(z);
				}
				assert(zones.size() == tag.size());
			}
			
			void sort(){
				for( auto it = zones.begin() ; it != zones.end() ; ++it ){	
					it->sort();
				}
				std::sort(zones.begin(), zones.end(), []( const ZoneAsso& a, const ZoneAsso& b)
				{						
					return a.other_zones[0].size() > b.other_zones[0].size();
				});
				
		
			}

			bool isAsso(int i){
				for(auto it = associations.begin() ; it != associations.end() ; ++it){
					if(i == it->second){
						return true;
					}
				}
				return false;
			}

			bool isAssoSeg(int i){
				for(auto it = associations.begin() ; it != associations.end() ; ++it){
					if(i == it->first){
						return true;
					}
				}
				return false;
			}
			
			void calculateAsso(){
// 				print();
				sort();
				std::cout << "\nAfter sort" << std::endl;
// 				print();
				for( auto it = zones.begin() ; it != zones.end() ; ++it ){	
					int segmented = it->pixel_value;
					int pos = 0;
					int gt = -1;

					//Explore until we find a non associated gt.
					while(pos < it->other_zones.size() && (isAsso(gt) || gt == -1 ) ){
						gt = it->other_zones[pos].pixel_value;
						++pos;
					}
					
					if(gt != -1){
						associations.push_back(std::pair<int, int>(segmented, gt));
					}
					//TODO HANDLE NON ASSOCIATION
					// of both
					
				}

				//Non associated gt with empty
				for( auto it = zones.begin() ; it != zones.end() ; ++it ){
					int segmented = it->pixel_value;

					if(!isAssoSeg(segmented)) {
						non_asso_seg.push_back(segmented);
					}

					int pos = 0;
					int gt = -1;

					//Explore until we find a non associated gt.
					for(int pos = 0; pos < it->other_zones.size(); ++pos) {
						gt = it->other_zones[pos].pixel_value;
						if (!isAsso(gt)) {
							non_asso_gt.push_back(gt);
						}
					}
					//TODO HANDLE NON ASSOCIATION
					// of both
				}
			}



			
			void print(){
				for( auto it = zones.begin() ; it != zones.end() ; ++it ){	
					it->print();
				}
			}
			
			
			
		};

		
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
			std::vector<double> _matthewCC_perzone_individual;
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
			double _matthewCC_median;
			double _matthewCC_perzone;
			double _matthewCC_median_perzone;
			double _sd_mCC;
			double _dor;
			
			double _max_mcc;
			double _min_mcc;
			double _max_mcc_perzone;
			double _min_mcc_perzone;
			
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
			double getSDMatthewCC(){return _sd_mCC;}

			double getMax(){return _max_mcc;}
			double getMin(){return _min_mcc;}
			double getMaxPerZone(){return _max_mcc_perzone;}
			double getMinPerZone(){return _min_mcc_perzone;}
			
			void calculate();
			
			size_t size(){return _precision.size();}
			void compare(const cv::Mat& seg, const cv::Mat& GT_segmentation, double time, const std::string& file);
			void exportAll(const std::string& file_out);
			double getMatthewsCC(){return _matthewCC;};
            double getMatthewsCCMedian(){return _matthewCC_median;};
			double getMatthewsCCPerZone(){return _matthewCC_perzone;};
            double getMatthewsCCMedianPerZone(){return _matthewCC_median_perzone;};
            std::vector<double> getAllMatthewsCC(){return _matthewCC_individual;};
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
