#ifndef MAORIS_ZONE_17052016
#define MAORIS_ZONE_17052016

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <sstream>

// #include "ZoneLight.hpp"

namespace AASS{
	
	namespace maoris{
		
		class ZoneHasMoreThanOneContour : public std::runtime_error {
		public:
			ZoneHasMoreThanOneContour(const std::string& message) 
				: std::runtime_error(message) { };
		};
		class ZoneHasNoContour : public std::runtime_error {
		public:
			ZoneHasNoContour(const std::string& message) 
				: std::runtime_error(message) { };
		};
		
// 		class ZoneCompared;
// 		class ZoneComparedInterface;
		
		/**
		 * bool _isUnique: true if the zone is unique, false otherwise. It is initialised as true, so that every new zone is considered ofr comparison unless setUniqueness as been explicitely called before.
		 * 
		 * bool _uniqueness_calculated: flag to check if the zone was initialised or not.
		 */
		class Zone{
			
		protected:
			bool _use_cvMat;
			
			int _value;
			
			//Change it for a set for constant time find !
			std::deque <cv::Point2i> _zone;
			///@brief sum of all value of x and y of all point in zone. For fast update and get of centroid
			cv::Point2i _sum_of_x_and_y;
			///@brief Zone drawn on the Mat
			cv::Mat _zone_mat;
			cv::Size _img_size;
						
			std::vector< std::vector< cv::Point2i> > _contours;
			
			
		public:
			Zone() : _use_cvMat(false) {};
			Zone(const cv::Size& size) : _use_cvMat(false), _img_size(size){
			};
			Zone(int rows, int cols) : _use_cvMat(false), _img_size(rows, cols){
			};
			
			
			void useCvMat(bool should_use){
				if(_use_cvMat == false && should_use == true){
					//Init the mat
					_zone_mat = cv::Mat::zeros(_img_size, CV_8U);
					for(int i = 0; i < _zone.size() ; ++i){
						_zone_mat.at<uchar>(_zone[i].x, _zone[i].y) = 255;
					}
					updateContour();
					
				}
				if(_use_cvMat == true && should_use == false){
					//Deallocate the mat
					_zone_mat = cv::Mat();
				}
				_use_cvMat = should_use;
				
			}
			
			bool useCvMat() const {return _use_cvMat;}
			
			void push_back(const cv::Point2i& p){_zone.push_back(p); addPoint(p);}
			void push_front(const cv::Point2i& p){_zone.push_front(p); addPoint(p);}
			
			void pop_back(){removePoint(_zone.size()-1); _zone.pop_back();}
			void pop_front(){removePoint(0); _zone.pop_front();}
			
			///@breif search and remove the point p(row, col)
			void removePoint(const cv::Point2i& p){
				removePoint(p.x, p.y);
			}
			
			void removePoint(int row, int col){
				auto position = std::find(_zone.begin(), _zone.end(), cv::Point2i(row, col) );
				if(position != _zone.end()){
					_zone.erase(position);
					if(_use_cvMat == true){
						_zone_mat.at<uchar>(row, col) = 0;
					}
				}
			}
			
			bool isEmpty(){return (0 == _zone.size());}
			int size(){return _zone.size();}
			void setImageSize(const cv::Size& in){_img_size = in;}
			
			size_t size() const {return _zone.size();}
			
			void clear(){
				_zone.clear(); 
				_value = 0; _sum_of_x_and_y.x = 0 ; _sum_of_x_and_y.y = 0; 
				_zone_mat = cv::Mat();
			}
			
			cv::Point2i& operator[](int i){return _zone[i];};
			const cv::Point2i& operator[](const int i) const {return _zone[i];};
			
			void setValue(size_t i){_value = i;}
			int getValue(){return _value;}
			int getValue() const {return _value;}
			const std::deque <cv::Point2i >& getZone() const {return _zone;}
		// 	std::deque <cv::Point2i >& getZone(){return _zone;}
			const cv::Mat& getZoneMat() const {return _zone_mat;}
			
			cv::Point2i getSumXY() const { return _sum_of_x_and_y;}
			cv::Size getSizeImg() const {return _img_size;}
			
			///@brief return the center of the zone by doing the mean of all the points
			cv::Point2i getCentroid(){
				if( _zone.size() == 0 ){
					throw std::runtime_error("zone is empty");
				}
				return cv::Point2i(_sum_of_x_and_y.y / _zone.size(), _sum_of_x_and_y.x / _zone.size());	
			}
			///@brief return the center of the zone by doing the mean of all the points
			const cv::Point2i getCentroid() const {
				if( _zone.size() == 0 ){
					throw std::runtime_error("zone is empty");
				}
				return cv::Point2i(_sum_of_x_and_y.y / _zone.size(), _sum_of_x_and_y.x / _zone.size());		
			}
			
			
			void fuse(const Zone& input){
// 				assert(input.getValue() < _value);
				for(size_t i = 0 ; i < input.size() ; ++i){
					this->push_back(input.getZone()[i]);
				}
				if(_use_cvMat == true){
					try{
						updateContour();
					}
					catch(const std::runtime_error& e){
						std::cout << "Warning: " << e.what() << std::endl;
					}
				}
				
// 				cv::Mat graphmat2 = cv::Mat::zeros(600,600, CV_8U);
// 				draw(graphmat2, cv::Scalar(100));
// 				cv::imshow("fused", graphmat2);
// 				cv::waitKey(0);	
// 				std::cout << "Fused now pca " << std::endl;
				//Do not updated PCA because sometime when doing recursive fusion, we must wait until all zones have been fused in the initial to perform fusion OR use throw mechanism but it's complex for no reason
// 				PCA();
			}
			
// 			void initMat(){
// 				_zone_mat = cv::Mat::zeros(_img_size, CV_8U);
// 				for(int i = 0; i < _zone.size() ; ++i){
// 					_zone_mat.at<uchar>(_zone[i].x, _zone[i].y) = 255;
// 				}
// 				
// // 				cv::imshow("Vertex ", _zone_mat);
// // 				cv::waitKey(0);
// 				
// 			}
			
			void drawZone(cv::Mat& img, const cv::Scalar& color) const{
				img.convertTo(img, CV_8U);
				uchar colllo = color[0];
				for(int i = 0; i < _zone.size() ; ++i){
					img.at<uchar>(_zone[i].x, _zone[i].y) = colllo;
				}					
			}
			
			///@brief simply draw the map
			void drawContour(cv::Mat& img, const cv::Scalar& color) const{				
				for (int i = 0; i < _contours.size(); ++i)
				{
					for (int j = 0; j < _contours[i].size(); ++j)
					{
						img.at<uchar>(_contours[i][j].y, _contours[i][j].x) = 255;
					}
				}
			}
			
			
			void printLabel(cv::Mat& img) const{
				std::string text;
				text = std::to_string(_value);
				std::string textsi;
				textsi = std::to_string(size());
				std::stringstream precisionValue;
				precisionValue.precision(2);
// 				text = text + " sd " + precisionValue.str() ;
// 				text = text + ":s:" + textsi;
				cv::putText(img, text, getCentroid(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
			}
			
			///Update the contour before giving it to always be up to date
			std::vector< std::vector< cv::Point > > getContour(){
				updateContour();
				return _contours;
			}
			
			///Do not update the contour since it's unmmutable. Need to call update before to be sure.
			std::vector< std::vector< cv::Point > > getContour() const {
// 				updateContour();
				return _contours;
			}
			

			///@brief return the points in contact between the zones
			std::vector< std::vector<cv::Point2i > > getContactPointSeparated(const Zone& zone){
				assert(_use_cvMat == true && "We need the opencv::Mat for this function");
				assert(zone.useCvMat() == true && "We need the opencv::Mat for this function. input zone");
				
				std::vector< std::vector< cv::Point2i> > contact_point;

				cv::Mat copyTest;
				zone.getZoneMat().copyTo(copyTest);
				
// 				cv::imshow("i",zone.getZoneMat());
// 				cv::imshow("b",_zone_mat);
// 				cv::Mat copy_tmp;
// 				_zone_mat.copyTo(copy_tmp);
				
// 				std::vector< std::vector< cv::Point> > contours;
// 				std::vector<cv::Vec4i> hierarchy;
// 				cv::findContours(copy_tmp, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
								
				//Do for all contour and add percentage
				// Calculate the area of each contour

				
				auto lambda = [](int x, int y, const cv::Mat& mat) -> bool{
					
					int xx;
					for( xx = x - 1 ; xx < x + 2 ; ++xx ){
						int yy;
						for( yy = y - 1 ; yy < y + 2 ; ++yy ){
// 							std::cout << " x y " << xx << " " << yy << std::endl;
							if(mat.at<uchar>(yy, xx) > 0){
								return true;
							}
						}
					}
					return false;

				};
				
				assert(_contours.size() > 0 && "there is no contour :S");
				
				
				
				
				
				std::vector<std::pair<int, int > > seen;
				std::vector<cv::Point2i> tmp;
				bool fadding_points = false;
				for(int j = 0 ; j < _contours.size() ; ++j){
					
					auto it = _contours[j].begin();
					while(lambda(it->x, it->y, copyTest)){
						++it;
						if(it == _contours[j].end()){
							throw std::runtime_error("Terrible contour for zone :(");
						}
					}
					
					for(int i = 0 ; i < _contours[j].size() ; ++i){
				
						bool asbeenseeen = false;
						for(size_t i = 0 ; i < seen.size() ; ++i){
							if(seen[i].first == it->x && seen[i].second == it->y){
								asbeenseeen = true;
							}
						}
						seen.push_back(std::pair<int, int>(it->x, it->y));
						if(!asbeenseeen){
							if(lambda(it->x, it->y, copyTest)){
								tmp.push_back(*it);
								fadding_points = true;
							}
							else if(fadding_points == true){
								contact_point.push_back(tmp);
								tmp.clear();
								fadding_points = false;
							}
						}
						
						++it;
						if(it == _contours[j].end()) it = _contours[j].begin();
						
					}
					if(tmp.size() != 0) contact_point.push_back(tmp);
				}
				auto contact_test = getContactPoint(zone);
				int size = 0;
				for(auto it = contact_point.begin() ; it != contact_point.end() ; ++it){
					size = size + it->size();
				}
				
				if(contact_point.size() == 0){
					
					std::cout << "New contour " << size << " old contour " << contact_test.size() << std::endl;
					
					cv::Mat graphmat2 = cv::Mat::zeros(600,600, CV_8U);
					for(auto it = _contours.begin() ; it != _contours.end() ; ++it){
						for(auto it2 = it->begin() ; it2 != it->end() ; ++it2){
							graphmat2.at<uchar>(it2->y, it2->x) = 255;
						}
						
					}
					cv::imshow("fina", graphmat2);
					std::cout << " No contours" << std::endl;
					cv::imshow("the zone",_zone_mat);
					cv::imshow("from", copyTest);
					cv::waitKey(0);
				}
				return contact_point;
				
			}
			
			///@brief return the point in contact between the zones
			std::vector<cv::Point2i> getContactPoint(const Zone& zone){
// 				std::cout << "Get contact " << std::endl;
				assert(_use_cvMat == true && "We need the opencv::Mat for this function");
				assert(zone.useCvMat() == true && "We need the opencv::Mat for this function. input zone");
				
				std::vector<cv::Point2i> contact_point;

				cv::Mat copyTest;
				zone.getZoneMat().copyTo(copyTest);
				
// 				cv::imshow("i",zone.getZoneMat());
// 				cv::imshow("b",_zone_mat);
// 				cv::Mat copy_tmp;
// 				_zone_mat.copyTo(copy_tmp);
				
// 				std::vector< std::vector< cv::Point> > contours;
// 				std::vector<cv::Vec4i> hierarchy;
// 				cv::findContours(copy_tmp, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
								
				//Do for all contour and add percentage
				// Calculate the area of each contour

				
				auto lambda = [](int x, int y, const cv::Mat& mat) -> bool{
					
					int xx;
					for( xx = x - 1 ; xx < x + 2 ; ++xx ){
						int yy;
						for( yy = y - 1 ; yy < y + 2 ; ++yy ){
// 							std::cout << " x y " << xx << " " << yy << std::endl;
							if(mat.at<uchar>(yy, xx) == 255){
								return true;
							}
						}
					}
					return false;

				};
				
				assert(_contours.size() > 0 && "there is no contour :S");
// 				std::cout << "NUMBER OF CONTOUR " << _contours.size() << std::endl;
// 				for(size_t i = 0 ; i < _contours.size() ; i++){
// 					auto contour = _contours[i];
					
// 					final_size = final_size + contour.size();
					
					std::vector<std::pair<int, int > > seen;
					
// 					cv::Mat matcon = cv::Mat::zeros(_zone_mat.size(), CV_8U);
					for(int j = 0 ; j < _contours.size() ; ++j){
						for(auto it = _contours[j].begin() ; it != _contours[j].end() ; ++it ){
	// 						matcon.at<uchar>(it->y, it->x) = 255;
	// 						std::cout << "LAMBDAS" << std::endl;
							bool asbeenseeen = false;
							for(size_t i = 0 ; i < seen.size() ; ++i){
								if(seen[i].first == it->x && seen[i].second == it->y){
									asbeenseeen = true;
								}
							}
							seen.push_back(std::pair<int, int>(it->x, it->y));
							if(!asbeenseeen){
								if(lambda(it->x, it->y, copyTest)){
									contact_point.push_back(*it);
								}
							}

						}
					}

				return contact_point;
// 				}
			}
			
			//Return the number of contact points in percent compared to size of contour. DO NOT need an update from PCA() or updateContours.
			int contactPoint(const Zone& zone){
				
// 				assert(_use_cvMat == true && "We need the opencv::Mat for this function");
// 				
// 				int whitepix = 0 ;
// 				int final_size = 0;
// 				int test_size = 0 ;
// 
// 				cv::Mat copyTest;
// 				zone.getZoneMat().copyTo(copyTest);
// 				
// // 				cv::imshow("i",zone.getZoneMat());
// // 				cv::imshow("b",_zone_mat);
// 				cv::Mat copy_tmp;
// 				_zone_mat.copyTo(copy_tmp);
// 				
// 				std::vector< std::vector< cv::Point> > contours;
// 				std::vector<cv::Vec4i> hierarchy;
// 				cv::findContours(copy_tmp, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
// 								
// 				//Do for all contour and add percentage
// 				// Calculate the area of each contour
// 
// 				
// 				auto lambda = [](int x, int y, const cv::Mat& mat) -> bool{
// 					
// 					int xx;
// 					for( xx = x - 1 ; xx < x + 2 ; ++xx ){
// 						int yy;
// 						for( yy = y - 1 ; yy < y + 2 ; ++yy ){
// // 							std::cout << " x y " << xx << " " << yy << std::endl;
// 							if(mat.at<uchar>(yy, xx) == 255){
// 								return true;
// 							}
// 						}
// 					}
// 					return false;
// 
// 				};
// 				
// // 				std::cout << "NUMBER OF CONTOUR " << contours.size() << " with " << contours[0].size() << std::endl;
// 				for(size_t i = 0 ; i < contours.size() ; i++){
// 					auto contour = contours[i];
// 					
// // 					final_size = final_size + contour.size();
// 					
// 					std::vector<std::pair<int, int > > seen;
// 					
// // 					cv::Mat matcon = cv::Mat::zeros(_zone_mat.size(), CV_8U);
// 					for(auto it = contour.begin() ; it != contour.end() ; ++it ){
// // 						matcon.at<uchar>(it->y, it->x) = 255;
// // 						std::cout << "LAMBDAS" << std::endl;
// 						bool asbeenseeen = false;
// 						for(size_t i = 0 ; i < seen.size() ; ++i){
// 							if(seen[i].first == it->x && seen[i].second == it->y){
// 								asbeenseeen = true;
// 							}
// 						}
// 						seen.push_back(std::pair<int, int>(it->x, it->y));
// 						if(!asbeenseeen){
// 							if(lambda(it->x, it->y, copyTest)){
// 								whitepix++;
// 							}
// 							final_size++;
// 						}
// 
// 					}
// 
// 				}
				
				auto contact_point = getContactPoint(zone);
				
				//Can't store cv::Point directly because no "<" operator.
				std::set<std::pair<int, int> > all_contour;
				for(auto it = _contours.begin() ; it != _contours.end() ; ++it){
					for(auto it2 = it->begin(); it2 != it->end() ; ++it2){
						all_contour.insert(std::pair<int, int>(it2->x, it2->y) );
					}
				}
				
				auto percent = contact_point.size() * 100 / all_contour.size();
// 				std::cout << "Percent " << percent << std::endl;
				
// 				cv::imshow("input",zone.getZoneMat());
// 				cv::imshow("base",_zone_mat);
// 				cv::waitKey(0);
				
// 				assert(percent < 50 && "Percentage can't be more than 50% because that means the shape is flat");
				return percent;
				
			}
			
			
			/** @brief comparison this is where I choose to use PCA or not. The lowest the score, the better the matching. Result is between 0 and 1
			 */
			
// 			ZoneComparedInterface compare(const AASS::RSI::Zone& zone_in) const;
			
			void updateContour();
			
		private: 
			void addPoint(const cv::Point2i& p){
				_sum_of_x_and_y.x = _sum_of_x_and_y.x + p.x;
				_sum_of_x_and_y.y = _sum_of_x_and_y.y + p.y;
				//TODO drawing function of new point
				if(_use_cvMat){
					_zone_mat.at<uchar>(p.x, p.y) = 255;
				}
			}
			void removePoint(int i){
				_sum_of_x_and_y.x = _sum_of_x_and_y.x - _zone[i].x;
				_sum_of_x_and_y.y = _sum_of_x_and_y.y - _zone[i].y;
				//TODO un-drawing function of new point
				if(_use_cvMat){
					_zone_mat.at<uchar>(_zone[i].x, _zone[i].y) = 0;
				}
			}
			

			
		};
		
		
		inline std::ostream& operator<<(std::ostream& in, const Zone &p){
			
			in << "v " << p.getValue() << " s " << p.size();

			return in;
			
		}
		
		inline std::istream& operator>>(std::istream& in, Zone &p){
			char tmp;
			in >> tmp;
			std::cout << "v " << tmp << std::endl;
			int vall;
			
			in >> vall;
// 			std::cout << "v " << vall << std::endl;
			in >> tmp;
			int vall_s;
			in >> vall_s;
			
			p.setValue(vall);
					
			return in;
		}
	}
}

#endif
