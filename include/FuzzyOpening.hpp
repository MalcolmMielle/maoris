#ifndef MAORIS_FUZZYOPENING_16052016
#define MAORIS_FUZZYOPENING_16052016

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Utils.hpp"

namespace AASS{
	namespace maoris{
		/**
		* @brief Fuzzy opening on an image
		* Calculate the size of the biggest circle that any pixel can be fitted in. Circles don't have to be centered on the pixel considered.
		*/
		class FuzzyOpening{
			
		protected:
			/**
			* @brief Max distance from a point to a wall to be consider an open space.
			* All point that can fit in a circle with at least radius _size will be considered the same
			*/
// 			int _size;
			/**
			* @brief Fast or accurate
			*/
			bool _fast;
			/**
			* @brief vector of element object of different size for not having to generate them each time
			*/
		// 	std::vector<cv::Mat> _elements;
			/**
			* @brief id of element
			* 0 : circle
			* 1 : rectangle
			*/
		// 	size_t _element_id;
		
			std::deque<cv::Mat> _masks; //All circle of until size we need
			
		public:
			FuzzyOpening(): _fast(false){};
// 			void setSize(int i){_size = i;}
			
		// 	void generateElements();
		// 	void generateCircle(size_t size, cv::Mat& out);
			
			///@brief Determine if the algorithm needs to run fast or accuratly. Cannot be fast AND accurate for now
			void fast(bool fast){_fast = fast;}
			///@brief Determine if the algorithm needs to run fast or accuratly. Cannot be fast AND accurate for now
			void accurate(bool acc){_fast = !acc;}
			
			///@brief return true or false depending on if the circle discribed by circle is empty in input
			bool circleIsEmpty(cv::Mat& input, cv::Mat& circle);
			/**
			 * @brief Perform on opening on the map using either a circle or a square. Using the square is not recommended.
			 * Indeed using the square assume, no curve in the map and all walls parallele or perpendicular to to the horizon.
			 * Other the size of the square might be wrong if the distance is on the diagonal. For curves, use the circle element.
			 * Start with biggest circle and go down until empty
			 * @param[in] src : source image
			 * @param[out] output : output image
			 * @param[in] size : optionnal argument to define the mximum size of the element. Default value is 50
			 */
			void fuzzyOpening(const cv::Mat& src, cv::Mat& output, int size = 50);
			void addPointValueInCircle(cv::Mat& input, cv::Mat& output, int value);
			
			
			
		private:
			
			void createAllMasks(int size){
				
				if(size != _masks.size()){
					_masks.clear();
					for(size_t i = 0 ; i < size ; ++i){
						cv::Mat element = cv::Mat::zeros((i * 2) + 2 , (i * 2) + 2, CV_32F);
						cv::circle(element, cv::Point2i((i), (i)), (i), cv::Scalar(1), -1);
						_masks.push_back(element);
					}
					
				}
				
			}
			
			
		};

		// void FuzzyOpening::generateElements()
		// {
		// 	for(size_t i = 0 ; i < _size ; ++i){
		// // 		if()
		// 	}
		// 
		// }
	}
}





#endif