#ifndef MAORIS_KMEANS_18052016
#define MAORIS_KMEANS_18052016

#include <opencv2/opencv.hpp>
#include "Utils.hpp"

namespace AASS{
	namespace maoris{
		
		class Kmeans{
		protected :
			int _K;
			cv::Mat _bestLabels, _centers;
			
			///@brief All the colors used in the kmean in increasing order
			std::vector< int > _colors;
			
		public:
			Kmeans() : _K(4){}
			
			void setK(int k){_K = k ;}
			int getK(){return _K;}
			
			void printColors() {
				for(size_t i = 0 ; i < _colors.size() ; ++i){
					std::cout << " color " << i << " : " << _colors[i] << std::endl;
				}
			}
			
			std::vector< int > getColors(){return _colors;}
			const std::vector< int >& getColors() const {return _colors;}
			
			/**
			 * @brief Kmean algorithm
			 * @param in : Mat input
			 * @param dest : Mat destination
			 * @param sketch : if not empty, Mat used to rectify the color zone wrongfully added to the background
			 */
			void kmeansColor(cv::Mat& in, cv::Mat& dest, const cv::Mat& sketch = cv::Mat(), int backrgound_color_model = 255);
			
			void getSilhouette();
			
			//TODO : look up if meanShift could be used
// 			void meanShift(cv::Mat& in, cv::Mat& dest);
			
			void clear(){
				_bestLabels.release();
				_centers.release();
				_colors.clear();
			}
			
		};
	}
}
#endif