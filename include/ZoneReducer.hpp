#ifndef MAORIS_ZONEREDUCER_30052016
#define MAORIS_ZONEREDUCER_30052016

#include "GraphZone.hpp"

namespace AASS{
	
	namespace maoris{
		
// 		class ZoneReducer{
// 		protected:
// 			
// 		public:
// 			ZoneReducer(){}
// 		
// 		};
		
		
		/**
		 * @brief return a version of cv::Mat in where every value is the upper multiple of value_zone closest to the pixel value
		 */
		inline void reduceZone(cv::Mat& in, cv::Mat& dest, int value_zone = 10){
			
			if ( in.empty() )
			{
				throw std::runtime_error("input matrix is empty");
			}
			in.convertTo(dest, CV_8U);
			if ( dest.empty() )
			{
				throw std::runtime_error("dest matrix is empty");
			}
			
			cv::normalize(dest, dest, 0, 255, cv::NORM_MINMAX, CV_8U);
		
			for(int row = 0 ; row < dest.rows ; row++){
				uchar* p = dest.ptr<uchar>(row); //point to each row
				for(int col = 0 ; col < dest.cols ; col++){
					
					while(p[col]%value_zone != 0 && p[col] != 255){
						++p[col];
					}
				}
			}
		}
		
		
	}
}


#endif
