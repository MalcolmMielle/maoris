#ifndef MAORIS_MEANSHIFT_13062016
#define MAORIS_MEANSHIFT_13062016

#include <iterator>

#include <opencv2/opencv.hpp>

namespace AASS{
	namespace maoris{
		
		class MeanShift{
		protected:
		public:
			MeanShift() {}
			void meanshift(cv::Mat& img, cv::Mat& dest, const cv::Mat& model = cv::Mat(), int background = 255){
				
				cv::Mat tmp;
				cv::cvtColor(img, tmp, CV_GRAY2RGB);
				cv::pyrMeanShiftFiltering(tmp, dest, 12, 25, 5);
				cv::cvtColor(dest, dest, CV_RGB2GRAY);
				
				if(model.empty() == false){
					//Just making sure that their is a threshold between the background and the rest.
					for(int row = 0 ; row < model.rows ; row++){
						const uchar* pointer = model.ptr<uchar>(row); //point to each row
						uchar* pointer_dest = dest.ptr<uchar>(row); //point to each row
						for(int col = 0 ; col < model.cols ; col++){
							if(pointer[col] > background && pointer_dest[col] < background + 20){
								pointer_dest[col] = background + 20;
							}
						}				
					}
				}
				
			}
		};
		
	}
}


#endif