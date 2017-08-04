#ifndef MAORIS_HISTOGRAM_20052016
#define MAORIS_HISTOGRAM_20052016

#include <opencv2/opencv.hpp>
#include "Utils.hpp"

namespace AASS{
	namespace maoris{
		
		/** @brief Used to find the number of cluster needed by Kmean ?
		 */
		class Histogram{
		
		public:
			Histogram(){}
			
			void histogram(cv::Mat src){
				
				cv::imshow("INPUT HISTO", src);
				
				std::vector<cv::Mat> bgr_planes;
				cv::split( src, bgr_planes );	
				
				/// Establish the number of bins
				int histSize = 256;
				float range[] = { 0, 256 } ;
				const float* histRange = { range };	
				cv::Mat b_hist, g_hist, r_hist;
				bool uniform = true; bool accumulate = false;
				cv::calcHist( &bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
				
				
				int hist_w = 512; int hist_h = 400;
				int bin_w = cvRound( (double) hist_w/histSize );

				cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

				/// Normalize the result to [ 0, histImage.rows ]
				cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
// 				cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
// 				cv::normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

				/// Draw for each channel
				for( int i = 1; i < histSize; i++ ){
					cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
									cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
									cv::Scalar( 255, 0, 0), 2, 8, 0  );
// 					cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
// 									cv::Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
// 									cv::Scalar( 0, 255, 0), 2, 8, 0  );
// 					cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
// 									cv::Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
// 									cv::Scalar( 0, 0, 255), 2, 8, 0  );
				}

				/// Display
				cv::namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
				cv::imshow("calcHist Demo", histImage );
				cv::waitKey(0);

			}
			
		};
		
	}
}

#endif