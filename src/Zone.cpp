#include "Zone.hpp"

		
void AASS::maoris::Zone::updateContour()
{
	cv::Mat copy_tmp;
	_zone_mat.copyTo(copy_tmp);
	
	std::vector< std::vector< cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(copy_tmp, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	
// 				assert(contours.size() >= 1 && "More than one shape in Zone");
	
	// Calculate the area of each contour
	
	//Use a lambda function to sort the contours
	std::sort(contours.begin(), contours.end(), [](std::vector<cv::Point> &pts, std::vector<cv::Point> &pts2){return pts.size() > pts2.size(); } );
	
	if(contours.size() > 1)
	{
// 		cv::Mat graphmat2 = cv::Mat::zeros(600,600, CV_8U);
// 		drawZone(graphmat2, cv::Scalar(255));
// 		drawContour(graphmat2, cv::Scalar(255));
// // 					std::cout << contours[0].size() << " " << contours[1].size() << std::endl;
// // 					if(contours[0].size() < contours[1].size()){
// // 						cv::Mat graphmat2 = cv::Mat::zeros(600,600, CV_8U);
// // 						for (int i = 0; i < contours[0].size(); ++i)
// // 						{
// // 							graphmat2.at<uchar>(contours[0][i].y, contours[0][i].x) = 255;
// // 						}
// 			cv::imshow("fina", graphmat2);
// // 						cv::waitKey(0);	
// // 						throw std::runtime_error("Calculating PCA with zone not in order");
// // 					}
// 		cv::waitKey(0);
		
// 		throw ZoneHasMoreThanOneContour("MORE THAN ONE CONTOUR !");
	}
	if(contours.size() == 0 ){
		throw ZoneHasNoContour("NO CONTOUR IN ZONE !");
	}
// 	std::cout << "Contour size " << contours.size() << std::endl;
	_contours = contours;
}

