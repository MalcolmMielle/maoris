#include "FuzzyOpening.hpp"

//Needs to be int now
void AASS::maoris::FuzzyOpening::addPointValueInCircle(cv::Mat& input, cv::Mat& output, int value)
{
// 	std::cout << "INPUT " << std::endl << input << std::endl;
	
// 	std::cout << "OUTPUT " << std::endl << output << std::endl;
	//Making circle;
	cv::Mat element;
	
	//TODO : make circle drawing faster ! The problem is escentially the drawing time of the circles
	//If the algorithm doesn't need to run fast, we run a circluar element for more accurate result
	if(_fast == false){
// 		std::cout << "Value" << value << std::endl;
		element = _masks.at(value - 1);
// 		cv::imshow("MASK", element);
// 		cv::waitKey(0);
// 		element = cv::Mat::zeros((value * 2) + 2 , (value * 2) + 2, CV_32F);
// 		cv::circle(element, cv::Point2i((value), (value)), (value), cv::Scalar(1), -1);

		
		//Needs to be not even
// 		element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*value +1, 2*value +1), cv::Point(value-1 , value-1 ) );
// 		std::cout << value << std::endl << element << std::endl;
	}
	//To go fast we use a rectangular element for comparison
	else{
		element = cv::Mat::ones(value * 2 , value * 2, CV_32F);
	}
	
// 	std::cout << "CERLCE " << std::endl << circle << std::endl;
	
	if(input.rows > element.rows || input.cols > element.cols){
		std::ostringstream str_test;
		str_test <<  "Input is bigger than elemtn at line " << __LINE__ << " in file " << __FILE__ << "." << std::endl << "Input rows and col : " << input.rows << " " << input.cols << " element : " << element.rows << " " << element.cols ;
		throw std::runtime_error(str_test.str() );
	}
	
	
	for(int row = 0 ; row < input.rows ; row++){
		float* p = input.ptr<float>(row); //point to each row
		float* p_output = output.ptr<float>(row); //point to each row
		float* p_element = element.ptr<float>(row); //point to each row
		for(int col = 0 ; col < input.cols ; col++){
			//p[j] <- how to access element
	// 				std::cout << (int)p[j]<< std::endl;
// 			std::cout << "Comparing " << (float)p[col] << " and " << value << " while circle " << p_circle[col] << std::endl;
			if((float)p[col] > 0 && (float)p_output[col] < value && (float)p_element[col] > 0){
				p_output[col] = value;
			}
// 			if((float)p[col] == 0 && p_circle[col] > 0){
// 				p[col] = value;
// 			}
		}
	}
// 	return true;
	
	

}

void AASS::maoris::FuzzyOpening::fuzzyOpening(const cv::Mat& src, cv::Mat& output, int size)
{
	
	createAllMasks(size);
	
// 	cv:imshow("SRCSRC", src);
	
	//Calcul distance image
	cv::Mat distance_image, label;
	if(src.channels() == 3){
		std::cout << "Convert" << std::endl;
		cv::cvtColor(src, distance_image, cv::COLOR_RGB2GRAY);
	}
	else{
		src.copyTo(distance_image);
	}
	
// 	distance_image.convertTo(CV_);
	
	//I should do it here because it can lead to problem later when certain point in original map are forgotten
// 	cv::threshold(distance_image, distance_image, 50, 255, CV_THRESH_BINARY_INV);
	cv::distanceTransform(distance_image, distance_image, label, cv::DIST_L2, cv::DIST_MASK_PRECISE, cv::DIST_LABEL_CCOMP);
// 	std::cout << " TYPE " << type2str(distance_image.type()) << std::endl;	
	CV_Assert(distance_image.depth() == CV_32F);
	
// 	cv::normalize(distance_image, distance_image, 0, 255, cv::NORM_MINMAX, CV_8U);
// 	cv::imshow("dista img", distance_image);
// 	cv::waitKey(0);
	
	int pad = distance_image.rows;
	int old_rows = distance_image.rows;
	int old_cols = distance_image.cols;
	if(pad < distance_image.cols){
		pad = distance_image.cols;
	}
	
	cv::copyMakeBorder( distance_image, distance_image, pad, pad, pad, pad, cv::BORDER_CONSTANT, 0 );
	
	output = cv::Mat::zeros(distance_image.rows, distance_image.cols, CV_32F);
	cv::Mat roi_output_final = output(cv::Rect(pad, pad, old_cols, old_rows));
// 	std::cout << roi_output_final << std::endl;
	//Update result 
	int count = 0;
	for(int row = pad ; row < distance_image.rows - pad + 1 /*&& count < 15000*/ ; row++){
		// std::cout << "1 row \n";
		float* p = distance_image.ptr<float>(row); //point to each row
		float* p_output = output.ptr<float>(row); //point to each row
		for(int col = pad ; col < distance_image.cols - pad + 1 /*&& count < 15000*/ ; col++){
			
			int dist_to_obstacle = (float)p[col];
			dist_to_obstacle;
			if(dist_to_obstacle > size){
				dist_to_obstacle = size;
			}
			else{
// 				std::cout << "size is good " << dist_to_obstacle <<  std::endl;
			}
// 			std::cout << row << " " << col << " " << " dist : " <<dist_to_obstacle << " same " << (float)p[col] << " | " << std::endl;
			
			cv::Mat roi = distance_image(cv::Rect(col - (dist_to_obstacle), row - (dist_to_obstacle), dist_to_obstacle * 2, dist_to_obstacle * 2));
			cv::Mat roi_output = output(cv::Rect(col - (dist_to_obstacle), row - (dist_to_obstacle), dist_to_obstacle * 2, dist_to_obstacle * 2));
			if((int)dist_to_obstacle > 0){
				addPointValueInCircle(roi, roi_output, dist_to_obstacle);
// 				cv::imshow("out", roi_output);
// 				cv::imshow("roi", roi);
// 				cv::imshow("dist", distance_image);
// 				cv::imshow("roi_output_final", roi_output_final);
// 				cv::waitKey(1);
			}
			else{
				p_output[col] = 0;
// 				std::cout << "Wall" << std::endl;
			}
			
			
			
			
			
		}
// 		std::cout << std::endl;
	}
	
	//Need to be two different matrices now
	cv::Mat outout;
	cv::normalize(roi_output_final, outout, 0, 255, cv::NORM_MINMAX, CV_8U);
	
// 	std::cout << roi_output_final << std::endl;
	
// 	cv::imshow("out", outout);
// 	cv::waitKey(0);
	
	output = outout;

}

//I don't need that in the end 
bool AASS::maoris::FuzzyOpening::circleIsEmpty(cv::Mat& input, cv::Mat& circle)
{
	for(int row = 0 ; row < input.rows ; row++){
		uchar* p = input.ptr(row); //point to each row
		uchar* p_circle = circle.ptr(row); //point to each row
		for(int col = 0 ; col < input.cols ; col++){
			//p[j] <- how to access element
	// 				std::cout << (int)p[j]<< std::endl;
			if(p[col] > 0 && p_circle[col] > 0){
				return false;
			}
		}
	}
	return true;
	
	
}

