#include "Kmean.hpp"

void AASS::maoris::Kmeans::kmeansColor(cv::Mat& in, cv::Mat& dest, const cv::Mat& sketch, int backrgound_color_model){
	
	clear();
	
	cv::imshow("input", in);
	cv::Mat input;
	std::cout << "TYPE " << type2str(in.type()) << std::endl;
	in.convertTo(input, CV_8UC1);
	
	cv::Mat p = cv::Mat::zeros(input.cols*input.rows, 3, CV_32F);
	std::vector<cv::Mat> bgr;
	cv::split(input, bgr);

	for(int i = 0 ; i < input.cols*input.rows ; i++) {
		p.at<float>(i,0) = (i/input.cols) / input.rows; //Needs to be zero because it fails when I seem to put the good values...
		p.at<float>(i,1) = (i%input.cols) / input.cols;
		p.at<float>(i,2) =  bgr[0].data[i] / 255.0;
	}

	cv::kmeans(p, _K, _bestLabels, cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, _centers);
	
	int colors[_K];
	for(int i=0; i<_K; i++) {
		colors[i] = 255/(i+1);
	}
	// i think there is a better way to do this mayebe some Mat::reshape?
	dest = cv::Mat::zeros(input.rows, input.cols, CV_8U);
	std::cout << "TYPE " << type2str(_centers.type()) << " size " << _centers.rows << " " << _centers.cols << std::endl;
// 				
	for(int row = 0 ; row < _centers.rows ; row++){
		float* pointer_center = _centers.ptr<float>(row); //point to each row		
		_colors.push_back( (int) ( ((float)pointer_center[2] * 255) ) );
		
	}
	std::sort(_colors.begin(), _colors.end());
	
	uint32_t* pointer_label = _bestLabels.ptr<uint32_t>(0); //point to each row
	
	//Map all point in dest image to corresponding color
	for(int row = 0 ; row < input.rows ; row++){
		uchar* pointer = dest.ptr<uchar>(row); //point to each row
		uchar* pointer_src = in.ptr<uchar>(row); //point to each row
		for(int col = 0 ; col < input.cols ; col++){
			//Extract the value of kmean at point row,col
			float* pointer_center = _centers.ptr<float>(pointer_label[(row * input.cols) + col]);
			if(sketch.empty() == false){
				//For image with model, make sure we didn't suppress information from background
				if(sketch.at<uchar>(row, col) != backrgound_color_model ){
					//The color associated to the point is more that the lowest color which is assumed to be the background color
					if( (int) ((float)pointer_center[2] * 255) > _colors[0]){
						pointer[col] = (int) ( ((float)pointer_center[2] * 255) );
					}
					//Otherwise add some kind of minimal color
					else{
						pointer[col] = 20; /*( _colors[0] + _colors[1] ) / 2;*/
					}
				}
			}
			else{
				pointer[col] = (int) ( ((float)pointer_center[2] * 255) );
			}

			
		}				
	} 

}



// void AASS::maoris::Kmeans::meanShift(cv::Mat& in, cv::Mat& dest)
// {
// 
// 	cv::meanShift(in, );
// }
