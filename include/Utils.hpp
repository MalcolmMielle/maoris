#ifndef MAORIS_UTILS_18052016
#define MAORIS_UTILS_18052016

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace AASS{
	namespace maoris{
		inline std::string type2str(int type) {
		std::string r;

		uchar depth = type & CV_MAT_DEPTH_MASK;
		uchar chans = 1 + (type >> CV_CN_SHIFT);

		switch ( depth ) {
			case CV_8U:  r = "8U"; break;
			case CV_8S:  r = "8S"; break;
			case CV_16U: r = "16U"; break;
			case CV_16S: r = "16S"; break;
			case CV_32S: r = "32S"; break;
			case CV_32F: r = "32F"; break;
			case CV_64F: r = "64F"; break;
			default:     r = "User"; break;
		}

		r += "C";
		r += (chans+'0');

		return r;
		}
		
		
		template<typename T>
		inline T mean(std::vector<T>& in){
			T sum = 0 ;
			for(auto it = in.begin() ; it != in.end() ; ++it){
				sum = sum + *it;
			}
			sum = sum / in.size();
			return sum;
		}
			
		template<typename T>
		inline T variance(std::vector<T>& in, T mean){
			T vari = 0 ;
			for(auto it = in.begin() ; it != in.end() ; ++it){
				T temp_el = *it - mean;
				temp_el = temp_el * temp_el;
				vari = vari + temp_el;
			}
			vari = vari / (in.size() - 1);
			return vari;
		}
		
		template<typename T>
		inline T sd(T variance){
			return std::sqrt(variance);
		}
		
		
		
		
		
		
	}
}

#endif