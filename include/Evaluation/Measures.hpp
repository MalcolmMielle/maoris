#ifndef MAORIS_MEASURES_01082017
#define MAORIS_MEASURES_01082017

#include <cmath>
#include <stdio.h>
#include <algorithm>

namespace AASS{
	namespace maoris{
		
		template<typename T>
		inline T fscore(T precision, T recall){
			return 2 * (precision * recall) / (precision + recall);
		}
		
		template<typename T>
		inline T gscore(T precision, T recall){
			return std::sqrt(precision * recall);
		}
		
		template<typename T>
		inline T DOR(T tp, T fp, T tn, T fn){
			return (tp / fp) / (fn / tn);
		}
		
		template<typename T>
		inline T accuracy(T tp, T fp, T tn, T fn){
			return (tp + tn) / (tp + tn + fp + fn);
		}
		
		template<typename T>
		inline T matthewCC(T tp, T fp, T tn, T fn){

			assert(tp >= 0);
			assert(fp >= 0);
			assert(tn >= 0);
			assert(fn >= 0);

			std::cout << "( (" << tp<< " * "<< tn<< ") - (" << fp << " * " << fn << ") ) / ( std::sqrt( (" << tp << " + " << fp << ") * (" << tp << " + " << fn << ") * (" << tn << " + " << fp << ") * (" << tn << " + " << fn << ") ) ) " << std::endl;
			auto mcc = ( (tp * tn) - (fp * fn) ) / ( std::sqrt( (tp + fp) * (tp + fn) * (tn + fp) * (tn + fn) ) );
			assert(mcc >= 0 && mcc <= 1);
			return mcc;
		}
		
		template<typename T>
		inline T median( std::vector<T> values){
			double median;
			size_t size = values.size();

			std::sort(values.begin(), values.end());

			if (size  % 2 == 0)
			{
				median = (values[size / 2 - 1] + values[size / 2]) / 2;
			}
			else 
			{
				median = values[size / 2];
			}

			return median;
		}
		
		
		
	}
}
#endif