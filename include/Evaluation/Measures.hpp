#ifndef MAORIS_MEASURES_01082017
#define MAORIS_MEASURES_01082017

#include <cmath>

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
			return ( (tp * tn) - (fp * fn) ) / ( std::sqrt( (tp + fp) * (tp + fn) * (tn + fp) * (tn + fn) ) );
		}
		
		
		
	}
}
#endif