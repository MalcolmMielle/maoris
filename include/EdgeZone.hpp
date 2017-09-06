#ifndef MAORIS_EDGEZONE_27052016
#define MAORIS_EDGEZONE_27052016

#include <boost/math/distributions/normal.hpp>

#include <Zone.hpp>
#include "bettergraph/SimpleGraph.hpp"

namespace AASS{
	
	namespace maoris{
		
		class EdgeElement;
		
// 		struct ElementPairVertex{
// 			typedef typename bettergraph::SimpleGraph<Zone, EdgeElement>::Vertex VertexZone;
// 			VertexZone from;
// 			VertexZone toward;
// 			double score;
// 			///Minimum value between from and toward if ripple where added. It's -1 if not initialised and it's -2 if this is an original link from the first graph.
// 			double min_toward;
// 		};
		
		class EdgeElement{
		protected:
			double _diff;
			bool _breakable;
			bool _fromFuse;
			
			
			typedef typename bettergraph::SimpleGraph<Zone, EdgeElement>::Vertex VertexZone;
// 			std::vector<ElementPairVertex > _score_old;
			
			double min_toward;
		public:
			///init at -2 
			
			EdgeElement() : _diff(0), _breakable(true), _fromFuse(false), min_toward(-2){};
			
			EdgeElement(const EdgeElement& ed) : _diff(ed.getDiff()), _breakable(true), min_toward(-2){
// 				for (auto it = ed.getOldScore().begin() ; it != ed.getOldScore().end() ; ++it){
// 					_score_old.push_back(*it);
// 				}
			};
			
			void setMinimum(double in){_fromFuse = true; min_toward = in;}
			double getMinimum(){return min_toward;}
			double getMinimum() const {return min_toward;}
			
			bool wasRipple(){return _fromFuse;}
// 			size_t size(){return _score_old.size();}
// 			size_t size() const {return _score_old.size();}
// 			const std::vector<ElementPairVertex>& getOldScore() const {return _score_old;}
			double getDiff(){return _diff;}
			double getDiff() const {return _diff;}
			void setDiff(double diff){_diff = diff;}
			void makeUnbreakable(){_breakable = false;}
			bool canRemove() const {return _breakable;}
			
// 			double getOldScore(const VertexZone& v, const VertexZone& v_toward){
// 				for (auto it = _score_old.begin() ; it != _score_old.end() ; ++it){
// 					if(v == it->from && v_toward == it->toward){
// 						assert(it->score >=0);
// 						assert(it->score <= 255);
// 						return it->score;
// 					}
// 				}
// 				return -1;
// 			}
// 			
// 			void setOldScore(const VertexZone& from, const VertexZone& toward, double score){
// 				
// 				_fromFuse= true;
// 				
// 				ElementPairVertex el;
// 				el.from = from;
// 				el.toward = toward;
// 				el.score = score;
// 				bool found = false;
// 				//Find if node exist
// // 				std::cout << "Yolo" << _score_old.size() << std::endl;
// 				
// 				//If the exact same link exist
// 				for (auto it = _score_old.begin() ; it != _score_old.end() ;){
// 					if(from == it->from && toward == it->toward){
// // 						std::cout << "Set old score" << std::endl;
// 						found = true;
// 						el.score = it->score;
// 						el.min_toward = it->min_toward;
// 						//Keep the score closest to the start value. -2 means direct link
// 						if(el.min_toward <= score && el.min_toward != -2){
// 							el.min_toward = score;
// 						}
// 						auto it_cpy = it - 1;
// 						_score_old.erase(it);
// 						it = it_cpy +1;
// 					}
// 					else{
// 						++it;
// 					}
// // 					std::cout << "done" << std::endl;
// 				}
// // 				std::cout << "Push back" << std::endl;
// 				_score_old.push_back(el);
// 				
// 				//If we might create new links
// 				if(found == false){
// 					
// 					bool f_init = false;
// 					ElementPairVertex el;
// 					el.toward = toward;
// 					el.score = -1;
// 					el.min_toward = -1;
// 					for (auto it = _score_old.begin() ; it != _score_old.end() ; ++it){
// 						if(from == it->toward){
// 							ElementPairVertex el;
// 							el.from = it->from;
// 							el.min_toward = it->min_toward;
// 							el.min_toward = score;
// 							if(it->min_toward >= el.min_toward ){
// 								el.min_toward = it->min_toward;								
// 							}
// 							f_init = true;
// 						}
// 					}
// 					
// 					_score_old.push_back(el);
// 				}
// 				
// 			}
			
			//Test if need to be makeUnbreakable thanks to all value of zone fused in it
			bool shouldBeUnbreakable(double max_start, const VertexZone& start, double max_end, const VertexZone& end, double thresh){

				double max = std::max(max_start, max_end);
// 				std::cout << " GOOD max " << max << " min " << min_toward << std::endl;
				if(min_toward == -2){
					return false;
				}
				else if(min_toward <= max - (max * thresh)){
					return true;
				}
				return false;
				
			}
						
		};
		
		
		inline int getDirection(const EdgeElement& edg, const Zone& src, const Zone& targ, double thresh = 0){
			if(edg.getDiff() < thresh){
				if(src.getValue() > targ.getValue()){
					//Going down
					return 1;
				}
				else{
					//Going Up
					return 2;
				}
			}
			//Same value
			return 0;
			
		}
		
		
		inline std::ostream& operator<<(std::ostream& in, const EdgeElement &p){
			
			in << "d " << p.getDiff();
			return in;
			
		}
		
		inline std::istream& operator>>(std::istream& in, EdgeElement &p){
			char tmp; 
			in >> tmp;
			std::cout << tmp << " ";
			//TODO change to double
			int input;
			in >> input;
			std::cout << input << std::endl;
			p.setDiff(input);
			return in;
		}
		
		
	}
	
}


#endif