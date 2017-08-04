#ifndef MAORIS_WATERSHED_17052016
#define MAORIS_WATERSHED_17052016

#include "Zone.hpp"
#include "Utils.hpp"
#include "GraphZone.hpp"
#include "bettergraph/SimpleGraph.hpp"

namespace AASS{
	namespace maoris{
		
		inline bool sortFunction(std::pair < size_t, size_t > i, std::pair < size_t, size_t > j){return (i.second > j.second); }
		
// 		inline bool sortZone(ZoneLight i, ZoneLight j){return (i.size() > j.size  ()); }

		//TODO convert Mat to Eigen !

		class ZoneExtractor{
			
		protected :
			
			typedef typename GraphZone::GraphType GraphZoneType;
			typedef typename GraphZone::VertexIterator VertexZoneIterator;
			typedef typename GraphZone::Vertex VertexZone;
			typedef typename GraphZone::Edge EdgeZone;
			typedef typename GraphZone::EdgeIterator EdgeZoneIterator;	
			
			std::vector < std::pair < size_t, size_t > > _index_of_zones_to_fuse_after;
			std::set < std::pair < size_t, size_t > > _index_of_zones_to_fuse_after_set;
			std::map<size_t, size_t> _mapping;
			std::map<size_t, int> _mapping_of_node_alive;
			std::deque< Zone > _zones;
			//int as edge because we don't care aboput edge 
			std::set < std::pair < size_t, size_t > > _index_of_edges_set;
			std::vector < std::pair < size_t, size_t > > _index_of_edges;
			GraphZone _graph;
			
			std::set<unsigned int> _values_to_ignore;
			
		public:
			ZoneExtractor(){
				//Cheat for obstacle being 0
// 				Zone z; _zones.push_back(z);
			}
			
			void addValueToIgnore(unsigned int i){_values_to_ignore.insert(i);}
			
			size_t size(){return _zones.size();}
			GraphZone& getGraph(){return _graph;}
			size_t getNumEdge(){return _index_of_edges.size();}
			/**
			* @brief Main algorithm : extract all zone of same value from Matrix and store them in _zones
			* @param in Matrix from where the zone are to be extracted
			*/
			void extract(cv::Mat& in);

			/**
			* @brief Print all zone and values
			*/
			void print(){
				std::cout << "Print watershed" << std::endl;
				
				for(size_t i = 0 ; i < _zones.size() ; ++i){
					std::cout << "Zone " << i << " of size " << _zones[i].size() << " and value " << _zones[i].getValue() << std::endl;
		// 			for(size_t j = 0 ; j < _zones[i].size() ; ++j){
		// 				std::cout << " x : " << _zones[i][j].x << " y : " << _zones[i][j].y << std::endl;
		// 			}
				}
				
				std::cout << "Number of zones " << _zones.size() << std::endl;
				
			}
			
			/**
			* @brief Print zone number i
			*/
			void printZone(int i){
				for(size_t j = 0 ; j < _zones[i].size() ; ++j){
					std::cout << " x : " << _zones[i][j].x << " y : " << _zones[i][j].y << std::endl;
				}
			}
			
			/**
			* @brief Draw all zones one by one
			* @param draw input Matrix defining the size of the drawing matrix
			* @param size Minimum size of zone for it to be drawn
			*/ 
			void drawAllZones(cv::Mat& draw, int size){
		// 		std::cout << "_zone size " << _zones.size() << std::endl;
				
				for(size_t i = 0 ; i < _zones.size() ; ++i){
					if(_zones[i].size() > size){
						cv::Mat draw_tmp = cv::Mat::zeros(draw.rows, draw.cols, CV_8U);
						for(size_t j = 0 ; j < _zones[i].size() ; ++j){
							draw_tmp.at<uchar>(_zones[i][j].x, _zones[i][j].y) = 255;
						}
						cv::imshow("Zones " , draw_tmp);
						cv::waitKey(0);
						
					}
				
				}
			
			}
			
			/**
			* @brief Testing drawing sortFunction
			*/
			void slowdraw(cv::Mat& draw, int size){
				std::cout << "_zone size " << _zones.size() << std::endl;
				cv::Mat draw_tmp = cv::Mat::zeros(draw.rows, draw.cols, CV_8U);
				for(size_t i = 0 ; i < _zones.size() ; ++i){
					if ( i > 1){
						
						if(_zones[i].size() > size){
							
							for(size_t j = 0 ; j < _zones[i].size() ; ++j){
								draw_tmp.at<uchar>(_zones[i][j].x, _zones[i][j].y) = 255;
							}
		// 					std::cout << "VALUE " << i << std::endl;
							
						}
						
					}
				}
				if(_zones.size() >1){
					cv::imshow("Zones " , draw_tmp);
					cv::waitKey(1);
				}
			}
			
			
		private :
			/**
			* @brief Create a first version of the zone in Matrix
			* Swype through the in put matrix once and extract a certain number of zones stored in _zones. 
			* Actual zones might be separated in multiple parts and one needs to use the function  ZoneExtractor::fuse after. Zones to be fused are stored by id in _index_of_zones_to_fuse_after
			* @param input Input matrix with the values
			*/ 
			void makeZones(cv::Mat& input);
			/**
			* @brief Fuse zone that were meant to be together
			* Zones to be fused are stored by id in _index_of_zones_to_fuse_after. It first create a mapping of all fusion to be done and uses linear copy of decks
			*/
			void fuse();
			/**
			* @brief return all the pixel around a given pixel belonging to an existing cluster
			* @param value Value of studied pixel
			* @param input cv::Mat input with value
			* @param zones_star cv::Mat with value of cluster for each pixel
			* @param row row of studied pixel
			* @param col col of studied pixel
			* @param zone_index std::vector containing the index of all connected pixel from the same type of zone (zone with the same value)
			* @param zone_edges std::vector containing the index of all connected pixel from a different type of zone (zone with the same value)
			*/
			void isolatedOrNot(int value, cv::Mat& input, cv::Mat& zones_star, int row, int col, std::vector< unsigned int >& zone_index, std::vector< unsigned int >& zone_edges);
			
			void createGraph();
			
			/// @brief Add key to Map and update the whole structure
			void updateMapping(int from, int to){
				_mapping_of_node_alive[from] = -1;
				_mapping[from] = to;
				std::map<size_t, size_t>::iterator iter;
				for (iter = _mapping.begin(); iter != _mapping.end(); ++iter) {
					if(iter->second == from){
						iter->second = to ;
					}
				}
				
				std::map<size_t, int>::iterator iter_2;
				for (iter_2 = _mapping_of_node_alive.begin(); iter_2 != _mapping_of_node_alive.end(); ++iter_2) {
					if(iter_2->first > from){
						iter_2->second = iter_2->second - 1 ;
					}
				}
			}
			
			///@brief init _mapping_of_node_alive
			void initMappingAlive(){
				std::map<size_t, int>::iterator iter;
				for (size_t i = 0 ; i < _zones.size() ; ++i){
					_mapping_of_node_alive[i] = i;
				}
			}
			
			
		};

	}
}


#endif