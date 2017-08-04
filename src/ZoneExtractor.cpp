#include "ZoneExtractor.hpp"

void AASS::maoris::ZoneExtractor::extract(cv::Mat& in)
{
	//Make zones
	
	makeZones(in);
	
	//Fuse zones with close value
	fuse(); 
	
	std::cout << "Number of zones " << _zones.size() << std::endl;
	//Prune small zones
	
	//Return zones and links
	
	createGraph();

}

/*Two way : I can either swype one way creating zone
* OR
* I can for every value only keep a certain value -> FloodFill all spaces while marking the pixel that disappeared as a zone
*/
void AASS::maoris::ZoneExtractor::makeZones(cv::Mat& input)
{
	
// 	cv::imshow("input water ", input);
// 	cv::waitKey(0);
	std::cout << "Make zone : " << std::endl;
	cv::Mat in;
// 	std::cout << input << std::endl;
	input.convertTo(in, CV_32SC1);
// 	std::cout << input <<std::endl;
// 	exit(0);
	// Mat with 0 when pixel has not been seen or is a wall and a number when it belong to a zone
	cv::Mat zones_star = cv::Mat::ones(in.rows, in.cols, in.depth());
	zones_star = - zones_star;
// 	std::cout << zones_star << std::endl;
	
	
// 	std::cout << in <<std::endl;
// 	exit(0);
	_index_of_zones_to_fuse_after.clear();
	_graph.clear();

	// Swype in one way and add pixel to zones
	int step = 0 ;
	for(int row = 0 ; row < in.rows ; row++){
// 		std::cout << "Start loop " << std::endl;
		int* p = in.ptr<int>(row); //point to each row
		int* p_zone_star = zones_star.ptr<int>(row); //point to each row
		for(int col = 0 ; col < in.cols ; col++){
// 			slowdraw(in, 0);
			assert(_zones.size() <= in.rows * in.cols );
// 			for(int i = 0 ; i < _zones.size() ; ++i){
// 				assert(_zones[i].size() <= in.rows * in.cols);
// 			}
// 			std::cout << "Step : " << step << " row and col " << row << " " << col << " and " << in.rows << " " << in.cols << " multi " << in.rows * in.cols << " zone size" << sizeof(Zone) <<  std::endl;
// 			std::cout << "Zone size " << _zones.size() << std::endl;
// 			std::cout << "Full size " << (sizeof(std::vector<Zone>) + _zones.size() * sizeof(Zone))/1024.0f <<std::endl;
			++step;
			
			//If the pixel is not a wall
			if(_values_to_ignore.count(p[col]) == 0 ){

				std::vector<unsigned int> zone_index;
				std::vector<unsigned int> zone_edges;
				
				isolatedOrNot(p[col], in, zones_star, row, col, zone_index, zone_edges);
				
				
				//New zone since the pixel is connected to no  already seen + same value pixel
				if(zone_index.size() == 0){
	// 				std::cout << "New Zone" << std::endl;
					Zone new_zone(input.size());
					new_zone.setValue(p[col]);
					new_zone.push_back(cv::Point2i(row, col));
					_zones.push_back(new_zone);
					p_zone_star[col] = (int)_zones.size() - 1;					
				}
				//If the pixel is part of (an) already seen zone(s)
				else{
	// 				std::cout << " NOT new Zone" << std::endl;
					_zones[ zone_index[0] ].push_back(cv::Point2i(row, col));
					p_zone_star[col] = zone_index [0];
					
					if(_zones[ zone_index[0] ].getValue() != p[col]){
						throw std::runtime_error("Different value between the place and the zone it's added to");
					}
					
					//If more than one zone push the value of other zones to _index_of_zones_to_fuse_after to fuse them later
					if(zone_index.size() > 1){
						bool flag_exist = false;
						for(size_t i = 1 ; i < zone_index.size() ; ++i){
							
							//Sort them by index value
							size_t min = std::min(zone_index[0], zone_index[i]);
							size_t max = std::max(zone_index[0], zone_index[i]);
// 							if(min > zone_index[i]){
// 								min = zone_index[i];
// 								max = zone_index[0];
// 							}
							
							//Make sure the pair does not already exist
// 							for(size_t j = 0 ; j < _index_of_zones_to_fuse_after.size() ; ++j){
// 								if( (min == _index_of_zones_to_fuse_after[j].first &&\
// 									max == _index_of_zones_to_fuse_after[j].second) ){
// 									flag_exist = true;
// 								}
// 							}
							if(_index_of_zones_to_fuse_after_set.count(std::pair<size_t, size_t>(min, max)) == 0){
// 							if(flag_exist == false){
								_index_of_zones_to_fuse_after.push_back(std::pair<size_t, size_t>(min, max) );
								_index_of_zones_to_fuse_after_set.insert(std::pair<size_t, size_t>(min, max) );
							}
						}
						
					}
				}
				
				
	// 			std::cout << "Zone edge " << zone_edges.size() << std::endl;
				//Adding edges if needs be done
				if(zone_edges.size() > 0){
					
	// 				std::cout << "Adding edges " << _index_of_edges.size() << std::endl;
					bool flag_exist = false;
					for(size_t i = 0 ; i < zone_edges.size() ; ++i){
						
						//Sort them by index value
						
						unsigned int min = std::min( (unsigned int) p_zone_star[col], zone_edges[i]);
						unsigned int max = std::max( (unsigned int) p_zone_star[col], zone_edges[i]);
						
// 						size_t min = p_zone_star[col];
// 						size_t max = zone_edges[i];
// 						if(min > zone_edges[i]){
// 							min = zone_edges[i];
// 							max = p_zone_star[col];
// 						}
// 						if(min == max){
// 							throw std::runtime_error("linking zone to itself !");
// 						}
						
						//Make sure the pair does not already exist
// 						for(size_t j = 0 ; j < _index_of_edges.size() ; ++j){
// 							if( (min == _index_of_edges[j].first &&\
// 								max == _index_of_edges[j].second) ){
// 	// 							std::cout << "EDGE ALREADY EXIST ??" << std::endl;
// 								flag_exist = true;
// 							}
// 						}
						if(_index_of_edges_set.count(std::pair<size_t, size_t>(min, max)) == 0){
							_index_of_edges_set.insert(std::pair<size_t, size_t>(min, max) );
							_index_of_edges.push_back(std::pair<size_t, size_t>(min, max) );
						}
					}
					
				}
			
// 			std::cout << "end of loop " << std::endl;
			
			}
			
		}
		
	}
	
	std::cout << "Zone made" << std::endl;
	
}

void AASS::maoris::ZoneExtractor::isolatedOrNot(int value, cv::Mat& input, cv::Mat& zones_star, int row, int col, std::vector< unsigned int >& zone_index, std::vector< unsigned int >& zone_edges)
{

	if(zones_star.size() != input.size()){
		throw std::runtime_error("Zone start and input of different sizes");
	}
	
	for(int row_tmp = row - 1 ; row_tmp <= row + 1 ; ++row_tmp){
		for(int col_tmp = col - 1 ; col_tmp <= col + 1 ; ++col_tmp){
			//Inside mat
			if(row_tmp >= 0 && col_tmp >=0 && row_tmp < input.rows && col_tmp < input.cols){
				
// 				std::cout << "Inside of mat " << row_tmp <<"  "<< col_tmp << std::endl;
				
				//Not the center point
				
				if(row != row_tmp || col != col_tmp){
					//Not the diagonal
					if(row_tmp == row || col_tmp == col){
						int* p = input.ptr<int>(row_tmp);
						int* p_star = zones_star.ptr<int>(row_tmp);
						
	// 					std::cout << "Not center and vaues are " << value <<" " << p[col_tmp] << " " << p_star[col_tmp] << std::endl;

						//Same value and visited before
						if(value == p[col_tmp] && p_star[col_tmp] >= 0){
							
							if(_zones[p_star[col_tmp]].getValue() != value){
								printZone(p_star[col_tmp]);
								std::cout << "Size of matrix : " << zones_star.size() << std::endl;
								std::cout << "At : row col " << row_tmp << " " << col_tmp << " we should not have " << _zones[p_star[col_tmp]].getValue() << " != " << value << std::endl;
								throw std::runtime_error("Value and zone mismatch");
							}
							//SAME ZONE
							bool flag_seen = false;
							for(size_t i = 0 ; i < zone_index.size() ; ++i){
								if(zone_index[i] == p_star[col_tmp]){
									flag_seen = true;
								}
							}
							if(flag_seen == false){
								zone_index.push_back(p_star[col_tmp]);
							}
						}
						
						//Check for other value bordering
						else if(value != p[col_tmp] && p_star[col_tmp] >= 0){
							
	// 						std::cout <<"Found edge" << std::endl;
							
	// 						std::cout << "Was ? " << value << " != " << _zones[p_star[col_tmp]].getValue() << " " << p[col_tmp] << std::endl << "zone : " << p_star[col_tmp] << std::endl;
							
							if(_zones[p_star[col_tmp]].getValue() == value){
	// 							printZone(p_star[col_tmp]);
								std::cout << "At : row col " << row_tmp << " " << col_tmp << std::endl;
								throw std::runtime_error("Value and zone are the same in edges");
							}
							//SAME ZONE
							bool flag_seen = false;
							for(size_t i = 0 ; i < zone_edges.size() ; ++i){
								if(zone_edges[i] == p_star[col_tmp]){
									flag_seen = true;
								}
							}
							if(flag_seen == false){
	// 							std::cout << "PUSHING BACK :D" << std::endl;
								zone_edges.push_back(p_star[col_tmp]);
							}
						}
					
						else if(p_star[col_tmp] >= 0){
							std::runtime_error("Something should have happenéd and it didn't. We found a value but it was not linked at the same or an edge");
						}
					}
				}
			}
		}
	}
	
// 	int a;
// 	std::cout << "ALL THE EDGES  : " << _index_of_edges.size() << "and later " << zone_edges.size() << std::endl;
// 	for(auto iter_edge = _index_of_edges.begin(); iter_edge != _index_of_edges.end() ; ++iter_edge){
// 		std::cout << "Edge : " << iter_edge->first << " -> " << iter_edge->second << std::endl;
// 	}
// 	std::cin >> a;
}

void AASS::maoris::ZoneExtractor::fuse()
{
	std::cout << "Fuse" << std::endl;
	for(size_t i = 0 ; i < _zones.size() ; ++i){
		if(_zones[i].isEmpty()){
			print();
			throw std::runtime_error("The zone is empty and it shouldn't happen");
		}
	}
	
	
	initMappingAlive();
	
// 	std::cout << "FUSE" << std::endl;
	
// 	print();
	
// 	std::cout << " ^- Before the fuse" << std::endl;
	
	//Sort the index backward from higest value for second so that we erase the zone from the back without changing the indexes.
	std::sort(_index_of_zones_to_fuse_after.begin(), _index_of_zones_to_fuse_after.end(), sortFunction);
	
// 	for(size_t i = 0 ; i < _zones.size() ; ++i){
// 		std::cout << "Zone " << i << " of size " << _zones[i].size() << std::endl;
// 	}
// 	for(size_t i = 0 ; i < _index_of_zones_to_fuse_after.size() ; ++i){
// 		int base = _index_of_zones_to_fuse_after[i].first;
// 		int to_fuse = _index_of_zones_to_fuse_after[i].second;
// 		std::cout << "Zone to fuse " << _index_of_zones_to_fuse_after[i].first << " " << _index_of_zones_to_fuse_after[i].second << " with values " << _zones[base].getValue() << " " << _zones[to_fuse].getValue() << std::endl;
// 	}
	
	_mapping.clear();	
	//Create the full mapping
	for(size_t i = 0 ; i < _index_of_zones_to_fuse_after.size() ; ++i){
// 		
		int base;
		int to_fuse;
		//Search the value in the mapping and update it
		if ( _mapping.find(_index_of_zones_to_fuse_after[i].first) == _mapping.end() ) {
		// not found
// 			std::cout << "not FOUND base " << _index_of_zones_to_fuse_after[i].first << std::endl;
			base = _index_of_zones_to_fuse_after[i].first;
		} else {
		// found
// 			std::cout << "FOUND base " << _index_of_zones_to_fuse_after[i].first << std::endl;
			base = _mapping[_index_of_zones_to_fuse_after[i].first];
// 			std::cout << "Now base is " << base << std::endl;
		}
		if ( _mapping.find(_index_of_zones_to_fuse_after[i].second) == _mapping.end() ) {
		// not found
// 			std::cout << "not FOUND fuse " << _index_of_zones_to_fuse_after[i].second << std::endl;
			to_fuse = _index_of_zones_to_fuse_after[i].second;
			
// 			mapping[_index_of_zones_to_fuse_after[i].second] = base;
		} else {
		// found
// 			std::cout << "FOUND fuse " << _index_of_zones_to_fuse_after[i].second << std::endl;
			to_fuse = _mapping[_index_of_zones_to_fuse_after[i].second];
// 			std::cout << " now fuse " << to_fuse << std::endl;
// 			mapping[to_fuse] = base;
			
		}
		
		//Always copy to the smallest number
		int max = to_fuse;
		int min = base;
		if(min != max){
			
			if(max < min){
				max = base;
				min = to_fuse;
				
				updateMapping(max, min);
// 				_mapping[max] = min;
				_mapping[_index_of_zones_to_fuse_after[i].first] = min;
			}else{
				updateMapping(max, min);
				_mapping[_index_of_zones_to_fuse_after[i].second] = min;
			}
			
			if(_zones[base].getValue() != _zones[to_fuse].getValue() || base == to_fuse){
				std::cout << min << " " << max << std::endl;
				std::cout << _zones[base].getValue() << " " << _zones[to_fuse].getValue() << std::endl;
				std::ostringstream str_test;
				str_test <<  "Fusing zone that shouldn't be fused at line " << __LINE__ << " in file " << __FILE__ << "." << std::endl;
				throw std::runtime_error(str_test.str() );	
			}
		}
		else{
// 			std::cout << "Already fused" << std::endl;
		}
		
		
	}
	
	std::map<size_t, size_t>::reverse_iterator iter;
	for (iter = _mapping.rbegin(); iter != _mapping.rend(); ++iter) {
		for(size_t j = 0 ; j < _zones[iter->first].size() ; ++j){
			_zones[iter->second].push_back(_zones[ iter->first][j]);
		}
// 		std::cout << iter->first << " -> " << iter->second << std::endl;
		_zones.erase(_zones.begin() + iter->first );
	}
// 	std::sort(_zones.begin(), _zones.end(), sortZone);
	
// 	print();
// 	std::cout << "Removed " << _mapping.size() << " zones " << std::endl;
	std::cout << "Done fusing" << std::endl;
	
}

void AASS::maoris::ZoneExtractor::createGraph(){
	
// 	std::cout << "Create graph " << std::endl;
	std::map<size_t, int>::iterator iter;
	for (iter = _mapping_of_node_alive.begin(); iter != _mapping_of_node_alive.end(); ++iter) {
// 		std::cout << iter->first << " -> " << iter->second << std::endl;
	}
	
// 	std::cout << "Number of edges " << _index_of_edges.size() << std::endl;
	
// 	for(auto iter_edge = _index_of_edges.begin(); iter_edge != _index_of_edges.end() ; ++iter_edge){
// 		std::cout << "Edge : " << iter_edge->first << " -> " << iter_edge->second << std::endl;
// 	}
	
	
// 	exit(0);
	
	std::vector <VertexZone> vertices_zones;
	
	//Adding all vertices
	for(size_t i = 0 ; i < _zones.size() ; ++i){
// 		std::cout << "Adding a vertex " << i << std::endl;
		VertexZone v;
		if(_zones[i].isEmpty()){
			throw std::runtime_error("The zone is empty and it shouldn't happen");
		}
		_graph.addVertex(v,_zones[i]);
		vertices_zones.push_back(v);
	}
	
// 	std::cout << "adding the edges" << std::endl;
	
	//Adding all edge
	for(size_t i = 0 ; i < _index_of_edges.size() ; ++i){
// 		std::cout << " edge : "<< _index_of_edges[i].first << " " << _index_of_edges[i].second << std::endl;
		int base;
		int destination;
		//Search the value in the mapping and update it
		if ( _mapping.find(_index_of_edges[i].first) == _mapping.end() ) {
		// not found
// 			std::cout << "not FOUND base " << _index_of_zones_to_fuse_after[i].first << std::endl;
			base = _index_of_edges[i].first;
		} else {
		// found
// 			std::cout << "FOUND base " << _index_of_zones_to_fuse_after[i].first << std::endl;
			base = _mapping[_index_of_edges[i].first];
// 			std::cout << "Now base is " << base << std::endl;
		}
		if ( _mapping.find(_index_of_edges[i].second) == _mapping.end() ) {
		// not found
// 			std::cout << "not FOUND fuse " << _index_of_zones_to_fuse_after[i].second << std::endl;
			destination = _index_of_edges[i].second;
			
// 			mapping[_index_of_zones_to_fuse_after[i].second] = base;
		} else {
		// found
// 			std::cout << "FOUND fuse " << _index_of_zones_to_fuse_after[i].second << std::endl;
			destination = _mapping[_index_of_edges[i].second];
// 			std::cout << " now fuse " << to_fuse << std::endl;
// 			mapping[to_fuse] = base;
		}
		EdgeZone e;
// 		std::cout << "actual edge : "<< base << " " << destination << " " << _mapping[24] << std::endl;
		if ( _mapping_of_node_alive[base] > 0) {
		// found
// 			std::cout << "not FOUND base " << _index_of_zones_to_fuse_after[i].first << std::endl;
			base =  _mapping_of_node_alive[base];
		}
		if ( _mapping_of_node_alive[destination] > 0) {
		// found
// 			std::cout << "not FOUND base " << _index_of_zones_to_fuse_after[i].first << std::endl;
			destination =  _mapping_of_node_alive[destination];
		}
		
// 		std::cout << "actual actual edge : "<< base << " " << destination << " " << _mapping[24] << std::endl;
		_graph.addEdge(e, vertices_zones[ base ], vertices_zones[ destination ]);
	}
	
// 	std::cout << "Graph vertices : " << _graph.getNumVertices() << " egdes : " << _graph.getNumEdges() << std::endl;
	
	
// 	if(_graph.asVerticesWithNoEdges() == true){
// 		throw std::runtime_error("Some nodes are no linked");
// 	}
	
	if(_graph.getNumVertices() != _zones.size()){
		throw std::runtime_error("Graph and zone not of same dimensions");
	}
	
	
}
