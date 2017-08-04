#ifndef MAORIS_GRAPHZONE_19052016
#define MAORIS_GRAPHZONE_19052016

#include <iterator>
#include <forward_list>
#include <type_traits>

#include "Zone.hpp"
#include "EdgeZone.hpp"
#include "Utils.hpp"
#include "bettergraph/SimpleGraph.hpp"
// #include "ZoneCompared.hpp"


namespace AASS{
	namespace maoris{
		
		template<typename VertexType, typename EdgeType>
		class GraphZoneInterface : public bettergraph::SimpleGraph<VertexType, EdgeType>{
			
			static_assert(std::is_base_of<Zone, VertexType>::value, "VertexType must derive from Zone");
			static_assert(std::is_base_of<EdgeElement, EdgeType>::value, "EdgeType must derive from EdgeElement");
			
		public:
			typedef typename bettergraph::SimpleGraph<Zone, EdgeElement>::GraphType GraphZoneType;
			typedef typename bettergraph::SimpleGraph<Zone, EdgeElement>::Vertex VertexZone;
			typedef typename bettergraph::SimpleGraph<Zone, EdgeElement>::Edge EdgeZone;
			typedef typename bettergraph::SimpleGraph<Zone, EdgeElement>::VertexIterator VertexIteratorZone;
			typedef typename bettergraph::SimpleGraph<Zone, EdgeElement>::EdgeIterator EdgeIteratorZone;
			
			GraphZoneInterface(){}
			
			///@brief draw a simple image with no annotations.
			void drawSimple(cv::Mat& drawmat) const;
			void drawSimple(cv::Mat& m, const VertexZone& v, const cv::Scalar& color) const;
			///@brief draw map with all info on image
			void draw(cv::Mat& drawmat) const;
			void draw(cv::Mat& m, const VertexZone& v, const cv::Scalar& color) const;
			///@brief draw the zone one by one.
			void drawPartial(cv::Mat& drawmat) const;
			///@brief drawing function to compare the maps to the dataset of Bromman.
			void drawEvaluation(cv::Mat& drawmat) const;
			void drawEvaluation(cv::Mat& m, const VertexZone& v, const cv::Scalar& color) const;
			
			void drawContours(cv::Mat& drawmat) const;
			void drawContours(cv::Mat& m, const VertexZone& v, const cv::Scalar& color) const;	
			
		};


		//TODO convert Mat to Eigen !
		// ATTENTION: if uniqueness is calculated and then a node is added, then the flag to check if the uniqueness was updated is wrong... Need to do it either every time a node is added but it will need to move the addVertex function here.

		class GraphZone : public GraphZoneInterface<Zone, EdgeElement>{
			
		private:
			//TEST
			int _iteration;
			
		protected:
			
			
			double _margin_factor;
		
			 ///@param[in] threshold : fraction representing the fraction of the biggest value of cluster until a new cluster must created. If init node got value 100 and threshold = 1/10 then if the new node as 90 or less, it is not fused.
			double _threshold;
			
		public:
			

			GraphZone(): _margin_factor(0.10), _threshold(0.25){};
			
			
			void setThreshold(double t){if(t >= 0 && t<= 1){_threshold = t;}else{throw std::runtime_error("Threhsold needs to be between 0 and 1");}}
			void setMargin(double m){if(m >= 0 && m<= 1){_margin_factor = m;}else{throw std::runtime_error("Margin needs to be between 0 and 1");}}
			
			double getT(){return _threshold;}
			

			
			///@brief Remove all vertex with Value value. Do not preserve edges or zones
			void removeVertexValue(int value){
				std::pair<VertexIteratorZone, VertexIteratorZone> vp;
				//vertices access all the vertix
				for (vp = boost::vertices((*this)); vp.first != vp.second;) {
					VertexZone v = *vp.first; 
					++vp.first;
					// std::cout << "Value " << (*this)[v].getValue() << std::endl;
					if((*this)[v].getValue() == value){
// 						std::cout << "Trying to remove" << std::endl;
						removeVertex(v);
// 						std::cout << "Removed" << std::endl;
					}
				}
				removeLonelyVertices();
			}
			
			/**
			 * @brief TODO : spurious branches removed on a zone
			 * Method :
			 * -> Finding only node with one link does not work because it maybe be in between two nodes
			 * -> Should use watershed algorithm + SOMETHING
			 */
// 			void removeRiples(){};
			
			/** @brief Remove all vertex which zone is less than size in size.
			 * @param size size under which the vertex gets removed
			 * @param preserveEdgeConnectic if true the connection between edge is "preserved" i.e if we have Node1 - Node2 - Node3 and we remove Node2 the end result will be Node1 - Node3. Plus the zone of the removed vertex will be fused into the biggest neighbor
			 * TODO : change name to Vertices
			 */			
			void removeVertexUnderSize(int size, bool preserveEdgeConnectic);
			
			bool asVerticesWithNoEdges();
			
			/**
			 * @brief watershed.
			 */
			void watershed(){watershed(_threshold);}
			
			/**
			 * @brief watershed. DO NOT USE DIRECTLY. Indeed, the parameter thrshold is also used by the ripple removal so it should be the same. One should use watershed()
			 * @param[in] threshold : fraction representing the fraction of the biggest value of cluster until a new cluster must created. If init node got value 100 and threshold = 1/10 then if the new node as 90 or less, it is not fused.
			 */
			void watershed(double threshold);	
			
			/*** Attempt to remove doors that were too big ot be detected by the ripple removal
			 */
			void removeDoors();
			
			double contactPointWithWalls(AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone v);
			
			bool lonelyVertices(){
				std::pair<AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexIteratorZone, AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexIteratorZone> vp;
				//vertices access all the vertix
				std::cout << "NEW start lonely" << std::endl;
		// 		std::cout << "num of vertices " << getNumVertices() << std::endl; 
				for (vp = boost::vertices(*this); vp.first != vp.second;) {
		// 			std::cout << "Looking up vertex " << std::endl;
					auto v = *vp.first;
					++vp.first;
					if(getNumEdges(v) == 0){
						return true;
					} 
				}
				std::cout << "Checked" << std::endl;
				return false;
			
			}
			
			/**
			 * @brief Remove vertices with no edges
			 */
			void removeLonelyVertices(){
				std::pair<AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexIteratorZone, AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexIteratorZone> vp;
				//vertices access all the vertix
				// std::cout << "NEW start remove lonely" << std::endl;
				// std::cout << "num of vertices " << getNumVertices() << std::endl; 
				int i = 0;
				for (vp = boost::vertices(*this); vp.first != vp.second;) {

					// std::cout << "num of vertices " << getNumVertices() << std::endl; 
					// std::cout << "Looking up vertex " << i <<std::endl;
					++i;
					auto v = *vp.first;
					++vp.first;
					if(getNumEdges(v) == 0){
						// std::cout << "Remove" << std::endl;
						removeVertex(v);
					}
				}
			}
			
			void useCvMat(bool should_we_use){
				auto vp = boost::vertices((*this));
				for(vp = boost::vertices((*this)) ; vp.first != vp.second;){
// 					std::cout <<"Updating Mat "<< std::endl;
					auto v = *vp.first;
					++vp.first;
					(*this)[v].useCvMat(should_we_use);
				}
			}
			
			void updateContours(){
// 				std::pair<VertexIteratorZone, VertexIteratorZone> vp;
				auto vp = boost::vertices((*this));
				for(vp = boost::vertices((*this)) ; vp.first != vp.second;){
// 					std::cout <<"Updating Contours "<< std::endl;
					auto v = *vp.first;
					++vp.first;
					try{
						(*this)[v].updateContour();
					}
					catch(std::exception& e){
						std::cout << "Here : " << e.what() << std::endl;
						cv::Mat graphmat2 = cv::Mat::zeros(600,600, CV_8U);
						(*this)[v].drawZone(graphmat2, cv::Scalar(100));
						(*this)[v].drawContour(graphmat2, cv::Scalar(100));
						cv::imshow("fused", graphmat2);
						cv::waitKey(0);	
						exit(0);
					}
				}
			}
			
			///@brief update all zones and element of the graph: remove Ripples, update countours
// 			void update(){
// // 				removeRiplesv2();
// 				updateContours();
// 			}
			

			void removeRiplesv2(int dist = -1);
			void removeRiplesv3(int dist = -1);
				
			void updateAllEdges(){
				auto vp = boost::vertices((*this));
				for(vp ; vp.first != vp.second; ++vp.first){
					auto v = *vp.first;
					EdgeIteratorZone out_i, out_end;
					//Since we fuse the old zone in biggest we only need to link them to biggest
					for (boost::tie(out_i, out_end) = boost::out_edges(v, (*this)); 
						out_i != out_end; out_i = ++out_i) {
						EdgeZone e_second = *out_i;
						VertexZone targ = boost::target(e_second, (*this));
// 						(*this)[e_second].setOldScore(v, targ, -2);
						(*this)[e_second].setMinimum(-2);
					}
				}
			}
			
			
			///@brief return source first and target second
// 			std::vector<ZoneCompared> compare(GraphZone& target);
			

			///Overwrite
			virtual void removeVertex(Vertex& v){
				EdgeIteratorZone out_i, out_end;	
				for (boost::tie(out_i, out_end) = boost::out_edges(v, (*this)); 
					out_i != out_end;) {
					EdgeZone e_second = *out_i;
					boost::remove_edge(e_second, (*this));
					boost::tie(out_i, out_end) = boost::out_edges(v, (*this));
				}
				boost::remove_vertex(v, (*this));
				
			}
			virtual void removeEdge(Edge& e){
				if(!(*this)[e].canRemove()){
					throw std::runtime_error("Can't remove unbreakable edge");
				}
				boost::remove_edge(e, (*this));
			}
			

			
		private:
			/**
			 * @brief recursively fuse node to be fuse to the top_vertex
			 * @param top_vertex : Vertex currently studied
			 * @param first_vertex : Vertex on the top of the food chain. The initial vertex of the watershed
			 * @param top_vertex_visited : The vertices already visited by the watershed algorithm
			 * @param top_vertex_visited_tmp : empty deque used by the algorithm to remember which node were visited in the recursion
			 * @param threshold : Threshold at which the difference between the value of top_vertex and a neighborhood vertex is considered enough for the neighborhood vertex not to be fused and to be considered a new zone. Actually no : fraction the new node must be partt of to not be fyused
			 */
			void getAllNodeRemovedWatershed(AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& top_vertex, AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& first_vertex, const std::deque< AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone >& top_vertex_visited, std::deque< AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone >& top_vertex_visited_tmp, double threshold, std::deque< AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone >& to_be_removed);

			/**
			* @brief get all ripples
			* base_vertex is the vertex to fuse in
			* top_vertex_visited is all the vertext visited since the beginning
			* dist is the maximum distance between the base and a ripple to be fused
			*/
			void getAllNodeRemovedRipples(AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& base_vertex, const std::deque< AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone >& top_vertex_visited, int dist = -1);
			void removeVertexWhilePreservingEdges(AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& v, bool createUnBreakableLinks);
			void removeVertexWhilePreservingEdges(AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& v, AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& v_to_fuse_in, bool createUnBreakableLinks);
			
			///@brief Return true of the zone is ripple
			bool isRipple(const AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& base_vertex, const AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& might_be_ripple) const;
			
			///@brief Check if the zone is a ripple of a neighborhood and removes it
			bool checkAndReplaceRipple(AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& might_be_ripple, AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& to_fuse_in);
			
			double variance (std::vector<double> input, double mean){
				double variance = 0 ;
				auto it = input.begin();
				for(it ; it != input.end() ; ++it){
					variance = variance + ( (*it - mean) * (*it - mean) );
// 					std::cout << "variance " << variance << std::endl;
				}
				
				return variance / (double) input.size();
			}
				
			double mean(std::vector<double> input){
				double sum = 0;
				auto it = input.begin();
				for(it ; it != input.end() ; ++it){
					sum = sum + (*it);
				}
				return sum / (double) input.size();
			}
				
			std::vector<double> standardization (std::vector<double> input, double mean, double sdeviation){
				std::vector<double> out;
				auto it = input.begin();
				for(it; it != input.end() ; ++it){
					double v_value = (*it - mean) / sdeviation;
					out.push_back(v_value);
				}
				return out;
			}
			
			
			void makeAllUnbreakableEdges(VertexZone& base_vertex);
			
			
		};
		


	}
}

template<typename VertexType, typename EdgeType>		
inline void AASS::maoris::GraphZoneInterface<VertexType, EdgeType>::drawPartial(cv::Mat& drawmat) const
{
	cv::Mat drawmat_old;
	drawmat.copyTo(drawmat_old);
	
	cv::Scalar color;
	cv::RNG rng(12345);
	std::pair<VertexIteratorZone, VertexIteratorZone> vp;
	//vertices access all the vertix
	for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
		std::cout << "VERTEX" << std::endl;
		if(drawmat.channels() == 1){
			color = rng.uniform(50, 255);
		}
		else if(drawmat.channels() == 3){
			color[0] = rng.uniform(50, 255);
			color[1] = rng.uniform(50, 255);
			color[2] = rng.uniform(50, 255);
		}
		
		VertexZone v = *vp.first;
		
		if((*this)[v].getZone().size() > 0){
// 				if(getNumEdges(v) > 1){
			
			cv::Mat copy = cv::Mat::zeros(drawmat_old.rows, drawmat_old.cols, CV_8U);
			for(size_t j = 0 ; j < (*this)[v].getZone().size() ; ++j){
				copy.at<uchar>((*this)[v].getZone()[j].x, (*this)[v].getZone()[j].y) = (*this)[v].getValue();
			}
			draw(drawmat, v, color);
			draw(copy, v, color);
			
			EdgeIteratorZone out_i, out_end;
			EdgeZone e;
			
			for (boost::tie(out_i, out_end) = boost::out_edges(v, (*this)); 
				out_i != out_end; ++out_i) {
				e = *out_i;
				VertexZone src = boost::source(e, (*this)), targ = boost::target(e, (*this));
				if( (*this)[targ].getZone().size() > 100 ){
					cv::line(drawmat, (*this)[src].getCentroid(), (*this)[targ].getCentroid(), color);
					cv::line(copy, (*this)[src].getCentroid(), (*this)[targ].getCentroid(), color);
					for(size_t j = 0 ; j < (*this)[targ].getZone().size() ; ++j){
						copy.at<uchar>((*this)[targ].getZone()[j].x, (*this)[targ].getZone()[j].y) = (*this)[targ].getValue();
					}
				}
			}
			
			cv::imshow("Z", (*this)[v].getZoneMat());
			cv::imshow("Partial" , copy);
			cv::waitKey(0);
		}
		
	}

}

template<typename VertexType, typename EdgeType>
inline void AASS::maoris::GraphZoneInterface<VertexType, EdgeType>::drawEvaluation(cv::Mat& drawmat) const
{
	cv::Mat drawmat_old;
	drawmat.convertTo(drawmat_old, CV_8U);
	
	cv::Scalar color;
	int count = 1;
	std::pair<VertexIteratorZone, VertexIteratorZone> vp;
	//vertices access all the vertix
	for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
		
		if(drawmat.channels() == 1){
			color = count;
		}
		else if(drawmat.channels() == 3){
			color[0] = count;
			color[1] = count;
			color[2] = count;
		}
		count++;
		
		VertexZone v = *vp.first;
		
// 		if((*this)[v].getZone().size() > 100){
// 				if(getNumEdges(v) > 1){
			
		drawEvaluation(drawmat, v, color);
	}
}

template<typename VertexType, typename EdgeType>
inline void AASS::maoris::GraphZoneInterface<VertexType, EdgeType>::drawSimple(cv::Mat& drawmat) const
{
	cv::Mat drawmat_old;
	drawmat.convertTo(drawmat_old, CV_8U);
	
	cv::Scalar color;
	cv::RNG rng(12345);
	int nb_zones = this->getNumVertices();
	int color_step = 249 / nb_zones;
	int count = 1;
	std::pair<VertexIteratorZone, VertexIteratorZone> vp;
	//vertices access all the vertix
	for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
		
		if(drawmat.channels() == 1){
			color = color_step * count;
		}
		else if(drawmat.channels() == 3){
			color[0] = color_step * count;
			color[1] = color_step * count;
			color[2] = color_step * count;
		}
		count++;
		
		VertexZone v = *vp.first;
		
// 		if((*this)[v].getZone().size() > 100){
// 				if(getNumEdges(v) > 1){
			
		drawSimple(drawmat, v, color);
		
	}
		

}

template<typename VertexType, typename EdgeType>
inline void AASS::maoris::GraphZoneInterface<VertexType, EdgeType>::draw(cv::Mat& drawmat) const
{
	
	cv::Mat drawmat_old;
	drawmat.convertTo(drawmat_old, CV_8U);
	
	cv::Scalar color;
	cv::RNG rng(12345);
	int nb_zones = this->getNumVertices();
	int color_step = 249 / nb_zones;
	int count = 1;
	std::pair<VertexIteratorZone, VertexIteratorZone> vp;
	//vertices access all the vertix
	for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
		
		if(drawmat.channels() == 1){
			color = color_step * count;
		}
		else if(drawmat.channels() == 3){
			color[0] = color_step * count;
			color[1] = color_step * count;
			color[2] = color_step * count;
		}
		count++;
		
		VertexZone v = *vp.first;
		
// 		if((*this)[v].getZone().size() > 100){
// 				if(getNumEdges(v) > 1){
			
		draw(drawmat, v, color);
		
		EdgeIteratorZone out_i, out_end;
		EdgeZone e;
		
		for (boost::tie(out_i, out_end) = boost::out_edges(v, (*this)); 
			out_i != out_end; ++out_i) {
			e = *out_i;
			VertexZone src = boost::source(e, (*this)), targ = boost::target(e, (*this));
			if( (*this)[targ].getZone().size() > 100 ){
// 					cv::line(drawmat, (*this)[src].getCentroid(), (*this)[targ].getCentroid(), cv::Scalar(255));
			}
			if( (*this)[e].canRemove() == false ){
				cv::line(drawmat, (*this)[src].getCentroid(), (*this)[targ].getCentroid(), cv::Scalar(255), 2);
			}
			else{
				cv::line(drawmat, (*this)[src].getCentroid(), (*this)[targ].getCentroid(), cv::Scalar(150));
			}
		}
	}
		
	for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {

		VertexZone v = *vp.first;
		EdgeIteratorZone out_i, out_end;
		EdgeZone e;
		(*this)[v].printLabel(drawmat);
		
	}

}

template<typename VertexType, typename EdgeType>
inline void AASS::maoris::GraphZoneInterface<VertexType, EdgeType>::drawEvaluation(cv::Mat& m, const VertexZone& v, const cv::Scalar& color) const
{
// 		std::cout << std::endl;
// 			for(size_t i = 0 ; i < (*this)[v].landmarks.size() ; i++){
// 				cv::Point2i point;
// 				point.x = (*this)[v].landmarks[i].first.getX();
// 				point.y = (*this)[v].landmarks[i].first.getY();
// 				cv::circle(m, point, 10, color, 3);
// 			}
// 			cv::drawContours( m, std::vector<std::vector<cv::Point> >(1,(*this)[v].contour), -1, color, 2, 8);
	(*this)[v].drawZone(m, color);
// 	std::cout << "VALUIE " << std::endl;
// 	(*this)[v].printPCA();
	
// 	(*this)[v].printLabel(m);
// 	cv::circle(m, (*this)[v].getCentroid(), 2, 255, -1);
	
// 			cv::Mat draw_tmp = cv::Mat::zeros(m.rows, m.cols, CV_8U);
// 			for(size_t j = 0 ; j < (*this)[v].getZone().size() ; ++j){
// 				draw_tmp.at<uchar>((*this)[v].getZone()[j].x, (*this)[v].getZone()[j].y) = 150;
// 			}
// 			
// 			cv::circle(draw_tmp, (*this)[v].getCentroid(), 10, 255, -1);
// 			
// 			std::cout << "CENTROID " << (*this)[v].getCentroid() << "image size "<<  m.size() << std::endl;
// 			
// 			cv::imshow("Zones " , draw_tmp);
// 			cv::waitKey(0);
		
}

template<typename VertexType, typename EdgeType>
inline void AASS::maoris::GraphZoneInterface<VertexType, EdgeType>::drawContours(cv::Mat& m,  const VertexZone& v, const cv::Scalar& color) const
{
	(*this)[v].drawContour(m, color);
}

template<typename VertexType, typename EdgeType>
inline void AASS::maoris::GraphZoneInterface<VertexType, EdgeType>::drawContours(cv::Mat& drawmat) const
{
		cv::Mat drawmat_old;
	drawmat.convertTo(drawmat_old, CV_8U);
	
	cv::Scalar color(255);
	cv::RNG rng(12345);
	int nb_zones = this->getNumVertices();
	std::pair<VertexIteratorZone, VertexIteratorZone> vp;
	//vertices access all the vertix
	for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
		
		
		
		VertexZone v = *vp.first;
		
// 		if((*this)[v].getZone().size() > 100){
// 				if(getNumEdges(v) > 1){
			
		drawContours(drawmat, v, color);
	}

}

template<typename VertexType, typename EdgeType>
inline void AASS::maoris::GraphZoneInterface<VertexType, EdgeType>::drawSimple(cv::Mat& m, const VertexZone& v, const cv::Scalar& color) const
{
	(*this)[v].drawZone(m, color);
	(*this)[v].drawContour(m, color);
}

template<typename VertexType, typename EdgeType>
inline void AASS::maoris::GraphZoneInterface<VertexType, EdgeType>::draw(cv::Mat& m, const VertexZone& v, const cv::Scalar& color) const
{
	(*this)[v].drawZone(m, color);
	(*this)[v].drawContour(m, color);	
}

#endif
