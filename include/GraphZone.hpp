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
		};
		
		

		//TODO convert Mat to Eigen !
		// ATTENTION: if uniqueness is calculated and then a node is added, then the flag to check if the uniqueness was updated is wrong... Need to do it either every time a node is added but it will need to move the addVertex function here.

		class GraphZone : public bettergraph::SimpleGraph<Zone, EdgeElement>{
			
		private:
			//TEST
			int _iteration;
			
		protected:
			
			
			double _margin_factor;
		
			 ///@param[in] threshold : fraction representing the fraction of the biggest value of cluster until a new cluster must created. If init node got value 100 and threshold = 1/10 then if the new node as 90 or less, it is not fused.
			double _threshold;
			
		public:
			typedef typename bettergraph::SimpleGraph<Zone, EdgeElement>::GraphType GraphZoneType;
			typedef typename bettergraph::SimpleGraph<Zone, EdgeElement>::Vertex VertexZone;
			typedef typename bettergraph::SimpleGraph<Zone, EdgeElement>::Edge EdgeZone;
			typedef typename bettergraph::SimpleGraph<Zone, EdgeElement>::VertexIterator VertexIteratorZone;
			typedef typename bettergraph::SimpleGraph<Zone, EdgeElement>::EdgeIterator EdgeIteratorZone;

			GraphZone(): _margin_factor(0.10), _threshold(0.25){};
			
			
			void setThreshold(double t){if(t >= 0 && t<= 1){_threshold = t;}else{throw std::runtime_error("Threhsold needs to be between 0 and 1");}}
			void setMargin(double m){if(m >= 0 && m<= 1){_margin_factor = m;}else{throw std::runtime_error("Margin needs to be between 0 and 1");}}
			
			double getT(){return _threshold;}
			
			///@brief draw a simple image with no annotations.
			void drawSimple(cv::Mat& drawmat) const;
			void drawSimple(cv::Mat& m, const bettergraph::SimpleGraph<Zone, int>::Vertex& v, const cv::Scalar& color) const;
			///@brief draw map with all info on image
			void draw(cv::Mat& drawmat) const;
			void draw(cv::Mat& m, const bettergraph::SimpleGraph<Zone, int>::Vertex& v, const cv::Scalar& color) const;
			///@brief draw the zone one by one.
			void drawPartial(cv::Mat& drawmat) const;
			///@brief drawing function to compare the maps to the dataset of Bromman.
			void drawEvaluation(cv::Mat& drawmat) const;
			void drawEvaluation(cv::Mat& m, const bettergraph::SimpleGraph<Zone, int>::Vertex& v, const cv::Scalar& color) const;
			
			void drawContours(cv::Mat& drawmat) const;
			void drawContours(cv::Mat& m, const bettergraph::SimpleGraph<Zone, int>::Vertex& v, const cv::Scalar& color) const;
			
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
			
			double contactPointWithWalls(AASS::maoris::GraphZone::VertexZone v);
			
			bool lonelyVertices(){
				std::pair<AASS::maoris::GraphZone::VertexIteratorZone, AASS::maoris::GraphZone::VertexIteratorZone> vp;
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
				std::pair<AASS::maoris::GraphZone::VertexIteratorZone, AASS::maoris::GraphZone::VertexIteratorZone> vp;
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
			void getAllNodeRemovedWatershed(AASS::maoris::GraphZone::VertexZone& top_vertex, AASS::maoris::GraphZone::VertexZone& first_vertex, const std::deque< AASS::maoris::GraphZone::VertexZone >& top_vertex_visited, std::deque< AASS::maoris::GraphZone::VertexZone >& top_vertex_visited_tmp, double threshold, std::deque< AASS::maoris::GraphZone::VertexZone >& to_be_removed);

			/**
			* @brief get all ripples
			* base_vertex is the vertex to fuse in
			* top_vertex_visited is all the vertext visited since the beginning
			* dist is the maximum distance between the base and a ripple to be fused
			*/
			void getAllNodeRemovedRipples(AASS::maoris::GraphZone::VertexZone& base_vertex, const std::deque< AASS::maoris::GraphZone::VertexZone >& top_vertex_visited, int dist = -1);
			void removeVertexWhilePreservingEdges(AASS::maoris::GraphZone::VertexZone& v, bool createUnBreakableLinks);
			void removeVertexWhilePreservingEdges(AASS::maoris::GraphZone::VertexZone& v, AASS::maoris::GraphZone::VertexZone& v_to_fuse_in, bool createUnBreakableLinks);
			
			///@brief Return true of the zone is ripple
			bool isRipple(const AASS::maoris::GraphZone::VertexZone& base_vertex, const AASS::maoris::GraphZone::VertexZone& might_be_ripple) const;
			
			///@brief Check if the zone is a ripple of a neighborhood and removes it
			bool checkAndReplaceRipple(AASS::maoris::GraphZone::VertexZone& might_be_ripple, AASS::maoris::GraphZone::VertexZone& to_fuse_in);
			
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

#endif
