#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include <boost/filesystem.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/filesystem/path.hpp>

#include "ZoneExtractor.hpp"
#include "FuzzyOpening.hpp"
#include "Kmean.hpp"
#include "ZoneReducer.hpp"
#include "Segmentor.hpp"
#include "Evaluation.hpp"



inline double getTime() //in millisecond
{
    //assuming unix-type systems
    //timezone tz;
    timeval  tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec*1000000+tv.tv_usec)*1.0/1000;
}



void draw(AASS::maoris::GraphZone& gp_real, AASS::maoris::GraphZone& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, std::vector< std::pair<AASS::maoris::GraphZone::Vertex, AASS::maoris::GraphZone::Vertex> > matches){
	
	cv::Mat obst_copy;
	obstacle.copyTo(obst_copy);
	
	cv::imshow("OBSOBS", obst_copy);
	
	cv::Mat obst_model_copy;
	obstacle_model.copyTo(obst_model_copy);
	
// 	cv::Mat draw_links = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
// 	cv::Mat draw_graph = cv::Mat::zeros(obst_copy.size(), CV_8UC3);
// 	cv::Mat draw_graph_model = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
	
	int cols_max = obst_model_copy.size().width;
	if(cols_max < obst_copy.size().width){
		cols_max = obst_copy.size().width;
	}
	
	cv::Size size(cols_max, obst_model_copy.size().height + obst_copy.size().height);
	cv::Mat all = cv::Mat::zeros(size, CV_8UC1);
// 	cv::Mat only_linked = cv::Mat::zeros(size, CV_8UC3);
	cv::Mat all_maps = cv::Mat::zeros(size, CV_8UC3);
	
	cv::Mat roi = all(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
// 	cv::Mat roi_linked = only_linked(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
	cv::Mat roi_model = all(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
	
// 	cv::Mat roi_maps = all_maps(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
// 	cv::Mat roi_model_maps = all_maps(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
	
// 	gp_real.draw(roi);
// 	gp_model.draw(roi_model);
	
	obst_copy.copyTo(roi);
	obst_model_copy.copyTo(roi_model);
	
	cv::Scalar color;
	cv::RNG rrng(12345);
		
	if(all.channels() == 1){
		color = rrng.uniform(50, 255);
	}
	
	else if(all.channels() == 3){
		color[1] = rrng.uniform(50, 255);
		color[3] = rrng.uniform(50, 255);
		color[2] = rrng.uniform(50, 255);
	}
// 	
// 	cv::Scalar color_model;
// 
// 	
	auto it = matches.begin();
	
	for( ; it != matches.end() ; ++it){
		std::cout << "DRAW LINE " << std::endl;
		
		auto point = gp_model[it->second].getCentroid();
		point.y = point.y + obst_copy.size().height;
		
		cv::line(all, gp_real[it->first].getCentroid(), point, color, 5);
	}
	
	cv::imshow("all links", all);
	
}



void makeGraph(cv::Mat& slam, AASS::maoris::GraphZone& graph_slam, double& time){
		
	double begin_process, end_process, decompose_time;
	std::cout << "/************ FUZZY OPENING*************/ \n";
	AASS::maoris::FuzzyOpening fuzzy_slam;
	fuzzy_slam.fast(false);
	
	cv::Mat out_slam;
	
	begin_process = getTime();	
	fuzzy_slam.fuzzyOpening(slam, out_slam, 500);
	end_process = getTime();	decompose_time = end_process - begin_process;
	time = decompose_time;
	
	std::cout << "Fuzzy opening time: " << time << std::endl;
	
	out_slam.convertTo(out_slam, CV_8U);
		
	std::cout << "/************ REDUCING THE SPACE OF VALUES *****************/\n";
	cv::Mat out_tmp_slam;
	AASS::maoris::ZoneExtractor zone_maker;
	zone_maker.addValueToIgnore(0);
	
	begin_process = getTime();
	AASS::maoris::reduceZone(out_slam, out_tmp_slam, 5);
	zone_maker.extract(out_tmp_slam);
	end_process = getTime();	decompose_time = end_process - begin_process;
	time = time + decompose_time;
	
	std::cout << "Zone reducing: " << decompose_time << std::endl;
		
	std::cout << "/*********** MAKING AND TRIMMING THE GRAPH ***************/\n";
	
	int size_to_remove2 = 10;
	
	begin_process = getTime();
	graph_slam = zone_maker.getGraph();
	graph_slam.setThreshold(0.25);
// 	graph_slam.removeVertexValue(0);	
	graph_slam.removeVertexUnderSize(size_to_remove2, true);

	graph_slam.useCvMat(true);
// 	graph_slam.updatePCA();
	graph_slam.updateContours();
	graph_slam.removeRiplesv3();
	
	end_process = getTime();	decompose_time = end_process - begin_process;
	time = time + decompose_time;
	std::cout << "Ripples: " << decompose_time << std::endl;
	
	begin_process = getTime();
// 	graph_slam.updatePCA();
	graph_slam.updateContours();
	
	//Watershed Algorithm
	graph_slam.watershed();
	
	int size_to_remove = 100;
	graph_slam.removeVertexUnderSize(size_to_remove, true);
	graph_slam.removeLonelyVertices();
	end_process = getTime();	decompose_time = end_process - begin_process;
	time = time + decompose_time;
	
	std::cout << "watershed: " << decompose_time << std::endl;
	
	if(graph_slam.lonelyVertices())
		throw std::runtime_error("Fuck you lonelyness");	
	
}


BOOST_AUTO_TEST_CASE(trying)
{
	
	int argc = boost::unit_test::framework::master_test_suite().argc;
	char** argv = boost::unit_test::framework::master_test_suite().argv;
		
	std::string file = argv[1];
	std::string full_path_GT = argv[2];
// 	std::string file = "../../Test/Thermal/cold.jpg";
	AASS::maoris::GraphZone graph_slam;
	
	cv::Mat slam_in = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
	assert(CV_LOAD_IMAGE_GRAYSCALE == 0);
	cv::Mat slam = slam_in > 250;
	
	
	
// 	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY);
// 	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY_INV);
// 	cv::imshow("map in", slam);
// 	cv::waitKey(0);
	
	AASS::maoris::Segmentor segmenteur;
	
	double time = 0;
// 	makeGraph(slam, graph_slam, time);
	
	time = segmenteur.segmentImage(slam, graph_slam);
	
	cv::Mat segmented_map = segmenteur.getSegmentedMap();
	
	cv::imshow("straight segmented", segmented_map);
	
	std::cout << "Total time: " << time << std::endl;
			
	/********** PCA of all zones in Graph and removing the ripples **********/
	graph_slam.updateContours();

	
	cv::Mat slam1 = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
   cv::Mat graphmat = cv::Mat::zeros(slam1.size(), CV_8U);
   graph_slam.drawEvaluation(graphmat);
	
// 	cv::Mat partial = cv::Mat::zeros(slam1.size(), CV_8U);
//     graph_slam.drawPartial(partial);
	
	cv::Mat img_hist_equalized;
	cv::equalizeHist(graphmat, img_hist_equalized);
// 	cv::resize(graphmat, graphmat, cv::Size(graphmat.cols * 2, graphmat.rows * 2));
	cv::imshow("GRAPH", img_hist_equalized);
	
// 	cv::Mat minus = graphmat - slam;
// 	cv::Mat minus2 = slam - graphmat;
// 	cv::imshow("black", minus);
// 	cv::imshow("alos black", minus2);
// 	cv::waitKey(0);
	
	cv::Mat graphmat_vis = cv::Mat::zeros(slam1.size(), CV_8U);
	graph_slam.draw(graphmat_vis);
// 	cv::resize(graphmat, graphmat, cv::Size(graphmat.cols * 2, graphmat.rows * 2));
	cv::imshow("GRAPH Visible", graphmat_vis);
	
	cv::Mat graphmat_c = cv::Mat::zeros(slam1.size(), CV_8U);
	graph_slam.drawContours(graphmat_c);
	cv::imshow("contours", graphmat_c);
	
	std::cout << "Size of graph" << graph_slam.getNumVertices() << std::endl;
   
	
	//Reading the GT
	
	cv::Mat image_GT = cv::imread(full_path_GT,0);
// 	cv::imshow("GT raw", image_GT);
	cv::waitKey(0);
	
	cv::Mat GT_segmentation = AASS::maoris::segment_Ground_Truth(image_GT);
	cv::Mat graphmat_straight = AASS::maoris::segment_Ground_Truth(segmented_map);
// 	cv::Mat graphmat = segment_Ground_Truth(slam);
	
	cv::Mat img_hist_equalizedgt;
	cv::equalizeHist(GT_segmentation, img_hist_equalizedgt);
	cv::imshow("GT", img_hist_equalizedgt);
	
	cv::Mat img_hist_equalizedseg;
	cv::equalizeHist(graphmat_straight, img_hist_equalizedseg);
	cv::imshow("seg eq", img_hist_equalizedseg);
	
	cv::waitKey(0);
	
// 	compareAndExport(graphmat, GT_segmentation, time, file);
	AASS::maoris::Evaluation eval;
	eval.compare(graphmat_straight, GT_segmentation, time, file);
	eval.exportAll("test.dat");
	
	std::vector<int> param;
	param.push_back(CV_IMWRITE_PNG_COMPRESSION);
	param.push_back(9);
	boost::filesystem::path p(file);
	std::string name = p.filename().stem().string();
	cv::imwrite(name + "_straighten.png", graphmat_straight, param);
	cv::imwrite(name + ".png", graphmat, param);
	
	cv::imshow(name + "_straighten.png", graphmat_straight);
	cv::imshow(name + ".png", graphmat);
	
	cv::waitKey(0);
	
	
}
