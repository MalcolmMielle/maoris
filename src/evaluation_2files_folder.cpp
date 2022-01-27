#include <iostream>
// #define BOOST_TEST_DYN_LINK

// #define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
// #include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <string>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/filesystem/path.hpp>

#include "ZoneExtractor.hpp"
#include "FuzzyOpening.hpp"
#include "ZoneReducer.hpp"
#include "Segmentor.hpp"
#include "Evaluation.hpp"

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





// void makeGraph(cv::Mat& slam, AASS::maoris::GraphZone& graph_slam, double& time){
// 		
// 	double begin_process, end_process, decompose_time;
// 	std::cout << "/************ FUZZY OPENING*************/ \n";
// 	AASS::maoris::FuzzyOpening fuzzy_slam;
// 	fuzzy_slam.fast(false);
// 	
// 	cv::Mat out_slam;
// 	
// 	begin_process = getTime();	
// 	fuzzy_slam.fuzzyOpening(slam, out_slam, 500);
// 	end_process = getTime();	decompose_time = end_process - begin_process;
// 	time = decompose_time;
// 	
// 	std::cout << "Fuzzy opening time: " << time << std::endl;
// 	
// 	out_slam.convertTo(out_slam, CV_8U);
// 		
// 	std::cout << "/************ REDUCING THE SPACE OF VALUES *****************/\n";
// 	cv::Mat out_tmp_slam;
// 	AASS::maoris::ZoneExtractor zone_maker;
// 	zone_maker.addValueToIgnore(0);
// 	
// 	begin_process = getTime();
// 	AASS::maoris::reduceZone(out_slam, out_tmp_slam, 5);
// 	zone_maker.extract(out_tmp_slam);
// 	end_process = getTime();	decompose_time = end_process - begin_process;
// 	time = time + decompose_time;
// 	
// 	std::cout << "Zone reducing: " << decompose_time << std::endl;
// 		
// 	std::cout << "/*********** MAKING AND TRIMMING THE GRAPH ***************/\n";
// 	
// 	int size_to_remove2 = 10;
// 	
// 	begin_process = getTime();
// 	graph_slam = zone_maker.getGraph();
// 	graph_slam.setThreshold(0.25);
// // 	graph_slam.removeVertexValue(0);	
// 	graph_slam.removeVertexUnderSize(size_to_remove2, true);
// 
// 	graph_slam.useCvMat(true);
// // 	graph_slam.updatePCA();
// 	graph_slam.updateContours();
// 	graph_slam.removeRiplesv3();
// 	
// 	end_process = getTime();	decompose_time = end_process - begin_process;
// 	time = time + decompose_time;
// 	std::cout << "Ripples: " << decompose_time << std::endl;
// 	
// 	begin_process = getTime();
// // 	graph_slam.updatePCA();
// 	graph_slam.updateContours();
// 	
// 	//Watershed Algorithm
// 	graph_slam.watershed();
// 	
// 	int size_to_remove = 100;
// 	graph_slam.removeVertexUnderSize(size_to_remove, true);
// 	graph_slam.removeLonelyVertices();
// 	end_process = getTime();	decompose_time = end_process - begin_process;
// 	time = time + decompose_time;
// 	
// 	std::cout << "watershed: " << decompose_time << std::endl;
// 	
// 	if(graph_slam.lonelyVertices())
// 		throw std::runtime_error("Fuck you lonelyness");	
// 	
// }



void process(const std::string& file, const std::string& full_path_GT, AASS::maoris::Evaluation& eval){

// 	AASS::maoris::GraphZone graph_slam;
	
	cv::Mat slam_in = cv::imread(file, cv::ImreadModes::IMREAD_GRAYSCALE);
	assert(cv::ImreadModes::IMREAD_GRAYSCALE == 0);
	
// 	cv::Mat graphmat = AASS::maoris::segment_Ground_Truth(slam_in);

	cv::Mat graphmat = slam_in;
	
    cv::Mat img_hist_equalized;
	cv::equalizeHist(graphmat, img_hist_equalized);
	cv::imshow("TEST", img_hist_equalized);
	
	//Reading the GT
	
	cv::Mat image_GT = cv::imread(full_path_GT,0);
	
	cv::Mat GT_segmentation = AASS::maoris::segment_Ground_Truth(image_GT);
	
	cv::Mat img_hist_equalizedgt;
	cv::equalizeHist(GT_segmentation, img_hist_equalizedgt);
	cv::imshow("GT", img_hist_equalizedgt);
// 	cv::waitKey(0);
	
	assert(graphmat.rows == GT_segmentation.rows);
	assert(graphmat.cols == GT_segmentation.cols);
				
	double pixel_precision, pixel_recall;
	std::vector < std::vector<float> > Precisions, Recalls;
	std::vector<double> Times;
		
	eval.compare(graphmat, GT_segmentation, 0, file);
}


int main(int argc, char** argv){
	
// 	int argc = boost::unit_test::framework::master_test_suite().argc;
// 	char** argv = boost::unit_test::framework::master_test_suite().argv;
		
	
	AASS::maoris::Evaluation eval;
	
	std::string path_file = argv[1];
	std::string path_gt = argv[2];
// 	std::string file = "../../Test/Thermal/cold.jpg";
	
	boost::filesystem::path p(path_file);
	boost::filesystem::path p_gt(path_gt);
	try{
		if(! boost::filesystem::exists(p) || ! boost::filesystem::exists(p_gt) ){
			std::cout << "need a valid path toward the images" << std::endl;
			return 0;
		}
		if(! boost::filesystem::is_directory(p) || ! boost::filesystem::is_directory(p_gt) ){
			std::cout << "need a valid path folder toward the images" << std::endl;
			return 0;
		}
		
		if(boost::filesystem::is_directory(p)){
			
			
			
			std::vector<boost::filesystem::path> v;
			//Get all files and sort them
			std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), std::back_inserter(v));
			std::sort(v.begin(), v.end());
			
			
			
// 			int i = 0;
			for (std::vector<boost::filesystem::path>::const_iterator it (v.begin()); it != v.end(); it = ++it)
			{
				boost::filesystem::path fn = *it;
				
				std::string name = fn.filename().string();
				std::string model = path_gt + name;
				
				std::cout << "Process " << fn.string() << " with model " << model << std::endl;
				
				process(fn.string(), model, eval);
// 				eval.calculate();
				
				std::cout << "SIZE " << eval.size() << std::endl;
// 				if(i == 3){
// 					return 0;
// 				}
// 				++i;
			}
		}
	}
	catch (const boost::filesystem::filesystem_error& ex)
	{
		std::cout << ex.what() << '\n';
	}
	
	//add precision mean and recal + nb of file
	std::string result_file = "maoris_all_measures_2gtfolder.dat";
	std::cout << "SIZE " << eval.size() << std::endl;
	eval.exportAll(result_file);

}
