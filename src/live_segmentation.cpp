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
#include <tuple>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/filesystem/path.hpp>

#include "ZoneExtractor.hpp"
#include "FuzzyOpening.hpp"
#include "ZoneReducer.hpp"
#include "Segmentor.hpp"
#include "EvaluationParam.hpp"
#include <chrono>
#include <ctime>

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


cv::Mat threshold_maps(cv::Mat& slam_in_color){

	cv::Mat slam;
	bool not_good = true;
	int step = 100;

	while(not_good == true) {

//		cv::Mat slam_in;
//		cv::cvtColor(slam_in_color, slam_in, cv::COLOR_BGR2GRAY);
//	cv::imshow("map in", slam_in);
//	cv::waitKey(0);
//		slam = slam_in > step;


		cv::inRange(slam_in_color, cv::Scalar(step, 0, 0), cv::Scalar(255, 255, 255), slam);

		cv::imshow("input", slam);
//		cv::waitKey(0);

//		cv::cvtColor(slam_tmp, slam, cv::COLOR_BGR2GRAY);

		cv::Point seed(0, 0);
//		int i = 0;
//		int j = 0;
//
//		while (seed.x == -1 && i < slam.rows) {
//			if (slam.at<uchar>(i, j) == 0) {
//				seed.x = i;
//				seed.y = j;
//			}
//			j++;
//			if (j == slam.cols) {
//				j = 0;
//				i++;
//			}
//		}

//		cv::Mat mask;
//		cv::Canny(slam, mask, 100, 200);
//		cv::copyMakeBorder(mask, mask, 1, 1, 1, 1, cv::BORDER_REPLICATE);
//Fill mask with value 128
		cv::floodFill(slam, seed, cv::Scalar(255) ,0, cv::Scalar(), cv::Scalar());
		cv::floodFill(slam, seed, cv::Scalar(0) ,0, cv::Scalar(), cv::Scalar());

		cv::imshow("Threhsold", slam);
		auto output = cv::waitKey(0);
		std::cout << "OUTPUT " << output << "and step " << step  << " and seed " << seed.x << " " << seed.y << std::endl;

		if(output == 82){
			step++;
		}
		else if(output == 84){
			step--;
		}
		else {
			not_good = false;
		}
	}
	return slam;
}



void process(cv::Mat& slam_in, const cv::Mat& gt, double t, double margin, double ripples, double doors){

	AASS::maoris::GraphZone graph_slam;
	graph_slam.setThreshold(t);
	graph_slam.setMargin(margin);
	graph_slam.setThresholdFusionRipples(ripples);
	graph_slam.setThresholdFusionDoors(doors);
	
//	cv::Mat slam_in = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
//	assert(CV_LOAD_IMAGE_GRAYSCALE == 0);

//	cv::Mat slam_in;
//	cv::cvtColor(slam_in_color, slam_in, cv::COLOR_BGR2GRAY);
////	cv::imshow("map in", slam_in);
////	cv::waitKey(0);
//	cv::Mat slam = slam_in > 100;
//
//	cv::Mat gt;
//	cv::cvtColor(gt_in_color, gt, cv::COLOR_BGR2GRAY);
////	cv::waitKey(0);
//	cv::Mat gt = gt_in > 100;
//	cv::imshow("gt in", gt);
	
// 	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY);
// 	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY_INV);
 	cv::imshow("map in", slam_in);
 	cv::waitKey(0);
//	cv::resize(slam, slam, cv::Size(slam.cols / 2, slam.rows / 2));
//	std::cout << "Input parameters " << t << std::endl;
//	std::cout << "T" << graph_slam.getT() << std::endl;
	double time = 0;
// 	makeGraph(slam, graph_slam, time);
	AASS::maoris::Segmentor segmenteur;
	time = segmenteur.segmentImage(slam_in, graph_slam);

    std::cout << "NB OF ZONES " << graph_slam.getNumVertices() << std::endl;

	cv::Mat segmented_map = segmenteur.getSegmentedMap();
	
	std::cout << "Total time: " << time << std::endl;
			
	/********** PCA of all zones in Graph and removing the ripples **********/
// 	graph_slam.update();

	
//	cv::Mat slam1 = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat graphmat = cv::Mat::zeros(slam_in.size(), CV_8U);
    graph_slam.drawEvaluation(graphmat);

// 	cv::Mat partial = cv::Mat::zeros(slam1.size(), CV_8U);
//     graph_slam.drawPartial(partial);
	
 	cv::Mat img_hist_equalized;
 	cv::equalizeHist(graphmat, img_hist_equalized);
 // 	cv::resize(graphmat, graphmat, cv::Size(graphmat.cols * 2, graphmat.rows * 2));
 	cv::imshow("segmentation", img_hist_equalized);
// 	
//     cv::Mat graphmat_vis = cv::Mat::zeros(slam1.size(), CV_8U);
//     graph_slam.draw(graphmat_vis);
// 	cv::resize(graphmat, graphmat, cv::Size(graphmat.cols * 2, graphmat.rows * 2));
// 	cv::imshow("GRAPH Visible", graphmat_vis);
	
// 	std::cout << "Size of graph" << graph_slam.getNumVertices() << std::endl;
    
	
	//Reading the GT
	
// 	std::cout << " GT " << full_path_GT << std::endl;
// 	
//	cv::Mat image_GT = cv::imread(gt_path,0);
// 	cv::imshow("GT raw", image_GT);

	
//	cv::Mat GT_segmentation = AASS::maoris::segment_Ground_Truth(image_GT);
//    / THIS ONE IS USE IF YOU NEED STRAIGHTEN MAPS
	cv::Mat graphmat_straight = AASS::maoris::segment_Ground_Truth(segmented_map);
	
// 	cv::Mat img_hist_equalizedgt;
// 	cv::equalizeHist(GT_segmentation, img_hist_equalizedgt);
 	cv::imshow("segmentation straight", segmented_map);
 	cv::waitKey(0);

//    SKETCHES
	AASS::maoris::Evaluation eval;
    eval.compare(graphmat, gt, time, "camera_input");
	eval.calculate();
//    ROBOT MAPS
//	AASS::maoris::Evaluation eval;
	AASS::maoris::Evaluation evals;
    evals.compare(graphmat_straight, gt, time, "camera_inpu_s");
	evals.calculate();
//	std::cout << "SIZE " << eval.size() << std::endl;

	std::cout << "Evaluation gave a score of " << eval.getMatthewsCCMedianPerZone() << " for the not straight segmentation, and " << evals.getMatthewsCCMedianPerZone() << " for the straight segmentation" << std::endl;

}


std::tuple<std::string, std::vector<std::string> > getMapsAndGT(){

    std::string in = "o";
    std::cout << "Please input a source folder for the maps to segment" << std::endl;
    std::getline(std::cin, in);

    if(in.back() != '/') {
        in = in + "/";
    }

    boost::filesystem::path p(in);
    while(!boost::filesystem::exists(p) || !boost::filesystem::is_directory(p)) {
        std::cout << "need a valid path toward the images - please add new input" << std::endl;
        std::getline(std::cin, in);

        if(in.back() != '/') {
            in = in + "/";
        }

        boost::filesystem::path ptmp(in);
        p = ptmp;
    }

    std::string gt = "o";
    std::vector<std::string> all_gt;
    std::cout << "Please input as many gt folder as you want. Input nothing if done." << std::endl;
    while ( !gt.empty() ) {
        std::getline(std::cin, gt);

        std::cout << "Got : " << gt << std::endl;
        boost::filesystem::path p(gt);
        if(!boost::filesystem::exists(p) || !boost::filesystem::is_directory(p)) {
            std::cout << "need a valid path toward the images - ignoring input" << std::endl;
        }
        else{
            if(gt.back() != '/') {
                gt = gt + "/";
            }
            all_gt.push_back(gt);
        }
    }
    std::cout << "Done, lets' do it" << std::endl;
    return std::make_tuple(in, all_gt);

}




int main(int argc, char** argv) {

// 	int argc = boost::unit_test::framework::master_test_suite().argc;
// 	char** argv = boost::unit_test::framework::master_test_suite().argv;

    cv::VideoCapture cap;
    //Choose param
    double threshold = 0.3;
    double margin = 0.1;
    double ripples = 40;
    double doors = 60;

	std::string gt_path = "";
    
    std::cout << "Threshold : 0.3 " << std::endl;
//    std::cin >> threshold;
    std::cout << "Margin : 0.1" << std::endl;
//    std::cin >> margin;
    std::cout << "Ripples : 40" << std::endl;
//    std::cin >> ripples;
    std::cout << "Doors : 60" << std::endl;
//    std::cin >> doors;
    
    if(!cap.open(0)){
        return 0;
    }
    std::cout << "Welcome to the maoris test program.\n\n**** Press space to run maoris on the webcam image.\n**** Press t, m, r, or d to change the parameters\nInput: " << std::endl;
    
    bool run = true;
    double input = 0;
    while(run){
                 

        cv::Mat frame;
        cap >> frame;
        if(frame.empty() ) break;
        
        //PROCESS FRAME TODO
        
        cv::imshow("smile", frame);
	    auto output = cv::waitKey(10);
        if(output == 27){ std ::cout << "Quit " << std::endl; run = false;} //Press esc to quit
        if(output == 32) {
	        std::cout << "process" << std::endl;

	        frame = threshold_maps(frame);

	        auto time = std::chrono::system_clock::now();
	        std::time_t end_time = std::chrono::system_clock::to_time_t(time);
	        std::string date = std::ctime(&end_time);
	        date.pop_back();
//	        std::string date_sketch = date + ".png";
	        std::cout << "Saving file to " << date << std::endl;
	        cv::imwrite(date + ".png", frame);

	        std::cout << "Please input the ground truth" << std::endl;
	        auto output = cv::waitKey(10);
	        cv::Mat gt;
	        while(output != 32) {
//		        std::cout << "diff" << std::endl;
		        cap >> gt;
		        cv::imshow("smile", gt);
		        output = cv::waitKey(10);
//		        std::cout << output << std::endl;
	        }
	        gt = threshold_maps(gt);
	        cv::imwrite(date + "_gt.png", gt);
//	        if (output == 32) {
	        process(frame, gt, threshold, margin, ripples, doors);
//	        }
        }
        if(output == 116) {
	        std::cout << "Threshold : " << std::endl;
	        std::cin >> threshold;
        }
	    if(output == 109) {
		    std::cout << "Margin : " << std::endl;
		    std::cin >> margin;
	    }
	    if(output == 114) {
		    std::cout << "Ripples : " << std::endl;
		    std::cin >> ripples;
	    }
	    if(output == 100){
            std::cout << "Doors : " << std::endl;
            std::cin >> doors;
        }

//	    std::cout << "HOY" << std::endl;
    
    }

    //Read image
    
//    cap.close();
    
    
    // Send process
    
    

    

    //add precision mean and recal + nb of file
    // 	std::string result_file = "maoris_param_doors.dat";
    // 	std::cout << "SIZE " << evalparam.size() << std::endl;
    // 	evalparam.exportAll(result_file);

}
