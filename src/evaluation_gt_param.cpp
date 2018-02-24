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



void process(const std::string& file, const std::string& full_path_GT, AASS::maoris::Evaluation& eval, double t, int test_what){

	AASS::maoris::GraphZone graph_slam;
	if(test_what == 1){
		graph_slam.setThreshold(t);
		graph_slam.setMargin(0.1);
		graph_slam.setThresholdFusionRipples(40);
		graph_slam.setThresholdFusionDoors(40);
	}
	if(test_what == 2){
		graph_slam.setThreshold(0.30);
		graph_slam.setMargin(t);
		graph_slam.setThresholdFusionRipples(40);
		graph_slam.setThresholdFusionDoors(40);
	}
	if(test_what == 3){
		graph_slam.setThreshold(0.30);
		graph_slam.setMargin(0.1);
		graph_slam.setThresholdFusionRipples(t);
		graph_slam.setThresholdFusionDoors(40);
	}
	if(test_what == 4){
		graph_slam.setThreshold(0.30);
		graph_slam.setMargin(0.1);
		graph_slam.setThresholdFusionRipples(40);
		graph_slam.setThresholdFusionDoors(t);
	}
	
	cv::Mat slam_in = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
	assert(CV_LOAD_IMAGE_GRAYSCALE == 0);
	cv::Mat slam = slam_in > 250;
	
// 	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY);
// 	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY_INV);
// 	cv::imshow("map in", slam);
// 	cv::waitKey(0);
	std::cout << "Input parameters " << t << std::endl;
	std::cout << "T" << graph_slam.getT() << std::endl;
	double time = 0;
// 	makeGraph(slam, graph_slam, time);
	AASS::maoris::Segmentor segmenteur;
	time = segmenteur.segmentImage(slam, graph_slam);

    std::cout << "NB OF ZONES " << graph_slam.getNumVertices() << std::endl;

	cv::Mat segmented_map = segmenteur.getSegmentedMap();
	
	std::cout << "Total time: " << time << std::endl;
			
	/********** PCA of all zones in Graph and removing the ripples **********/
// 	graph_slam.update();

	
	cv::Mat slam1 = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat graphmat = cv::Mat::zeros(slam1.size(), CV_8U);
    graph_slam.drawEvaluation(graphmat);

// 	cv::Mat partial = cv::Mat::zeros(slam1.size(), CV_8U);
//     graph_slam.drawPartial(partial);
	
// 	cv::Mat img_hist_equalized;
 	//cv::Mat img_hist_equalized;
 	//cv::equalizeHist(graphmat, img_hist_equalized);
 	//cv::resize(graphmat, graphmat, cv::Size(graphmat.cols * 2, graphmat.rows * 2));
 	//cv::imshow("GRAPH", img_hist_equalized);
// 	
//     cv::Mat graphmat_vis = cv::Mat::zeros(slam1.size(), CV_8U);
//     graph_slam.draw(graphmat_vis);
// 	cv::resize(graphmat, graphmat, cv::Size(graphmat.cols * 2, graphmat.rows * 2));
// 	cv::imshow("GRAPH Visible", graphmat_vis);
	
// 	std::cout << "Size of graph" << graph_slam.getNumVertices() << std::endl;
    
	
	//Reading the GT
	
// 	std::cout << " GT " << full_path_GT << std::endl;
// 	
	cv::Mat image_GT = cv::imread(full_path_GT,0);
// 	cv::imshow("GT raw", image_GT);
 	//cv::waitKey(0);
	
	cv::Mat GT_segmentation = AASS::maoris::segment_Ground_Truth(image_GT);
    /// THIS ONE IS USE IF YOU NEED STRAIGHTEN MAPS
	cv::Mat graphmat_straight = AASS::maoris::segment_Ground_Truth(segmented_map);
	
// 	cv::Mat img_hist_equalizedgt;
// 	cv::equalizeHist(GT_segmentation, img_hist_equalizedgt);
// 	cv::imshow("GT", img_hist_equalizedgt);
 	//cv::waitKey(0);

//    SKETCHES
//    eval.compare(graphmat, GT_segmentation, time, file);
//    ROBOT MAPS
    eval.compare(graphmat_straight, GT_segmentation, time, file);
	std::cout << "SIZE " << eval.size() << std::endl;
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


    std::vector<std::string> gts;
    std::string maps;
    std::tie(maps, gts) = getMapsAndGT();

    int test_what = -1;
    if(argc >= 2) {
        test_what = atoi(argv[1]);
    }
//	std::string path_gt = argv[2];
// 	std::string file = "../../Test/Thermal/cold.jpg";

    int which_to_end = 5;
    if(test_what != -1){
        assert(test_what <= 4);
        assert(test_what >= 1);
        which_to_end = test_what + 1;
    }else{
        test_what = 1;
    }

    std::cout << test_what << " " << which_to_end << std::endl;
    for (; test_what < which_to_end; ++test_what) {

        AASS::maoris::EvaluationParam evalparam;

        double start = 0;
        double t = 0;
        double step = 0;
        double end = 0;

	    //END IS INCLUDED IN THE LOOP
        //Threshold
        if (test_what == 1) {
            std::cout << "THRESHOLD" << std::endl;
            t = 0.05;
            start = t;
            step = 0.05;
            end = 1;
        }
        //Margin
        else if (test_what == 2) {
            std::cout << "MARGIN" << std::endl;
            t = 0;
            start = t;
            step = 0.05;
            end = 1;
        }
        //Ripples
        else if (test_what == 3) {
            std::cout << "RIPPLES" << std::endl;
            t = 85;
            start = t;
            step = 5;
            end = 85;
        }
        //Doors
        else if (test_what == 4) {
            std::cout << "DOORS" << std::endl;
            t = 0;
            start = t;
            step = 5;
            end = 100;
        } else {
            throw std::runtime_error("TOO FAR");
        }


        try {

            for (auto gt : gts) {

                boost::filesystem::path p(maps);
                boost::filesystem::path p_gt(gt);


                if (!boost::filesystem::exists(p) || !boost::filesystem::exists(p_gt)) {
                    std::cout << "need a valid path toward the images" << std::endl;
                    return 0;
                }
                if (!boost::filesystem::is_directory(p) || !boost::filesystem::is_directory(p_gt)) {
                    std::cout << "need a valid path folder toward the images" << std::endl;
                    return 0;
                }

                if (boost::filesystem::is_directory(p)) {

                    for (t; t <= end; t = t + step) {
                        // 				for(m = 0; m <= 1 ; m = m + 0.05){

                        std::cout << "STAT T " << t << std::endl;
                        AASS::maoris::Evaluation eval;

                        std::vector<boost::filesystem::path> v;
                        //Get all files and sort them
                        std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(),
                                  std::back_inserter(v));
                        std::sort(v.begin(), v.end());



                        // 			int i = 0;
                        for (std::vector<boost::filesystem::path>::const_iterator it(v.begin());
                             it != v.end(); it = ++it) {
                            boost::filesystem::path fn = *it;

                            std::string name = fn.filename().string();
                            std::string model = gt + name;

                            std::cout << "Process " << fn.string() << " with model " << model << std::endl;

                            process(fn.string(), model, eval, t, test_what);

                            std::cout << "SIZE " << eval.size() << std::endl;
                            // 				if(i == 3){
                            // 					return 0;
                            // 				}
                            // 				++i;
                        }

                        eval.calculate();
                        evalparam.add(eval, t);

                    }
                }
                else{
                    std::cout << "NOt dir " << std::endl;
                }
            }

            //add precision mean and recal + nb of file
            std::string result_file;

            if (test_what == 1) {
                result_file = "maoris_param_threshold_bormann.dat";
            }
            if (test_what == 2) {
                result_file = "maoris_param_margin_bormann.dat";
            }
            if (test_what == 3) {
                result_file = "maoris_param_ripples_bormann.dat";
            }
            if (test_what == 4) {
                result_file = "maoris_param_doors_bormann.dat";
            }

            std::cout << "SIZE " << evalparam.size() << std::endl;
            evalparam.exportAll(result_file, start, step);
        }
        catch (const boost::filesystem::filesystem_error &ex) {
            std::cout << "ERROR" << ex.what() << '\n';
            exit(0);
        }
    }

    //add precision mean and recal + nb of file
    // 	std::string result_file = "maoris_param_doors.dat";
    // 	std::cout << "SIZE " << evalparam.size() << std::endl;
    // 	evalparam.exportAll(result_file);

}
