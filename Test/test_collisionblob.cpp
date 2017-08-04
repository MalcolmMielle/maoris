#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include <ctime>

#include "ZoneExtractor.hpp"
#include "FuzzyOpening.hpp"
// #include "Kmean.hpp"
#include "ZoneReducer.hpp"

BOOST_AUTO_TEST_CASE(trying)
{
	
	cv::Mat slam = cv::imread("../Test/blob1.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat map = cv::imread("../Test/blob2.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::imshow("Base input ", map);
	cv::imshow("SLAM", slam);

// 	std::cout << out << std::endl;
	
// 	cv::normalize(out, out, 0, 1, cv::NORM_MINMAX, CV_32F);
	
	std::cout << "Done opening " << std::endl;
	slam.convertTo(slam, CV_8U);
	map.convertTo(map, CV_8U);
	
// 	std::cout << out << std::endl;

	
	AASS::maoris::ZoneExtractor wzoneextract;
	wzoneextract.extract(map);
	AASS::maoris::ZoneExtractor wzoneextract2;
	wzoneextract2.extract(slam);
	
	auto graph_slam = wzoneextract2.getGraph();
	auto graph_map = wzoneextract.getGraph();
	
	cv::Mat draw;
	map.copyTo(draw);
	
	graph_map.draw(draw);
	cv::imshow("Map", draw);
	cv::waitKey(0);
	std::cout << "CLICKED" << std::endl;
	
	BOOST_CHECK_EQUAL(graph_map.getNumVertices(), 2);
	BOOST_CHECK_EQUAL(graph_slam.getNumVertices(), 2);
	
	int count;
	//vertices access all the vertix
	for (auto vp = boost::vertices(graph_slam.getGraph()); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
		++vp.first;
		auto v2 = *vp.first;
		auto zonemap = graph_map[v2];
		auto zoneslam = graph_slam[v];
		std::cout << "CCONTACT : " << zoneslam.contactPoint(zonemap) << std::endl;
		count = zoneslam.contactPoint(zonemap);
			
	}
	
	BOOST_CHECK_EQUAL(count, 35);
	
	
}