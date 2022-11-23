#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <boost/test/unit_test.hpp>
#include <cstdlib>
#include <ctime>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/iterator/filter_iterator.hpp>

#include "FuzzyOpening.hpp"
#include "ZoneExtractor.hpp"
#include "ZoneReducer.hpp"

int i = 0;

void makeGraph(const std::string& file, AASS::maoris::GraphZone& graph_slam) {
    ++i;

    cv::Mat slam_in = cv::imread(file, cv::ImreadModes::IMREAD_GRAYSCALE);
    cv::Mat slam = slam_in > 250;
    //
    // 	cv::imshow("input", slam);
    // 	cv::waitKey(0);

    // 	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY);
    // 	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY_INV);

    std::cout << "/************ FUZZY OPENING*************/ \n";
    AASS::maoris::FuzzyOpening fuzzy_slam;
    fuzzy_slam.fast(false);

    cv::Mat out_slam;
    cv::imshow("SLAM", slam);
    cv::waitKey(0);
    fuzzy_slam.fuzzyOpening(slam, out_slam, 500);
    std::cout << "Done opening " << std::endl;
    out_slam.convertTo(out_slam, CV_8U);

    // 	std::cout << out << std::endl;

    std::cout
        << "/************ REDUCING THE SPACE OF VALUES *****************/\n";
    cv::Mat out_tmp_slam;
    AASS::maoris::reduceZone(out_slam, out_tmp_slam, 5);

    cv::imshow("REDUCED", out_tmp_slam);
    std::cout << "CLICK now" << std::endl;
    cv::waitKey(0);
    std::cout << "CLICKed" << std::endl;

    AASS::maoris::ZoneExtractor zone_maker;
    zone_maker.addValueToIgnore(0);
    std::cout << "WHATERSHED SLAM" << std::endl;
    zone_maker.extract(out_tmp_slam);

    std::cout << "Got the ZONES" << std::endl;

    // 	std::cout << "Getting the graph" << std::endl;

    std::cout
        << "/*********** MAKING AND TRIMMING THE GRAPH ***************/\n";
    graph_slam = zone_maker.getGraph();
    graph_slam.setThreshold(0.25);
    // 	graph_slam.removeVertexValue(0);

    int size_to_remove2 = 10;
    graph_slam.removeVertexUnderSize(size_to_remove2, true);

    std::cout << "Init the maps" << std::endl;
    graph_slam.useCvMat(true);
    std::cout << "Done " << std::endl;
    graph_slam.updateContours();

    cv::Mat graphmat2 = cv::Mat::zeros(out_slam.size(), CV_8U);
    graph_slam.drawSimple(graphmat2);
    std::string s = std::to_string(i);
    cv::imshow(s, graphmat2);
    std::cout << "CLICK now" << std::endl;
    cv::waitKey(0);
    std::cout << "CLICKED" << std::endl;

    std::cout << "Remove ripples" << std::endl;
    graph_slam.removeRiplesv3();
    std::cout << "update contour" << std::endl;
    graph_slam.updateContours();

    cv::Mat graphmat3 = cv::Mat::zeros(out_slam.size(), CV_8U);
    graph_slam.draw(graphmat3);
    std::string ss = std::to_string(i + 1);
    cv::resize(graphmat3, graphmat3,
               cv::Size(graphmat3.cols * 2, graphmat3.rows * 2));
    cv::imshow("RIPPLE", graphmat3);
    cv::waitKey(0);
    std::cout << "CLICKED" << std::endl;

    std::cout << "Number of nodes" << graph_slam.getNumVertices() << std::endl;

    // Watershed Algorithm
    graph_slam.watershed();

    int size_to_remove = 100;
    graph_slam.removeVertexUnderSize(size_to_remove, true);
    graph_slam.removeLonelyVertices();
    if (graph_slam.lonelyVertices())
        throw std::runtime_error("Fuck you lonelyness");
}

BOOST_AUTO_TEST_CASE(trying) {
    int argc = boost::unit_test::framework::master_test_suite().argc;
    char** argv = boost::unit_test::framework::master_test_suite().argv;

    std::string file = argv[1];
    // 	std::string file = "../../Test/Thermal/cold.jpg";
    AASS::maoris::GraphZone graph_slam;
    makeGraph(file, graph_slam);

    cv::Mat slam1 = cv::imread(file, cv::ImreadModes::IMREAD_GRAYSCALE);

    /********** PCA of all zones in Graph and removing the ripples **********/

    // 	graph_slam.updatePCA();
    graph_slam.updateContours();

    cv::Mat graphmat = cv::Mat::zeros(slam1.size(), CV_8U);
    graph_slam.drawSimple(graphmat);
    cv::resize(graphmat, graphmat,
               cv::Size(graphmat.cols * 2, graphmat.rows * 2));
    cv::imshow("GRAPH", graphmat);

    std::cout << "Size of graph" << graph_slam.getNumVertices() << std::endl;

    cv::waitKey(0);
}
