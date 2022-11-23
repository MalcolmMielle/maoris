#include <iostream>
// #define BOOST_TEST_DYN_LINK

// #define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
// #include <boost/test/unit_test.hpp>
#include <sys/stat.h>
#include <sys/time.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/iterator/filter_iterator.hpp>

#include "Evaluation.hpp"
#include "FuzzyOpening.hpp"
#include "Segmentor.hpp"
#include "ZoneExtractor.hpp"
#include "ZoneReducer.hpp"

void draw(AASS::maoris::GraphZone& gp_real,
          AASS::maoris::GraphZone& gp_model,
          const cv::Mat& obstacle,
          const cv::Mat& obstacle_model,
          std::vector<std::pair<AASS::maoris::GraphZone::Vertex,
                                AASS::maoris::GraphZone::Vertex> > matches) {
    cv::Mat obst_copy;
    obstacle.copyTo(obst_copy);

    cv::imshow("OBSOBS", obst_copy);

    cv::Mat obst_model_copy;
    obstacle_model.copyTo(obst_model_copy);
    int cols_max = obst_model_copy.size().width;
    if (cols_max < obst_copy.size().width) {
        cols_max = obst_copy.size().width;
    }

    cv::Size size(cols_max,
                  obst_model_copy.size().height + obst_copy.size().height);
    cv::Mat all = cv::Mat::zeros(size, CV_8UC1);
    // 	cv::Mat only_linked = cv::Mat::zeros(size, CV_8UC3);
    cv::Mat all_maps = cv::Mat::zeros(size, CV_8UC3);

    cv::Mat roi =
        all(cv::Rect(0, 0, obst_copy.size().width, obst_copy.size().height));
    cv::Mat roi_model =
        all(cv::Rect(0, obst_copy.size().height, obst_model_copy.size().width,
                     obst_model_copy.size().height));

    obst_copy.copyTo(roi);
    obst_model_copy.copyTo(roi_model);

    cv::Scalar color;
    cv::RNG rrng(12345);

    if (all.channels() == 1) {
        color = rrng.uniform(50, 255);
    }

    else if (all.channels() == 3) {
        color[1] = rrng.uniform(50, 255);
        color[3] = rrng.uniform(50, 255);
        color[2] = rrng.uniform(50, 255);
    }
    auto it = matches.begin();

    for (; it != matches.end(); ++it) {
        std::cout << "DRAW LINE " << std::endl;

        auto point = gp_model[it->second].getCentroid();
        point.y = point.y + obst_copy.size().height;

        cv::line(all, gp_real[it->first].getCentroid(), point, color, 5);
    }

    cv::imshow("all links", all);
}

void process(const std::string& file,
             const std::string& full_path_GT,
             AASS::maoris::Evaluation& eval) {
    cv::Mat slam_in = cv::imread(file, cv::ImreadModes::IMREAD_GRAYSCALE);
    assert(cv::ImreadModes::IMREAD_GRAYSCALE == 0);

    cv::Mat graphmat = slam_in;

    cv::Mat img_hist_equalized;
    cv::equalizeHist(graphmat, img_hist_equalized);
    cv::imshow("TEST", img_hist_equalized);

    // Reading the GT

    cv::Mat image_GT = cv::imread(full_path_GT, 0);

    cv::Mat GT_segmentation = AASS::maoris::segment_Ground_Truth(image_GT);

    cv::Mat img_hist_equalizedgt;
    cv::equalizeHist(GT_segmentation, img_hist_equalizedgt);
    cv::imshow("GT", img_hist_equalizedgt);

    assert(graphmat.rows == GT_segmentation.rows);
    assert(graphmat.cols == GT_segmentation.cols);

    double pixel_precision, pixel_recall;
    std::vector<std::vector<float> > Precisions, Recalls;
    std::vector<double> Times;

    eval.compare(graphmat, GT_segmentation, 0, file);
}

int main(int argc, char** argv) {
    AASS::maoris::Evaluation eval;

    std::string path_file = argv[1];
    std::string path_gt = argv[2];

    boost::filesystem::path p(path_file);
    boost::filesystem::path p_gt(path_gt);
    try {
        if (!boost::filesystem::exists(p) || !boost::filesystem::exists(p_gt)) {
            std::cout << "need a valid path toward the images" << std::endl;
            return 0;
        }
        if (!boost::filesystem::is_directory(p) ||
            !boost::filesystem::is_directory(p_gt)) {
            std::cout << "need a valid path folder toward the images"
                      << std::endl;
            return 0;
        }

        if (boost::filesystem::is_directory(p)) {
            std::vector<boost::filesystem::path> v;
            // Get all files and sort them
            std::copy(boost::filesystem::directory_iterator(p),
                      boost::filesystem::directory_iterator(),
                      std::back_inserter(v));
            std::sort(v.begin(), v.end());

            for (std::vector<boost::filesystem::path>::const_iterator it(
                     v.begin());
                 it != v.end(); it = ++it) {
                boost::filesystem::path fn = *it;

                std::string name = fn.filename().string();
                std::string model = path_gt + name;

                std::cout << "Process " << fn.string() << " with model "
                          << model << std::endl;

                process(fn.string(), model, eval);

                std::cout << "SIZE " << eval.size() << std::endl;
            }
        }
    } catch (const boost::filesystem::filesystem_error& ex) {
        std::cout << ex.what() << '\n';
    }

    // add precision mean and recal + nb of file
    std::string result_file = "maoris_all_measures_2gtfolder.dat";
    std::cout << "SIZE " << eval.size() << std::endl;
    eval.exportAll(result_file);
}
