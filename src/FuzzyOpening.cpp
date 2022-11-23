#include "FuzzyOpening.hpp"

// Needs to be int now
void AASS::maoris::FuzzyOpening::addPointValueInCircle(cv::Mat& input,
                                                       cv::Mat& output,
                                                       int value) {
    // Making circle;
    cv::Mat element;

    // TODO : make circle drawing faster ! The problem is escentially the
    // drawing time of the circles If the algorithm doesn't need to run fast, we
    // run a circluar element for more accurate result
    if (_fast == false) {
        element = _masks.at(value - 1);
    }
    // To go fast we use a rectangular element for comparison
    else {
        element = cv::Mat::ones(value * 2, value * 2, CV_32F);
    }

    if (input.rows > element.rows || input.cols > element.cols) {
        std::ostringstream str_test;
        str_test << "Input is bigger than elemtn at line " << __LINE__
                 << " in file " << __FILE__ << "." << std::endl
                 << "Input rows and col : " << input.rows << " " << input.cols
                 << " element : " << element.rows << " " << element.cols;
        throw std::runtime_error(str_test.str());
    }

    for (int row = 0; row < input.rows; row++) {
        float* p = input.ptr<float>(row);            // point to each row
        float* p_output = output.ptr<float>(row);    // point to each row
        float* p_element = element.ptr<float>(row);  // point to each row
        for (int col = 0; col < input.cols; col++) {
            if ((float)p[col] > 0 && (float)p_output[col] < value &&
                (float)p_element[col] > 0) {
                p_output[col] = value;
            }
        }
    }
}

void AASS::maoris::FuzzyOpening::fuzzyOpening(const cv::Mat& src,
                                              cv::Mat& output,
                                              int size) {
    createAllMasks(size);

    // Calcul distance image
    cv::Mat distance_image, label;
    if (src.channels() == 3) {
        std::cout << "Convert" << std::endl;
        cv::cvtColor(src, distance_image, cv::COLOR_RGB2GRAY);
    } else {
        src.copyTo(distance_image);
    }

    cv::distanceTransform(distance_image, distance_image, label, cv::DIST_L2,
                          cv::DIST_MASK_PRECISE, cv::DIST_LABEL_CCOMP);
    CV_Assert(distance_image.depth() == CV_32F);

    int pad = distance_image.rows;
    int old_rows = distance_image.rows;
    int old_cols = distance_image.cols;
    if (pad < distance_image.cols) {
        pad = distance_image.cols;
    }

    cv::copyMakeBorder(distance_image, distance_image, pad, pad, pad, pad,
                       cv::BORDER_CONSTANT, 0);

    output = cv::Mat::zeros(distance_image.rows, distance_image.cols, CV_32F);
    cv::Mat roi_output_final = output(cv::Rect(pad, pad, old_cols, old_rows));
    int count = 0;
    for (int row = pad;
         row < distance_image.rows - pad + 1 /*&& count < 15000*/; row++) {
        float* p = distance_image.ptr<float>(row);  // point to each row
        float* p_output = output.ptr<float>(row);   // point to each row
        for (int col = pad;
             col < distance_image.cols - pad + 1 /*&& count < 15000*/; col++) {
            int dist_to_obstacle = (float)p[col];
            dist_to_obstacle;
            if (dist_to_obstacle > size) {
                dist_to_obstacle = size;
            }

            cv::Mat roi = distance_image(
                cv::Rect(col - (dist_to_obstacle), row - (dist_to_obstacle),
                         dist_to_obstacle * 2, dist_to_obstacle * 2));
            cv::Mat roi_output = output(
                cv::Rect(col - (dist_to_obstacle), row - (dist_to_obstacle),
                         dist_to_obstacle * 2, dist_to_obstacle * 2));
            if ((int)dist_to_obstacle > 0) {
                addPointValueInCircle(roi, roi_output, dist_to_obstacle);
            } else {
                p_output[col] = 0;
            }
        }
    }

    // Need to be two different matrices now
    cv::Mat outout;
    cv::normalize(roi_output_final, outout, 0, 255, cv::NORM_MINMAX, CV_8U);

    output = outout;
}

// I don't need that in the end
bool AASS::maoris::FuzzyOpening::circleIsEmpty(cv::Mat& input,
                                               cv::Mat& circle) {
    for (int row = 0; row < input.rows; row++) {
        uchar* p = input.ptr(row);          // point to each row
        uchar* p_circle = circle.ptr(row);  // point to each row
        for (int col = 0; col < input.cols; col++) {
            if (p[col] > 0 && p_circle[col] > 0) {
                return false;
            }
        }
    }
    return true;
}
