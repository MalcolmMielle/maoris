#include "Zone.hpp"

void AASS::maoris::Zone::updateContour() {
    cv::Mat copy_tmp;
    _zone_mat.copyTo(copy_tmp);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(copy_tmp, contours, hierarchy, cv::RETR_LIST,
                     cv::CHAIN_APPROX_NONE);

    // Calculate the area of each contour
    // Use a lambda function to sort the contours
    std::sort(contours.begin(), contours.end(),
              [](std::vector<cv::Point>& pts, std::vector<cv::Point>& pts2) {
                  return pts.size() > pts2.size();
              });

    if (contours.size() == 0) {
        throw ZoneHasNoContour("NO CONTOUR IN ZONE !");
    }
    _contours = contours;
}
