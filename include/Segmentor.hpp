#ifndef MAORIS_SEGMENTOR_17072017
#define MAORIS_SEGMENTOR_17072017

#include <sys/time.h>

#include "FuzzyOpening.hpp"
#include "GraphZone.hpp"
#include "ZoneExtractor.hpp"
#include "ZoneReducer.hpp"

namespace AASS {

namespace maoris {

class Segmentor {
   protected:
    std::vector<std::vector<cv::Point> > _contours;
    std::vector<std::pair<cv::Point, cv::Point> > _limits;
    ///@brief Segmented map with straighten lines
    cv::Mat _segmented;

   public:
    Segmentor() {}

    /**
     *
     * @param src voila
     * @param graph_src
     * @return
     */
    double segmentImage(cv::Mat& src, AASS::maoris::GraphZone& graph_src);

    cv::Mat& getSegmentedMap() { return _segmented; }
    const cv::Mat& getSegmentedMap() const { return _segmented; }

   private:
    double getTime()  // in millisecond
    {
        // assuming unix-type systems
        // timezone tz;
        timeval tv;
        gettimeofday(&tv, NULL);
        return (tv.tv_sec * 1000000 + tv.tv_usec) * 1.0 / 1000;
    }

    void addHoles(const cv::Mat& src,
                  std::vector<std::vector<cv::Point> > contours,
                  std::vector<cv::Vec4i> hierarchy,
                  AASS::maoris::GraphZone& graph_src);

    void findLimits(const cv::Mat& src, AASS::maoris::GraphZone& graph_src);
};

}  // namespace maoris
}  // namespace AASS
#endif