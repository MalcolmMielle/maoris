#include "Segmentor.hpp"

double AASS::maoris::Segmentor::segmentImage(
    cv::Mat& src,
    AASS::maoris::GraphZone& graph_src) {
    cv::Mat outer;
    src.copyTo(outer);

    double begin_process, end_process, decompose_time;
    std::cout << "/************ FUZZY OPENING*************/ \n";
    AASS::maoris::FuzzyOpening fuzzy_slam;
    fuzzy_slam.fast(false);

    cv::Mat out_slam;

    begin_process = getTime();
    fuzzy_slam.fuzzyOpening(outer, out_slam, 500);
    end_process = getTime();
    decompose_time = end_process - begin_process;
    double time = decompose_time;

    std::cout << "Fuzzy opening time: " << time << std::endl;

    out_slam.convertTo(out_slam, CV_8U);

    std::cout
        << "/************ REDUCING THE SPACE OF VALUES *****************/\n";
    cv::Mat out_tmp_slam;
    AASS::maoris::ZoneExtractor zone_maker;
    zone_maker.addValueToIgnore(0);

    begin_process = getTime();
    AASS::maoris::reduceZone(out_slam, out_tmp_slam, 5);
    zone_maker.extract(out_tmp_slam);
    end_process = getTime();
    decompose_time = end_process - begin_process;
    time = time + decompose_time;

    std::cout << "Zone reducing: " << decompose_time << std::endl;

    std::cout
        << "/*********** MAKING AND TRIMMING THE GRAPH ***************/\n";

    int size_to_remove2 = 10;

    begin_process = getTime();

    double thres = graph_src.getT();
    double marg = graph_src.getMargin();
    double t_ripples = graph_src.getThresholdFusionRipples();
    double t_doors = graph_src.getThresholdFusionDoors();

    graph_src = zone_maker.getGraph();

    graph_src.setThreshold(thres);
    graph_src.setMargin(marg);
    graph_src.setThresholdFusionRipples(t_ripples);
    graph_src.setThresholdFusionDoors(t_doors);

    /// THIS IS JUST HERE TO SPEED THINGS UP... So small vertex are removed from
    /// image to speed up computation.
    graph_src.removeVertexUnderSize(size_to_remove2, true);

    graph_src.useCvMat(true);

    graph_src.updateContours();
    graph_src.removeRiplesv3();

    end_process = getTime();
    decompose_time = end_process - begin_process;
    time = time + decompose_time;

    begin_process = getTime();
    graph_src.updateContours();

    // Watershed Algorithm
    graph_src.watershed();
    std::cout << "NB ZONE " << graph_src.getNumVertices() << std::endl;

    graph_src.removeDoors();

    int size_to_remove = 100;
    // TODO: Should actually fuse
    graph_src.removeVertexUnderSize(size_to_remove, true);

    // TODO: REMOVE THIS
    if (graph_src.getNumVertices() > 1) {
        graph_src.removeLonelyVertices();
    }
    end_process = getTime();
    decompose_time = end_process - begin_process;
    time = time + decompose_time;

    // TODO HENCE REMOVE THIS
    if (graph_src.getNumVertices() > 1) {
        if (graph_src.lonelyVertices())
            throw std::runtime_error("Fuck you lonelyness");
    }
    begin_process = getTime();
    findLimits(outer, graph_src);
    end_process = getTime();
    decompose_time = end_process - begin_process;
    time = time + decompose_time;

    cv::Mat copy;
    outer.copyTo(copy);

    begin_process = getTime();
    for (auto it = _limits.begin(); it != _limits.end(); ++it) {
        cv::line(copy, it->first, it->second, cv::Scalar(100), 1);
    }
    end_process = getTime();
    decompose_time = end_process - begin_process;
    time = time + decompose_time;

    _segmented = copy;

    return time;
}

void AASS::maoris::Segmentor::addHoles(
    const cv::Mat& src,
    std::vector<std::vector<cv::Point> > contours,
    std::vector<cv::Vec4i> hierarchy,
    AASS::maoris::GraphZone& graph_src) {
    // Draw all holes on a Mat
    cv::Mat drawing = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);
    // iterate through all the top-level contours,
    for (int i = 0; i < contours.size(); ++i) {
        if (hierarchy[i][3] != -1) {
            cv::drawContours(drawing, contours, i, 255, cv::FILLED, 8,
                             hierarchy);
        }
    }

    for (int row = 0; row < drawing.rows; ++row) {
        uchar* p = drawing.ptr(row);
        for (int col = 0; col < drawing.cols; ++col) {
            // If the pixel is part of a hole
            if (p[col] != 0) {
                // For all zone remove it if it is part of the zone
                std::pair<AASS::maoris::GraphZone::VertexIteratorZone,
                          AASS::maoris::GraphZone::VertexIteratorZone>
                    vp;

                for (vp = boost::vertices(graph_src); vp.first != vp.second;) {
                    auto v = *vp.first;
                    ++vp.first;

                    graph_src[v].removePoint(row, col);
                }
            }
        }
    }
}

void AASS::maoris::Segmentor::findLimits(const cv::Mat& src_mat,
                                         AASS::maoris::GraphZone& graph_src) {
    auto touchMat = [&src_mat](int x, int y, cv::Point2i& out) -> bool {
        int xx;
        for (xx = x - 2; xx < x + 3; ++xx) {
            int yy;
            for (yy = y - 2; yy < y + 3; ++yy) {
                if (src_mat.at<uchar>(yy, xx) == 0) {
                    out.x = xx;
                    out.y = yy;
                    return true;
                }
            }
        }
        return false;
    };

    // Return closest point to p, on contour vp, that touches a wall
    auto distPointPerimeter =
        [touchMat](cv::Point2i p,
                   std::vector<std::vector<cv::Point2i> > vp) -> cv::Point2i {
        double dist = -1;
        auto it_contour = vp.begin();
        cv::Point2i p_out = (*it_contour)[0];
        for (it_contour; it_contour != vp.end(); ++it_contour) {
            auto it = it_contour->begin();
            for (it; it != it_contour->end(); ++it) {
                cv::Point2i out;
                if (touchMat(it->x, it->y, out) == true) {
                    auto tdist = cv::norm(*it - p);
                    if (dist == -1 || dist > tdist) {
                        dist = tdist;
                        p_out = out;
                    }
                }
            }
        }
        return p_out;
    };

    std::pair<AASS::maoris::GraphZone::VertexIteratorZone,
              AASS::maoris::GraphZone::VertexIteratorZone>
        vp;
    std::set<AASS::maoris::GraphZone::EdgeZone> visited_edges;
    for (vp = boost::vertices(graph_src); vp.first != vp.second;) {
        auto v = *vp.first;
        ++vp.first;
        AASS::maoris::GraphZone::EdgeIteratorZone out_i, out_end;
        // Since we fuse the old zone in biggest we only need to link them to
        // biggest
        for (boost::tie(out_i, out_end) = boost::out_edges(v, graph_src);
             out_i != out_end; out_i = ++out_i) {
            AASS::maoris::GraphZone::EdgeZone e_second = *out_i;
            if (std::find(visited_edges.begin(), visited_edges.end(),
                          e_second) == visited_edges.end()) {
                visited_edges.insert(e_second);

                AASS::maoris::GraphZone::VertexZone targ =
                    boost::target(e_second, graph_src);
                AASS::maoris::GraphZone::VertexZone src =
                    boost::source(e_second, graph_src);

                auto contact =
                    graph_src[src].getContactPointSeparated(graph_src[targ]);

                assert(contact.size() > 0);
                cv::Point l1, l2;
                for (auto it3 = contact.begin(); it3 != contact.end(); ++it3) {
                    for (auto it = it3->begin(); it != it3->end(); ++it) {
                        double distance = -1;
                        for (auto it2 = it3->begin(); it2 != it3->end();
                             ++it2) {
                            auto tdist = cv::norm(*it - *it2);
                            if (distance == -1 || distance < tdist) {
                                l1 = *it;
                                l2 = *it2;
                                distance = tdist;
                            }
                        }
                    }

                    auto contour = graph_src[src].getContour();
                    double dist = -1;
                    auto l1_f = distPointPerimeter(l1, contour);
                    auto l2_f = distPointPerimeter(l2, contour);
                    std::cout << "Pushing back " << l1_f << " " << l2_f
                              << std::endl;
                    _limits.push_back(
                        std::pair<cv::Point2i, cv::Point2i>(l1_f, l2_f));
                }
            }
        }
    }
}
