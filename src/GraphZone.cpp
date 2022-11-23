#include "GraphZone.hpp"

void AASS::maoris::GraphZone::removeVertexUnderSize(int size,
                                                    bool preserveEdgeConnectic,
                                                    bool flag_to_test_if_used) {
    std::pair<VertexIteratorZone, VertexIteratorZone> vp;
    // vertices access all the vertix
    for (vp = boost::vertices((*this)); vp.first != vp.second;) {
        VertexZone v = *vp.first;
        ++vp.first;
        if ((*this)[v].size() < size) {
            if (flag_to_test_if_used) {
                std::cout << "WHAT THE FUCK WHY REMOVE :( ?" << std::endl;
                exit(0);
            }

            if (preserveEdgeConnectic == true) {
                if (getNumEdges(v) > 0) {
                    try {
                        removeVertexWhilePreservingEdges(v, false);
                    } catch (std::exception& e) {
                        std::cout << "Here : " << __LINE__ << " " << __FILE__
                                  << " " << e.what() << std::endl;
                        exit(0);
                    }
                } else {
                    removeVertex(v);
                }
            } else {
                removeVertex(v);
            }
        }
    }
}

bool AASS::maoris::GraphZone::asVerticesWithNoEdges() {
    std::pair<VertexIteratorZone, VertexIteratorZone> vp;
    // vertices access all the vertix
    for (vp = boost::vertices((*this)); vp.first != vp.second;) {
        VertexZone v = *vp.first;
        ++vp.first;
        if (this->getNumEdges(v) == 0) {
            return true;
        }
    }
    return false;
}

double AASS::maoris::GraphZone::contactPointWithWalls(
    AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone v) {
    EdgeIteratorZone out_i, out_end;
    int contact_point = 0;

    for (boost::tie(out_i, out_end) = boost::out_edges(v, (*this));
         out_i != out_end;) {
        EdgeZone e_second = *out_i;
        VertexZone targ = boost::target(e_second, (*this));
        contact_point = contact_point + (*this)[v].contactPoint((*this)[targ]);
        out_i++;
    }
    return contact_point;
}

void AASS::maoris::GraphZone::removeDoors() {
    std::pair<VertexIteratorZone, VertexIteratorZone> vp;
    std::map<VertexZone, double> contact_point;

    int i = 0;
    for (vp = boost::vertices((*this)); vp.first != vp.second;) {
        VertexZone v = *vp.first;
        double contact = contactPointWithWalls(v);
        ++vp.first;
        contact_point[v] = contact;
    }

    i = 0;
    for (auto it = contact_point.begin(); it != contact_point.end();) {
        ++i;
        if (it->second > _threshold_fusion_doors) {
            EdgeIteratorZone out_i, out_end;
            boost::tie(out_i, out_end) = boost::out_edges(it->first, (*this));
            EdgeZone e_second = *out_i;
            VertexZone targ = boost::target(e_second, (*this));
            VertexZone v_to_fuse_in = targ;

            for (boost::tie(out_i, out_end) =
                     boost::out_edges(it->first, (*this));
                 out_i != out_end;) {
                EdgeZone e_second = *out_i;
                VertexZone targ = boost::target(e_second, (*this));
                if (contact_point[targ] < _threshold_fusion_doors) {
                    v_to_fuse_in = targ;
                }
                out_i++;
            }

            VertexZone vv = it->first;
            removeVertexWhilePreservingEdges(vv, v_to_fuse_in, false);
            (*this)[v_to_fuse_in].updateContour();
            double contact = contactPointWithWalls(v_to_fuse_in);
            contact_point[v_to_fuse_in] = contact;
        }

        ++it;
    }
}

// TODO : would crash on self loop ?
/// Recurisve function to find all node to be fused to the original node by the
/// watershed !
void AASS::maoris::GraphZone::getAllNodeRemovedWatershed(
    AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& top_vertex,
    AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone&
        first_vertex,
    const std::deque<
        AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone>&
        top_vertex_visited,
    std::deque<AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone>&
        top_vertex_visited_tmp,
    double threshold,
    std::deque<VertexZone>& to_be_removed) {
    EdgeIteratorZone out_i, out_end;
    int num_edge = getNumEdges(top_vertex), count = 0;

    std::deque<EdgeZone> listedge;
    for (boost::tie(out_i, out_end) = boost::out_edges(top_vertex, (*this));
         out_i != out_end;) {
        EdgeZone e_second = *out_i;
        listedge.push_back(e_second);
        out_i++;
    }

    top_vertex_visited_tmp.push_back(top_vertex);

    for (boost::tie(out_i, out_end) = boost::out_edges(top_vertex, (*this));
         out_i != out_end;) {
        ++count;
        EdgeZone e_second = *out_i;

        if ((*this)[e_second].canRemove()) {
            VertexZone targ = boost::target(e_second, (*this));

            // Needed to not consider the new added vertex since they are
            // supposed to be stopping point of the recursion. This is a marker
            // for node that are here seen the begining. TODO USELESS
            bool is_old = false;
            for (size_t i = 0; i < listedge.size(); ++i) {
                if (listedge[i] == e_second)
                    is_old = true;
            }

            // Do not visit already seen nodes
            // Mark nodes that are already visited
            bool is_visited = false;
            for (size_t j = 0; j < top_vertex_visited.size(); ++j) {
                if (targ == top_vertex_visited[j]) {
                    // std::cout << "SEEN " << j << std::endl;
                    is_visited = true;
                }
            }

            // Already visited node in this recursion mode. Needed if we don't
            // do a watershed and no direction is kept. It's simply not go
            // backward in the exploration
            bool is_visited_tmp = false;
            for (size_t j = 0; j < top_vertex_visited_tmp.size(); ++j) {
                if (targ == top_vertex_visited_tmp[j]) {
                    // std::cout << "SEEN " << j << std::endl;
                    is_visited_tmp = true;
                }
            }

            double targ_value = (*this)[targ].getValue();
            double first_vertex_value = (*this)[top_vertex].getValue();

            // Comparison using the biggest space as a reference
            double max_value = first_vertex_value;
            double min_value = targ_value;
            if (max_value < min_value) {
                max_value = targ_value;
                min_value = first_vertex_value;
            }

            if (first_vertex_value == 36 || targ_value == 36) {
                std::cout << "Max " << max_value << " min " << min_value
                          << " thre " << threshold << " margin "
                          << _margin_factor << std::endl;
                std::cout << min_value << " >= "
                          << max_value - ((double)max_value *
                                          (threshold + _margin_factor))
                          << std::endl;
            }

            // REMOVE TARG
            if (

                min_value >= max_value - ((double)max_value *
                                          (threshold + _margin_factor)) &&

                is_old == true && is_visited == false &&
                is_visited_tmp == false) {
                // Clearly the same
                if (min_value >= max_value - ((double)max_value * threshold)) {
                    getAllNodeRemovedWatershed(
                        targ, first_vertex, top_vertex_visited,
                        top_vertex_visited_tmp, threshold, to_be_removed);

                    try {
                        to_be_removed.push_back(targ);
                        // TODO save a recursion by doing ++out here !
                    } catch (std::exception& e) {
                        std::cout << "Zone had more than one shape. It's fine "
                                     "at this point in the proccess. Continue"
                                  << std::endl;
                    }
                }
                // It's close but not close enough for us to be sure so we look
                // at the neighbors of the target to see if one is close to the
                // top_vertex
                else {
                    std::cout << "Close but not enough" << std::endl;

                    // Check if targ as neighbor close to top
                    auto closeNeigh = [this, threshold](
                                          const VertexZone& top,
                                          const VertexZone& targ) -> bool {
                        double f_v_value = (*this)[top].getValue();

                        EdgeIteratorZone out_i_tmp, out_end_tmp;
                        for (boost::tie(out_i_tmp, out_end_tmp) =
                                 boost::out_edges(targ, (*this));
                             out_i_tmp != out_end_tmp; ++out_i_tmp) {
                            EdgeZone e_second_tmp = *out_i_tmp;

                            // Only check neighbor with breakable links
                            if ((*this)[e_second_tmp].canRemove()) {
                                VertexZone targ_second_wave =
                                    boost::target(e_second_tmp, (*this));

                                // As long as it's not the original we check for
                                // closeness
                                if (targ_second_wave != top) {
                                    double targ_sw_value =
                                        (*this)[targ_second_wave].getValue();
                                    double max_sw =
                                        std::max(targ_sw_value, f_v_value);
                                    double min_sw =
                                        std::min(targ_sw_value, f_v_value);

                                    if (min_sw >=
                                        max_sw - ((double)max_sw * threshold)) {
                                        std::cout << "Close nieghbor"
                                                  << std::endl;
                                        return true;
                                    }
                                }
                            }
                        }
                        return false;
                    };

                    bool close_value_neighb = closeNeigh(top_vertex, targ);
                    if (close_value_neighb == false) {
                        close_value_neighb = closeNeigh(targ, top_vertex);
                    }

                    // If they were indeed close, they we remove it
                    if (close_value_neighb) {
                        getAllNodeRemovedWatershed(
                            targ, first_vertex, top_vertex_visited,
                            top_vertex_visited_tmp, threshold, to_be_removed);

                        try {
                            to_be_removed.push_back(targ);
                        } catch (std::exception& e) {
                            std::cout
                                << "Zone had more than one shape. It's fine at "
                                   "this point in the proccess. Continue"
                                << std::endl;
                        }
                    } else {
                        std::cout << "Not good " << first_vertex_value << " "
                                  << targ_value << std::endl;
                        ++out_i;
                    }
                }

            } else {
                ++out_i;
            }
        } else {
            ++out_i;
        }
    }
}

void AASS::maoris::GraphZone::watershed(double threshold) {
    std::cout << "Starting watershedd" << std::endl;

    _iteration = 0;
    std::vector<std::pair<std::deque<VertexZone>, VertexZone> >
        all_to_remove_and_in_what;
    std::deque<VertexZone> top_vertex_visited;
    // Find all "top node"

    std::list<VertexZone> all_vertex;
    std::pair<VertexIteratorZone, VertexIteratorZone> vp;
    for (vp = boost::vertices((*this)); vp.first != vp.second;) {
        VertexZone v = *vp.first;
        ++vp.first;
        all_vertex.push_front(v);
    }

    // Sort by smallest to biggest and hightes tpixel value to smallest if size
    // are equal
    all_vertex.sort([this](VertexZone a, VertexZone b) {
        if ((*this)[a].getValue() == (*this)[b].getValue()) {
            // Return biggest value when equal
            return (*this)[a].size() > (*this)[b].size();
        }
        return (*this)[a].getValue() > (*this)[b].getValue();
    });

    for (auto it = all_vertex.begin(); it != all_vertex.end(); ++it) {
        top_vertex_visited.push_back(*it);

        std::deque<VertexZone> top_vertex_visited_tmp;
        top_vertex_visited_tmp.push_back(*it);

        std::deque<VertexZone> to_be_removed;

        int num_edge = getNumEdges(*it), count = 0;
        if (num_edge > 0) {
            getAllNodeRemovedWatershed(*it, *it, top_vertex_visited,
                                       top_vertex_visited_tmp, threshold,
                                       to_be_removed);
            all_to_remove_and_in_what.push_back(
                std::pair<std::deque<VertexZone>, VertexZone>(to_be_removed,
                                                              *it));

            // Remove vertex to be remove from inspection
            for (auto it_rem = to_be_removed.begin();
                 it_rem != to_be_removed.end(); ++it_rem) {
                all_vertex.remove(*it_rem);
            }
        }
    }

    std::cout << "Removing" << std::endl;
    for (auto it_rem = all_to_remove_and_in_what.begin();
         it_rem != all_to_remove_and_in_what.end(); ++it_rem) {
        for (auto it = it_rem->first.begin(); it != it_rem->first.end(); ++it) {
            removeVertexWhilePreservingEdges(*it, it_rem->second, false);
        }
    }

    std::cout << "DONE Watershed" << std::endl;
}

///@Brief fuse two vertex into one.
void AASS::maoris::GraphZone::removeVertexWhilePreservingEdges(
    AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& v,
    AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone&
        v_to_fuse_in,
    bool createUnBreakableLinks) {
    assert(v != v_to_fuse_in);

    EdgeIteratorZone out_i, out_end;

    int diff = (*this)[v_to_fuse_in].getValue() - (*this)[v].getValue();
    bool up = true;
    bool same = false;
    if (diff < 0) {
        up = false;
    } else if (diff == 0) {
        same = true;
    }

    // Since we fuse the old zone in biggest we only need to link them to
    // biggest
    for (boost::tie(out_i, out_end) = boost::out_edges(v, (*this));
         out_i != out_end; out_i = ++out_i) {
        EdgeZone e_second = *out_i;
        VertexZone targ = boost::target(e_second, (*this));

        EdgeIteratorZone out_i_second;

        if (v_to_fuse_in != targ) {
            EdgeZone edz;
            EdgeElement ed_el((*this)[e_second]);

            double min_val = (*this)[v].getValue();
            if (min_val >= (*this)[e_second].getMinimum() &&
                (*this)[e_second].getMinimum() != -2) {
                min_val = (*this)[e_second].getMinimum();
            }

            // Keep the best minimum score in edge
            if (boost::edge(v_to_fuse_in, targ, (*this)).second == true) {
                EdgeZone edz_tmp;
                EdgeElement ed_el_tmp;
                (*this).getEdge(v_to_fuse_in, targ, edz_tmp);
                ed_el_tmp = (*this)[edz_tmp];
                double min_val_tmp = ed_el_tmp.getMinimum();
                if (min_val_tmp == -2) {
                    // 					min_val = min_val_tmp;
                } else if (min_val_tmp <= min_val) {
                    min_val = min_val_tmp;
                }
            }
            ed_el.setMinimum(min_val);

            if ((*this)[e_second].canRemove() == false) {
                ed_el.makeUnbreakable();
            }

            addEdge(edz, targ, v_to_fuse_in, ed_el);
        }
    }

    (*this)[v_to_fuse_in].fuse((*this)[v]);
    removeVertex(v);
}

void AASS::maoris::GraphZone::removeVertexWhilePreservingEdges(
    AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone& v,
    bool createUnBreakableLinks) {
    assert(getNumEdges(v) > 0 && "Node without edges Oo");
    // Find Closest valued neighbor vertex for fusion of zone
    VertexZone closest;
    bool init = true;
    EdgeIteratorZone out_i, out_end;
    // First find neighbor with biggest zone
    for (boost::tie(out_i, out_end) = boost::out_edges(v, (*this));
         out_i != out_end; out_i = ++out_i) {
        EdgeZone e_second = *out_i;
        VertexZone targ = boost::target(e_second, (*this));

        EdgeIteratorZone out_i_second;
        if (init == true) {
            closest = targ;
            init = false;
        } else {
            if (std::abs((*this)[closest].getValue() - (*this)[v].getValue()) >
                std::abs((*this)[targ].getValue() - (*this)[v].getValue())) {
                closest = targ;
            }
        }
    }

    removeVertexWhilePreservingEdges(v, closest, createUnBreakableLinks);
}

// Htis is n!*m n is node number and m is max number of edges
void AASS::maoris::GraphZone::removeRiplesv3(int dist) {
    updateAllEdges();

    std::list<VertexZone> all_vertex;

    std::pair<VertexIteratorZone, VertexIteratorZone> vp;
    for (vp = boost::vertices((*this)); vp.first != vp.second;) {
        VertexZone v = *vp.first;
        ++vp.first;
        all_vertex.push_front(v);
    }

    // Sort by smallest to biggest and hightes tpixel value to smallest if size
    // are equal
    all_vertex.sort([this](VertexZone a, VertexZone b) {
        if ((*this)[a].getValue() == (*this)[b].getValue()) {
            // Return biggest value when equal
            return (*this)[a].size() < (*this)[b].size();
        }
        return (*this)[a].getValue() > (*this)[b].getValue();
    });

    for (auto it = all_vertex.begin(); it != all_vertex.end();) {
        VertexZone to_fuse_in;
        if (checkAndReplaceRipple(*it, to_fuse_in)) {
            try {
                std::deque<VertexZone> to_check;
                this->getAllVertexLinked(*it, to_check);

                int add = 0;
                // Check the neighbor for ripples so we add right after in the
                // list
                for (int i = 0; i < to_check.size(); ++i) {
                    if (std::find(it, all_vertex.end(), to_check[i]) ==
                        all_vertex.end()) {
                        all_vertex.push_back(to_check[i]);
                        ++add;
                    }
                }
            } catch (std::exception& e) {
                std::cout << "Here : " << e.what() << std::endl;
                cv::Mat graphmat2 = cv::Mat::zeros(600, 600, CV_8U);
                (*this)[*it].drawZone(graphmat2, cv::Scalar(100));
                (*this)[*it].drawContour(graphmat2, cv::Scalar(100));
                cv::imshow("fused 222", graphmat2);
                cv::waitKey(0);
                exit(0);
            }

            removeVertexWhilePreservingEdges(*it, to_fuse_in, true);
            (*this)[to_fuse_in].updateContour();

            auto it_cpy = it;
            while (*it_cpy == *it) {
                ++it;
            }
            all_vertex.remove(*it_cpy);
        } else {
            ++it;
        }
    }
    for (auto it = all_vertex.begin(); it != all_vertex.end(); ++it) {
        makeAllUnbreakableEdges(*it);
    }
}

bool AASS::maoris::GraphZone::checkAndReplaceRipple(
    AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone&
        might_be_ripple,
    AASS::maoris::GraphZoneInterface<Zone, EdgeElement>::VertexZone&
        to_fuse_in) {
    EdgeIteratorZone out_i, out_end;
    int num_edge = getNumEdges(might_be_ripple);

    bool out = false;
    double value = -1;

    for (boost::tie(out_i, out_end) =
             boost::out_edges(might_be_ripple, (*this));
         out_i != out_end;) {
        EdgeZone e_second = *out_i;
        ++out_i;

        if ((*this)[e_second].canRemove()) {
            VertexZone targ = boost::target(e_second, (*this));

            if (isRipple(targ, might_be_ripple)) {
                // Keep the closest vertex ripple
                if (value == -1 ||
                    std::abs(value - (*this)[might_be_ripple].getValue()) >
                        std::abs((*this)[targ].getValue() -
                                 (*this)[might_be_ripple].getValue())) {
                    to_fuse_in = targ;
                    out = true;
                    value = (*this)[targ].getValue();
                }
            }
        }
    }
    return out;
}

// This is n*m!
void AASS::maoris::GraphZone::removeRiplesv2(int dist) {
    updateAllEdges();

    std::deque<VertexZone> top_vertex_visited;
    bool done = false;
    while (done == false) {
        VertexZone top_vertex;
        bool init = false;

        /// Find biggest vertex not visited

        std::pair<VertexIteratorZone, VertexIteratorZone> vp;
        for (vp = boost::vertices((*this)); vp.first != vp.second;) {
            VertexZone v = *vp.first;
            ++vp.first;
            bool is_visited = false;
            for (size_t j = 0; j < top_vertex_visited.size(); ++j) {
                if (v == top_vertex_visited[j]) {
                    is_visited = true;
                }
            }
            if (is_visited == false) {
                if (init == false) {
                    top_vertex = v;
                    init = true;
                } else if ((*this)[top_vertex].size() < (*this)[v].size()) {
                    top_vertex = v;
                }
            }
        }

        auto tmp_test_value = (*this)[top_vertex].getValue();

        getAllNodeRemovedRipples(top_vertex, top_vertex_visited, dist);

        auto tmp_test_value2 = (*this)[top_vertex].getValue();
        assert(tmp_test_value == tmp_test_value2);

        top_vertex_visited.push_back(top_vertex);
        // Stopping condition
        if (top_vertex_visited.size() >= getNumVertices()) {
            done = true;
        } else if (top_vertex_visited.size() > getNumVertices()) {
            throw std::runtime_error("over shoot in the remove ripples");
        }
    }
}

// TODO : would crash on self loop ?
/// Recurisve function to find all node to be fused to the original node by the
/// watershed !
void AASS::maoris::GraphZone::getAllNodeRemovedRipples(
    VertexZone& base_vertex,
    const std::deque<VertexZone>& top_vertex_visited,
    int dist) {
    EdgeIteratorZone out_i, out_end;
    int num_edge = getNumEdges(base_vertex);

    for (boost::tie(out_i, out_end) = boost::out_edges(base_vertex, (*this));
         out_i != out_end;) {
        EdgeZone e_second = *out_i;

        ++out_i;

        if ((*this)[e_second].canRemove()) {
            VertexZone targ = boost::target(e_second, (*this));

            if (isRipple(base_vertex, targ) ==
                bool is_visited = false;
                for (size_t j = 0; j < top_vertex_visited.size(); ++j) {
                if (targ == top_vertex_visited[j]) {
                    is_visited = true;
                }
                }

                try {
                removeVertexWhilePreservingEdges(targ, base_vertex, true);
                } catch (std::exception& e) {
                std::cout << "Here : " << e.what() << std::endl;
                cv::Mat graphmat2 = cv::Mat::zeros(600, 600, CV_8U);
                (*this)[targ].drawZone(graphmat2, cv::Scalar(100));
                (*this)[targ].drawContour(graphmat2, cv::Scalar(100));
                cv::imshow("fused 222", graphmat2);
                cv::waitKey(0);
                exit(0);
                }
                (*this)[base_vertex].updateContour();

                // Need to restart from the begining since the contour may have
                // changed and some that were not ripples might be now.
                boost::tie(out_i, out_end) =
                    boost::out_edges(base_vertex, (*this));
        }
    }
}

makeAllUnbreakableEdges(base_vertex);
}

void AASS::maoris::GraphZone::makeAllUnbreakableEdges(VertexZone& base_vertex) {
    EdgeIteratorZone out_i, out_end;
    // Make unbreakable edges

    for (boost::tie(out_i, out_end) = boost::out_edges(base_vertex, (*this));
         out_i != out_end;) {
        EdgeZone e_second = *out_i;
        VertexZone targ = boost::target(e_second, (*this));
        ++out_i;

        double val_base = (*this)[base_vertex].getValue();
        double val_targ = (*this)[targ].getValue();

        double min_value = std::min(val_base, val_targ);
        double max_value = std::max(val_base, val_targ);

        if ((*this)[e_second].wasRipple()) {
            if ((*this)[e_second].shouldBeUnbreakable(
                    (*this)[base_vertex].getValue(), base_vertex,
                    (*this)[targ].getValue(), targ, _threshold)) {
                (*this)[e_second].makeUnbreakable();
            }
        }
    }
}

bool AASS::maoris::GraphZone::isRipple(
    const VertexZone& base_vertex,
    const VertexZone& might_be_ripple) const {
    Zone z_ripple = (*this)[might_be_ripple];
    Zone z_base = (*this)[base_vertex];
    int nb_contact = z_ripple.contactPoint(z_base);

    // BEST FOR SKETCHMAPS
    // Check that the object is not enterely circled by the zone. i.e a windows
    // or a object in the room
    if (nb_contact >= _threshold_fusion_ripples) {
        return true;
    }

    return false;
}
