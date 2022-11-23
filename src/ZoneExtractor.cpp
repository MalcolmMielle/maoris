#include "ZoneExtractor.hpp"

void AASS::maoris::ZoneExtractor::extract(cv::Mat& in) {
    // Make zones
    makeZones(in);
    // Fuse zones with close value
    fuse();
    createGraph();
}

/*Two way : I can either swype one way creating zone
 * OR
 * I can for every value only keep a certain value -> FloodFill all spaces while
 * marking the pixel that disappeared as a zone
 */
void AASS::maoris::ZoneExtractor::makeZones(cv::Mat& input) {
    cv::Mat in;
    input.convertTo(in, CV_32SC1);
    // Mat with 0 when pixel has not been seen or is a wall and a number when it
    // belong to a zone
    cv::Mat zones_star = cv::Mat::ones(in.rows, in.cols, in.depth());
    zones_star = -zones_star;
    _index_of_zones_to_fuse_after.clear();
    _graph.clear();

    // Swype in one way and add pixel to zones
    int step = 0;
    for (int row = 0; row < in.rows; row++) {
        int* p = in.ptr<int>(row);
        int* p_zone_star = zones_star.ptr<int>(row);
        for (int col = 0; col < in.cols; col++) {
            assert(_zones.size() <= in.rows * in.cols);
            ++step;

            // If the pixel is not a wall
            if (_values_to_ignore.count(p[col]) == 0) {
                std::vector<unsigned int> zone_index;
                std::vector<unsigned int> zone_edges;

                isolatedOrNot(p[col], in, zones_star, row, col, zone_index,
                              zone_edges);

                // New zone since the pixel is connected to no  already seen +
                // same value pixel
                if (zone_index.size() == 0) {
                    Zone new_zone(input.size());
                    new_zone.setValue(p[col]);
                    new_zone.push_back(cv::Point2i(row, col));
                    _zones.push_back(new_zone);
                    p_zone_star[col] = (int)_zones.size() - 1;
                }
                // If the pixel is part of (an) already seen zone(s)
                else {
                    _zones[zone_index[0]].push_back(cv::Point2i(row, col));
                    p_zone_star[col] = zone_index[0];

                    if (_zones[zone_index[0]].getValue() != p[col]) {
                        throw std::runtime_error(
                            "Different value between the place and the zone "
                            "it's added to");
                    }

                    // If more than one zone push the value of other zones to
                    // _index_of_zones_to_fuse_after to fuse them later
                    if (zone_index.size() > 1) {
                        bool flag_exist = false;
                        for (size_t i = 1; i < zone_index.size(); ++i) {
                            // Sort them by index value
                            size_t min = std::min(zone_index[0], zone_index[i]);
                            size_t max = std::max(zone_index[0], zone_index[i]);
                            if (_index_of_zones_to_fuse_after_set.count(
                                    std::pair<size_t, size_t>(min, max)) == 0) {
                                // 							if(flag_exist
                                // == false){
                                _index_of_zones_to_fuse_after.push_back(
                                    std::pair<size_t, size_t>(min, max));
                                _index_of_zones_to_fuse_after_set.insert(
                                    std::pair<size_t, size_t>(min, max));
                            }
                        }
                    }
                }
                // Adding edges if needs be done
                if (zone_edges.size() > 0) {
                    bool flag_exist = false;
                    for (size_t i = 0; i < zone_edges.size(); ++i) {
                        // Sort them by index value

                        unsigned int min = std::min(
                            (unsigned int)p_zone_star[col], zone_edges[i]);
                        unsigned int max = std::max(
                            (unsigned int)p_zone_star[col], zone_edges[i]);

                        if (_index_of_edges_set.count(
                                std::pair<size_t, size_t>(min, max)) == 0) {
                            _index_of_edges_set.insert(
                                std::pair<size_t, size_t>(min, max));
                            _index_of_edges.push_back(
                                std::pair<size_t, size_t>(min, max));
                        }
                    }
                }
            }
        }
    }

    std::cout << "Zone made" << std::endl;
}

void AASS::maoris::ZoneExtractor::isolatedOrNot(
    int value,
    cv::Mat& input,
    cv::Mat& zones_star,
    int row,
    int col,
    std::vector<unsigned int>& zone_index,
    std::vector<unsigned int>& zone_edges) {
    if (zones_star.size() != input.size()) {
        throw std::runtime_error("Zone start and input of different sizes");
    }

    for (int row_tmp = row - 1; row_tmp <= row + 1; ++row_tmp) {
        for (int col_tmp = col - 1; col_tmp <= col + 1; ++col_tmp) {
            // Inside mat
            if (row_tmp >= 0 && col_tmp >= 0 && row_tmp < input.rows &&
                col_tmp < input.cols) {
                // Not the center point
                if (row != row_tmp || col != col_tmp) {
                    // Not the diagonal
                    if (row_tmp == row || col_tmp == col) {
                        int* p = input.ptr<int>(row_tmp);
                        int* p_star = zones_star.ptr<int>(row_tmp);
                        // Same value and visited before
                        if (value == p[col_tmp] && p_star[col_tmp] >= 0) {
                            if (_zones[p_star[col_tmp]].getValue() != value) {
                                printZone(p_star[col_tmp]);
                                throw std::runtime_error(
                                    "Value and zone mismatch");
                            }
                            // SAME ZONE
                            bool flag_seen = false;
                            for (size_t i = 0; i < zone_index.size(); ++i) {
                                if (zone_index[i] == p_star[col_tmp]) {
                                    flag_seen = true;
                                }
                            }
                            if (flag_seen == false) {
                                zone_index.push_back(p_star[col_tmp]);
                            }
                        }

                        // Check for other value bordering
                        else if (value != p[col_tmp] && p_star[col_tmp] >= 0) {
                            if (_zones[p_star[col_tmp]].getValue() == value) {
                                std::cout << "At : row col " << row_tmp << " "
                                          << col_tmp << std::endl;
                                throw std::runtime_error(
                                    "Value and zone are the same in edges");
                            }
                            // SAME ZONE
                            bool flag_seen = false;
                            for (size_t i = 0; i < zone_edges.size(); ++i) {
                                if (zone_edges[i] == p_star[col_tmp]) {
                                    flag_seen = true;
                                }
                            }
                            if (flag_seen == false) {
                                zone_edges.push_back(p_star[col_tmp]);
                            }
                        }

                        else if (p_star[col_tmp] >= 0) {
                            std::runtime_error(
                                "Something should have happenéd and it didn't. "
                                "We found a value but it was not linked at the "
                                "same or an edge");
                        }
                    }
                }
            }
        }
    }
}

void AASS::maoris::ZoneExtractor::fuse() {
    std::cout << "Fuse" << std::endl;
    for (size_t i = 0; i < _zones.size(); ++i) {
        if (_zones[i].isEmpty()) {
            print();
            throw std::runtime_error(
                "The zone is empty and it shouldn't happen");
        }
    }

    initMappingAlive();
    // Sort the index backward from higest value for second so that we erase the
    // zone from the back without changing the indexes.
    std::sort(_index_of_zones_to_fuse_after.begin(),
              _index_of_zones_to_fuse_after.end(), sortFunction);

    _mapping.clear();
    // Create the full mapping
    for (size_t i = 0; i < _index_of_zones_to_fuse_after.size(); ++i) {
        //
        int base;
        int to_fuse;
        // Search the value in the mapping and update it
        if (_mapping.find(_index_of_zones_to_fuse_after[i].first) ==
            _mapping.end()) {
            // not found
            base = _index_of_zones_to_fuse_after[i].first;
        } else {
            // found
            base = _mapping[_index_of_zones_to_fuse_after[i].first];
        }
        if (_mapping.find(_index_of_zones_to_fuse_after[i].second) ==
            _mapping.end()) {
            // not found
            to_fuse = _index_of_zones_to_fuse_after[i].second;
        } else {
            // found
            to_fuse = _mapping[_index_of_zones_to_fuse_after[i].second];
        }

        // Always copy to the smallest number
        int max = to_fuse;
        int min = base;
        if (min != max) {
            if (max < min) {
                max = base;
                min = to_fuse;

                updateMapping(max, min);
                // 				_mapping[max] = min;
                _mapping[_index_of_zones_to_fuse_after[i].first] = min;
            } else {
                updateMapping(max, min);
                _mapping[_index_of_zones_to_fuse_after[i].second] = min;
            }

            if (_zones[base].getValue() != _zones[to_fuse].getValue() ||
                base == to_fuse) {
                std::cout << min << " " << max << std::endl;
                std::cout << _zones[base].getValue() << " "
                          << _zones[to_fuse].getValue() << std::endl;
                std::ostringstream str_test;
                str_test << "Fusing zone that shouldn't be fused at line "
                         << __LINE__ << " in file " << __FILE__ << "."
                         << std::endl;
                throw std::runtime_error(str_test.str());
            }
        }
    }

    std::map<size_t, size_t>::reverse_iterator iter;
    for (iter = _mapping.rbegin(); iter != _mapping.rend(); ++iter) {
        for (size_t j = 0; j < _zones[iter->first].size(); ++j) {
            _zones[iter->second].push_back(_zones[iter->first][j]);
        }
        _zones.erase(_zones.begin() + iter->first);
    }
    std::cout << "Done fusing" << std::endl;
}

void AASS::maoris::ZoneExtractor::createGraph() {
    std::map<size_t, int>::iterator iter;

    std::vector<VertexZone> vertices_zones;

    // Adding all vertices
    for (size_t i = 0; i < _zones.size(); ++i) {
        VertexZone v;
        if (_zones[i].isEmpty()) {
            throw std::runtime_error(
                "The zone is empty and it shouldn't happen");
        }
        _graph.addVertex(v, _zones[i]);
        vertices_zones.push_back(v);
    }

    // Adding all edge
    for (size_t i = 0; i < _index_of_edges.size(); ++i) {
        int base;
        int destination;
        // Search the value in the mapping and update it
        if (_mapping.find(_index_of_edges[i].first) == _mapping.end()) {
            base = _index_of_edges[i].first;
        } else {
            base = _mapping[_index_of_edges[i].first];
        }
        if (_mapping.find(_index_of_edges[i].second) == _mapping.end()) {
            destination = _index_of_edges[i].second;
        } else {
            destination = _mapping[_index_of_edges[i].second];
        }
        EdgeZone e;
        if (_mapping_of_node_alive[base] > 0) {
            base = _mapping_of_node_alive[base];
        }
        if (_mapping_of_node_alive[destination] > 0) {
            destination = _mapping_of_node_alive[destination];
        }
        _graph.addEdge(e, vertices_zones[base], vertices_zones[destination]);
    }

    if (_graph.getNumVertices() != _zones.size()) {
        throw std::runtime_error("Graph and zone not of same dimensions");
    }
}
