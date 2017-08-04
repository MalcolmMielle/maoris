# add_executable(test_function_fuzzy test_function_fuzzy.cpp)
# target_link_libraries(test_function_fuzzy ${OpenCV_LIBS} RSILIB -lboost_unit_test_framework)
# 
# add_executable(test_watershed test_watershed.cpp)
# target_link_libraries(test_watershed ${OpenCV_LIBS} RSILIB -lboost_unit_test_framework)
# 
# add_executable(test_kmeans test_kmeans.cpp)
# target_link_libraries(test_kmeans ${OpenCV_LIBS} RSILIB -lboost_unit_test_framework)
# 
# add_executable(test_sketchify test_sketchify.cpp)
# target_link_libraries(test_sketchify ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB -lboost_unit_test_framework)
# 
# add_executable(test_graph test_graph.cpp)
# target_link_libraries(test_graph ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB -lboost_unit_test_framework)
# 
# add_executable(test_histogram test_histogram.cpp)
# target_link_libraries(test_histogram ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB -lboost_unit_test_framework)
# 
# add_executable(test_zonegraph test_zonegraph.cpp)
# target_link_libraries(test_zonegraph ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB -lboost_unit_test_framework)
# 
# add_executable(test_edit test_edit.cpp)
# target_link_libraries(test_edit ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB -lboost_unit_test_framework)
# 
# add_executable(test_zonereduce test_zonereduce.cpp)
# target_link_libraries(test_zonereduce ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB -lboost_unit_test_framework)
# 
# add_executable(test_pca test_pca.cpp)
# target_link_libraries(test_pca ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB -lboost_unit_test_framework -lboost_filesystem -lboost_system)

add_executable(test_collisionblob test_collisionblob.cpp)
target_link_libraries(test_collisionblob ${OpenCV_LIBS} ${vodigrex_LIBRARIES} MAORIS -lboost_unit_test_framework)

# add_executable(test_comparezone test_comparezone.cpp)
# target_link_libraries(test_comparezone ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB -lboost_unit_test_framework)
# 
# add_executable(harris_opencv harris_opencv.cpp)
# target_link_libraries(harris_opencv ${OpenCV_LIBS})
# 
# add_executable(goodFeatTrack goodFeatTrack.cpp)
# target_link_libraries(goodFeatTrack ${OpenCV_LIBS})

# add_executable(hungarian_test hungarian_test.c)
# target_link_libraries(hungarian_test hungarian_lib)
# 
# add_executable(test_zone_hungarian_itself test_zone_hungarian_itself.cpp)
# target_link_libraries(test_zone_hungarian_itself ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB -lboost_unit_test_framework -lboost_filesystem -lboost_system hungarian_lib)
# 
# 
# add_executable(test_zone_hungarian test_zone_hungarian.cpp)
# target_link_libraries(test_zone_hungarian ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB -lboost_unit_test_framework -lboost_filesystem -lboost_system hungarian_lib)


# add_executable(test_valgrind_errors test_valgrind_errors.cpp)
# target_link_libraries(test_valgrind_errors ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB hungarian_lib)
# 
# add_executable(test_slam_hungarian test_slam_maps_hungarian.cpp)
# target_link_libraries(test_slam_hungarian ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB -lboost_unit_test_framework -lboost_filesystem -lboost_system hungarian_lib)
# # 
# add_executable(test_uniqueness test_uniqueness.cpp)
# target_link_libraries(test_uniqueness ${OpenCV_LIBS} ${vodigrex_LIBRARIES} RSILIB -lboost_unit_test_framework -lboost_filesystem -lboost_system hungarian_lib)

add_subdirectory(stats_python)