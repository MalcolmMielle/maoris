#add_library(Fuzzylib
#			FuzzyOpening.cpp ZoneExtractor.cpp Kmean.cpp GraphZone.cpp
#			)

add_subdirectory(Evaluation)

add_library(MAORIS
			FuzzyOpening.cpp ZoneExtractor.cpp Kmean.cpp GraphZone.cpp Zone.cpp Segmentor.cpp
			)


INSTALL(TARGETS MAORIS DESTINATION lib/RSI)

add_executable(segmentation_example segmentation_example.cpp)
target_link_libraries(segmentation_example ${OpenCV_LIBS} ${vodigrex_LIBRARIES} MAORIS -lboost_unit_test_framework -lboost_filesystem -lboost_system)

add_executable(evaluation_gt evaluation_gt.cpp)
target_link_libraries(evaluation_gt ${OpenCV_LIBS} ${vodigrex_LIBRARIES} MAORIS Evaluation -lboost_unit_test_framework -lboost_filesystem -lboost_system)

add_executable(evaluation_gt_all_files evaluation_gt_all_files.cpp)
target_link_libraries(evaluation_gt_all_files ${OpenCV_LIBS} ${vodigrex_LIBRARIES} MAORIS Evaluation -lboost_filesystem -lboost_system)


add_executable(evaluation_gt_param evaluation_gt_param.cpp)
target_link_libraries(evaluation_gt_param ${OpenCV_LIBS} ${vodigrex_LIBRARIES} MAORIS Evaluation -lboost_filesystem -lboost_system)


add_executable(evaluation_2gt evaluation_2gt.cpp)
target_link_libraries(evaluation_2gt ${OpenCV_LIBS} ${vodigrex_LIBRARIES} MAORIS Evaluation -lboost_unit_test_framework -lboost_filesystem -lboost_system)

add_executable(evaluation_2files_folder evaluation_2files_folder.cpp)
target_link_libraries(evaluation_2files_folder ${OpenCV_LIBS} ${vodigrex_LIBRARIES} MAORIS Evaluation -lboost_unit_test_framework -lboost_filesystem -lboost_system)

# add_executable(thermal_segmentation thermal_segmentation.cpp)
# target_link_libraries(thermal_segmentation ${OpenCV_LIBS} ${vodigrex_LIBRARIES} MAORIS -lboost_unit_test_framework -lboost_filesystem -lboost_system hungarian_lib)

