# MAORIS : MAp Of RIpples' Segmentation

Cutting most types of maps in smaller meaningful bits.

## Description

This program segment maps in semantically meaningful parts, such as rooms and corridors. It has been tested on robot build maps and sketch maps, but the principle can easily be extended to other type of maps such as city maps.

![Image of a segmentation](//https://github.com/MalcolmMielle/maoris/Images/maoris_NLB_straighten_color.png)
![Image of another segmentation](//https://github.com/MalcolmMielle/maoris/Images/maoris_Freiburg101_scan_straighten_color.png)

It also provides a way to evaluate and test your own segmentation using [Matthews Correlation Coefficient](https://en.wikipedia.org/wiki/Matthews_correlation_coefficient).

## Paper 

The paper describing the method has been submitted to ICRA2017 and is waiting for reviewing. It should soon be available on Arxiv.

## How to use

Compile by doing 

```
mkdir release
cmake ..
make
```

Then you will have those executables:

### For segmentation:

* evaluation_2files_fodler : compare segmented images files from two folders

* evaluation_gt : compare a map with a segmented ground truth image

### For evaluation

* evaluation_2gt : compare two segmented images
 
* evaluation_gt_all_files : segment all maps in a folder and then compare them to all segmented ground truths in another folder
This file returns a gnuplot dat file named `maoris_all_measures.dat` with all the data from the segmentation.

* evaluation_gt_param : evaluate maoris' parameters
This file returns four gnuplot dat files named `maoris_param_threshold.dat`, `maoris_param_margin.dat`, `maoris_param_ripples.dat`, and `maoris_param_doors.dat`, with all the data from the segmentation for each parameters.

The arguments of all programs are given in the command line as in `evaluation_gt_all_files folder1 folder2 should_draw`.

## Evaluating your own segmentations

You can evaluate your own segmentation results to ground truths by doing:

```
evaluation_2files_fodler folder_with_your_segmentation folder_with_your_ground_truth
```

We also provide segmentation by maoris, DuDe and a Voronoi based segmentation. DuDe segmentation results are in `Test/RobotMap/DuDe_results` for the robot maps and `Test/Sketches/DuDe` for the sketch maps. The Voronoi based segmentation results are in `Test/RobotMap/Voronoi` for the robot maps and `Test/Sketches/Voronoi` for the sketch maps. maoris segmentation results are in `Test/RobotMap/maoris` for the robot maps and `Test/Sketches/maoris` for the sketch maps.
