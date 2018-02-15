# MAORIS : MAp Of RIpples' Segmentation

Cutting most types of maps in smaller more meaningful bits.

## Description

This program segment maps in semantically meaningful parts, such as rooms and corridors. It has been tested on robot build maps and sketch maps, but the principle can easily be extended to other type of maps such as city maps.

![Image of a segmentation](https://raw.githubusercontent.com/MalcolmMielle/maoris/ICRA2018/Images/maoris_NLB_straighten_color.png)
![Image of another segmentation](https://raw.githubusercontent.com/MalcolmMielle/maoris/ICRA2018/Images/maoris_Freiburg101_scan_straighten_color.png)

It also provides a way to evaluate and test your own segmentation using [Matthews Correlation Coefficient](https://en.wikipedia.org/wiki/Matthews_correlation_coefficient).

A little bit more about this [here](https://malcolmmielle.wordpress.com/2017/10/05/breaking-maps-apart-in-tiny-more-meaningful-bits/).

## Paper 

The paper describing the method was accepted for publication at ICRA2017 (:D!) and should be publish soon after. The preprint version is [available on Arxiv](https://arxiv.org/abs/1709.09899).

## How to compile

### Dependencies

* boost
* openCV 2.9 or more (openCV 3 included)
* [bettergraph](https://github.com/MalcolmMielle/BetterGraph)
* [VoDiGrEx](https://github.com/MalcolmMielle/VoDiGrEx)

Compile by doing 

```
mkdir release
cmake ..
make
```
If one need to use maoris for other project you can do 
```
sudo make install
```
to install and
```
sudo make uninstall
```
To uninstall.

## How to use maoris

After compilation, you will have those executables:

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
