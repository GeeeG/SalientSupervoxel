# SalientSupervoxel
This is the C++ implementation of Saliency-guided adaptive seeding for supervoxel segmentation.

# Usage
Required dependency:
1. opencv2
2. pcl1.8

Build and run
```
mkdir build & cd build
```
```
cmake ..
```
```
make
```
Please note that you need a saliency map (salmap.png) and the corresponding point cloud (cloud.pcd) to run this algorithm. Here is an example of setting minimum and maximum seeding resolution to 0.05 and 0.5 respectively, with 3 clusters for kmeans. 
```
./ssv_test cloud.pcd salmap.png -k 3 -i 0.05 -a 0.5
```
A saliency map can be generated using "VOCUS2" (Frintrop et al., CVPR 2015), for more information refer to [Computer Vision Group @ Uni Hamburg](https://www.inf.uni-hamburg.de/en/inst/ab/cv/research/research1-visual-attention.html)

# Publication
Further details are available in our paper on the subject. If you use this code in an academic context, please cite the paper:
Ge Gao, Mikko Lauri, Jianwei Zhang and Simone Frintrop. "Saliency-guided adaptive seeding for supervoxel segmentation", IROS 2017.

BiBTeX:
```
@inproceedings{gaoetal2017ssv,
  title={Saliency-guided Adaptive Seeding for Supervoxel Segmentation},
  author={G. Gao, M. Lauri, J. Zhang and S. Frintrop},
  booktitle={IROS},
  year={2017}
}
```
