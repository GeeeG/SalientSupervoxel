# SalientSupervoxel
This is the C++ implementation of Saliency-guided adaptive seeding for supervoxel segmentation (SSV). 

Here's an example output of SSV comparing with baseline approach Voxel Cloud Connectivity Segmentation (VCCS) in [pcl library](http://pointclouds.org/documentation/tutorials/supervoxel_clustering.php). 

The basic idea is to grow smaller supervoxels in more salient regions and bigger supervoxels in less salient regions, by using visual saliency as prior knowledge.

Left is the input point cloud, middle is oversegmentation result with uniformed seeding with VCCS, right is oversegmentation result with saliency-guided adaptive seeding with our approach (SSV).

![ssv example](/fig/vccs_ssv_1.png?raw=true)

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

A set of .pcd and .png file example of a table scenario can be found under *example*.
```
./ssv_test table.pcd table.png -k 3 -i 0.05 -a 0.5
```
To test it on your own example, a saliency map can be generated using "VOCUS2" (Frintrop et al., CVPR 2015), you can find the instructions here [VOCUS2](https://github.com/GeeeG/VOCUS2). 
```
git clone https://github.com/GeeeG/VOCUS2.git
mkdir build & cd build
cmake . 
make
./vocus2 test_img.png
```
For more information on VOCUS2 please refer to [Computer Vision Group @ Uni Hamburg](https://www.inf.uni-hamburg.de/en/inst/ab/cv/research/research1-visual-attention.html)

# Results
SSV is evaluated using superpixel-benchmark by [Stutz et al](https://github.com/davidstutz/superpixel-benchmark) with NYUV2 by [Silberman et al](https://cs.nyu.edu/~silberman/datasets/nyu_depth_v2.html) and SUNRGBD by [Song et al](http://rgbd.cs.princeton.edu/) datasets.

Here are some results regarding boundary recall (REC) and undersegmentation error (UE) on SUNRGBD dataset.
![ssv result](/fig/results.png?raw=true)


# Performance
The implementation is tested on a 3.5 GHz Intel i7 CPU.

Average processing time for baseline approach VCCS is 0.45 second/frame

Average processing time is for SSV varies w.r.t number of k-means clusters, e.g. with K=3,4,5 processing times are 0.71, 0.58 and 0.48 second/frame respectively.

# Publication
Further details are available in our paper on the subject. If you use this code in an academic context, please cite the paper:
Ge Gao, Mikko Lauri, Jianwei Zhang and Simone Frintrop. "Saliency-guided adaptive seeding for supervoxel segmentation", IROS 2017.
[arxiv version](https://arxiv.org/abs/1704.04054)

BiBTeX:
```
@inproceedings{gaoetal2017ssv,
  title={Saliency-guided Adaptive Seeding for Supervoxel Segmentation},
  author={G. Gao, M. Lauri, J. Zhang and S. Frintrop},
  booktitle={International Conference on Intelligent Robots and Systems},
  year={2017}
}
```
