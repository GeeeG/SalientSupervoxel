# SalientSupervoxel
This is the C++ implementation of Saliency-guided adaptive seeding for supervoxel segmentation (SSV). 

Here's an example output of SSV comparing with baseline approach Voxel Cloud Connectivity Segmentation (VCCS) [pcl library](http://pointclouds.org/documentation/tutorials/supervoxel_clustering.php). Left is the input point cloud, middle is oversegmentation result with uniformed seeding with VCCS, right is oversegmentation result with saliency guided seeding with our approach (SSV).

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
```
./ssv_test cloud.pcd salmap.png -k 3 -i 0.05 -a 0.5
```
A saliency map can be generated using "VOCUS2" (Frintrop et al., CVPR 2015), for more information refer to [Computer Vision Group @ Uni Hamburg](https://www.inf.uni-hamburg.de/en/inst/ab/cv/research/research1-visual-attention.html)

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
  booktitle={IROS},
  year={2017}
}
```
