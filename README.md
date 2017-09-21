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
To use the toy example cloud, set minimum and maximum seeding resolution to 0.05 and 0.5 respectively, with 3 clusters for kmeans
```
./ssv_test 1.pcd 1.png -k 3 -i 0.05 -a 0.5
```

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
