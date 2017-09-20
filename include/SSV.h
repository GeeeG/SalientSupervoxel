#ifndef SSV_H
#define SSV_H

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>

struct SSVOptions
{
    float spatial_distance_weight_;
    float color_weight_;
    float normal_weight_;
    float voxel_resolution_;
};

class SSV
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef pcl::PointXYZL PointLT;
    typedef pcl::PointCloud<PointLT> PointLCloudT;

    struct Cluster
    {
        std::vector<int> cloud_indices_;
        float resolution_;
    };

    SSV(const SSVOptions& opt) : opt_(opt)  { }

    void set_input(PointCloudT::ConstPtr cloud_in,
                       const std::vector<Cluster>& clusters);

    std::vector<PointLCloudT::Ptr> get_labeled_clusters() const;
    std::vector<int> num_supervoxels_per_cluster() const;
    PointCloudT::Ptr get_clustered_cloud(int cluster) const;

private:
    SSVOptions opt_;
    std::vector<PointCloudT::Ptr> cloud_clusters_;
    typedef pcl::SupervoxelClustering<PointT> VCCS;
    typedef boost::shared_ptr<VCCS> VCCSPtr;
    std::vector<VCCSPtr> vccs_;

    void setup(PointCloudT::ConstPtr cloud_in,
                              const std::vector<SSV::Cluster> &clusters);
};

#endif
