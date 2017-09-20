#include "SSV.h"

void SSV::set_input(PointCloudT::ConstPtr cloud_in,
                    const std::vector<SSV::Cluster>& clusters)
{
    setup(cloud_in, clusters);
    for ( std::size_t c = 0; c < clusters.size(); ++c)
    {
        vccs_[c]->setInputCloud(cloud_clusters_[c]);
        vccs_[c]->setColorImportance(opt_.color_weight_);
        vccs_[c]->setSpatialImportance(opt_.spatial_distance_weight_);
        vccs_[c]->setNormalImportance(opt_.normal_weight_);
    }
}

std::vector<SSV::PointLCloudT::Ptr> SSV::get_labeled_clusters() const
{
    std::vector<PointLCloudT::Ptr> labeled_clouds( cloud_clusters_.size() );
    for (std::size_t c = 0; c < cloud_clusters_.size(); ++c)
    {
        std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxels;
        vccs_[c]->extract(supervoxels);
        labeled_clouds[c] = vccs_[c]->getLabeledCloud();
    }
    return labeled_clouds;
}

std::vector<int> SSV::num_supervoxels_per_cluster() const
{
    std::vector<int> num_sv;
    for (auto& v : vccs_)
    {
        num_sv.push_back( v->getMaxLabel() );
    }
    return num_sv;
}

SSV::PointCloudT::Ptr SSV::get_clustered_cloud(int cluster) const
{
    return cloud_clusters_.at(cluster);
}

void SSV::setup(PointCloudT::ConstPtr cloud_in,
                const std::vector<SSV::Cluster>& clusters)
{
    cloud_clusters_.clear();
    vccs_.clear();
    std::size_t num_clusters = clusters.size();
    for (std::size_t c = 0; c < num_clusters; ++c)
    {
        cloud_clusters_.push_back( PointCloudT::Ptr(
                                       new PointCloudT(*cloud_in, clusters[c].cloud_indices_)
                                       ) );
        vccs_.push_back( VCCSPtr( new VCCS(opt_.voxel_resolution_, clusters[c].resolution_) ) );
    }
}
