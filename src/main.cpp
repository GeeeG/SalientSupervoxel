#include "SSV.h"
#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <boost/timer.hpp>

struct cluster_t
{
    cluster_t(int index, float avg_saliency) : index_(index), avg_saliency_(avg_saliency) {}
    bool operator<(const cluster_t& other) const { return avg_saliency_ < other.avg_saliency_; }
    int index_;
    float avg_saliency_;
};

int main(int argc, char** argv)
{
    SSV::PointCloudT::Ptr cloud = boost::shared_ptr <SSV::PointCloudT> (new SSV::PointCloudT ());
    pcl::console::print_highlight ("Loading point cloud...\n");
    if (pcl::io::loadPCDFile<SSV::PointT> (argv[1], *cloud))
    {
        pcl::console::print_error ("Error loading cloud file!\n");
        return (1);
    }

    char* imageName = argv[2];
    cv::Mat salmap;
    salmap = cv::imread(imageName);
    if(!salmap.data)
    {
        pcl::console::print_error ("Error loading saliency maps!\n");
        return (1);
    }

    float voxel_resolution = 0.008f;
    bool voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
    if (voxel_res_specified)
      pcl::console::parse(argc, argv, "-v", voxel_resolution);

    float kmeans_num_cluster = 2;
    bool kmeans_num_cluster_specified = pcl::console::find_switch (argc, argv, "-k");
    if (kmeans_num_cluster_specified)
      pcl::console::parse(argc, argv, "-k", kmeans_num_cluster);

    float min_seed_resolution = 0.1f;
    bool min_seed_resolution_specified = pcl::console::find_switch (argc, argv, "-i");
    if (min_seed_resolution_specified)
      pcl::console::parse(argc, argv, "-i", min_seed_resolution);

    float max_seed_resolution = 0.5f;
    bool max_seed_resolution_specified = pcl::console::find_switch (argc, argv, "-a");
    if (max_seed_resolution_specified)
      pcl::console::parse(argc, argv, "-a", max_seed_resolution);

    float spatial_weight = 0.15f;
    if (pcl::console::find_switch (argc, argv, "-z"))
      pcl::console::parse(argc, argv, "-z", spatial_weight);

    float normal_weight = 0.15f;
    if (pcl::console::find_switch (argc, argv, "-n"))
      pcl::console::parse(argc, argv, "-n", normal_weight);


    cv::Mat bgr[3];
    cv::split(salmap, bgr);
    cv::Mat salmap_im = bgr[0];
    salmap_im.convertTo(salmap_im, CV_32F);

    int attempts = 5;
    cv::Mat labels;
    cv::Mat avg_saliency;
    cv::kmeans(salmap_im.reshape(0,1).t(),
               kmeans_num_cluster,
               labels,
               cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001),
               attempts,
               cv::KMEANS_PP_CENTERS,
               avg_saliency );

    std::vector<cluster_t> clusterdata(kmeans_num_cluster, cluster_t(0,0.0f));
    for (std::size_t c = 0; c < kmeans_num_cluster; ++c)
        clusterdata[c] = cluster_t(c, avg_saliency.at<float>(c));

    std::sort(clusterdata.begin(), clusterdata.end()); // sorts according to increasing average saliency

    // assign indices to clusters
    std::vector<SSV::Cluster> ssv_clusters(kmeans_num_cluster);
    for (int i = 0; i < cloud->size(); ++i)
    {
        int label = labels.at<int>(i);
        int cluster_idx = clusterdata[label].index_;
        ssv_clusters[cluster_idx].cloud_indices_.push_back(i);
    }

    // assign cluster resolutions
    if (kmeans_num_cluster == 1)
    {
        ssv_clusters[0].resolution_ = min_seed_resolution;
    }
    else
    {
    float step_size = (log10(min_seed_resolution)-log10(max_seed_resolution))/ static_cast<float>(kmeans_num_cluster-1);
        for (std::size_t c = 0; c < kmeans_num_cluster; ++c)
        {
            ssv_clusters[c].resolution_ = pow(10.0f, (log10(max_seed_resolution) + static_cast<float>(c)*step_size) );
        }
    }

    SSVOptions ssv_opt;
    ssv_opt.normal_weight_ = normal_weight;
    ssv_opt.spatial_distance_weight_ = spatial_weight;
    ssv_opt.color_weight_ = 1.0f - (normal_weight + spatial_weight);
    ssv_opt.voxel_resolution_ = voxel_resolution;

    pcl::console::print_highlight ("SSV Generating supervoxels...\n");
    boost::timer timer_ssv;
    SSV ssv(ssv_opt);
    ssv.set_input(cloud, ssv_clusters);

    // add clusters together
    std::vector<SSV::PointLCloudT::Ptr> labeled_clusters = ssv.get_labeled_clusters();
    SSV::PointLCloudT::Ptr result( new SSV::PointLCloudT() );

    std::vector<int> num_sv = ssv.num_supervoxels_per_cluster();
    int num_labels = 0;
    for (std::size_t c = 0; c < labeled_clusters.size(); ++c)
    {
        for (auto& p : *labeled_clusters[c])
            p.label += num_labels;

        *result += *labeled_clusters[c];
        num_labels += num_sv[c];
    }
    float elapsed = timer_ssv.elapsed();
    std::cout << "SSV Time elapsed: " << elapsed << " seconds.\n";

    // original vccs
    pcl::console::print_highlight ("VCCS Generating supervoxels...\n");
    boost::timer timer_vccs;
    pcl::SupervoxelClustering<SSV::PointT> super (voxel_resolution, min_seed_resolution);
    super.setInputCloud (cloud);
    super.setColorImportance (ssv_opt.color_weight_);
    super.setSpatialImportance (ssv_opt.spatial_distance_weight_);
    super.setNormalImportance (ssv_opt.normal_weight_);

     std::map <uint32_t, pcl::Supervoxel<SSV::PointT>::Ptr > supervoxel_clusters;

     super.extract (supervoxel_clusters);
     SSV::PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
     float elapsed_vccs = timer_vccs.elapsed();
     std::cout << "VCCS Time elapsed: " << elapsed_vccs << " seconds.\n";

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters();

    // Create all the viewports
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0.0, 0.0, 0.0, v1);

    int v2(0);
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.0, 0.0, 0.0, v2);

//    int v3(0);
//    viewer->createViewPort (0.0, 0.0, 0.5, 0.5, v3);
//    viewer->setBackgroundColor (0.0, 0.0, 0.0, v3);

//    int v4(0);
//    viewer->createViewPort (0.5, 0.0, 1.0, 0.5, v4);
//    viewer->setBackgroundColor (0.0, 0.0, 0.0, v4);

    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud(cloud, "point cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1.0, "point cloud");

    viewer->addPointCloud(labeled_voxel_cloud, "labeled cloud",v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1.0, "labeled cloud");

//    viewer->addPointCloud(ssv.get_clustered_cloud(0), "clustered_cloud",v3);
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1.0, "clustered_cloud");

//    viewer->addPointCloud(result, "result",v4);
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1.0, "result");

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }


    return 0;
}
