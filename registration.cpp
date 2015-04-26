#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
    
int main () {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_1_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_2_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("cloud_new_1.pcd", *cloud_1);
    pcl::io::loadPCDFile ("cloud_new_2.pcd", *cloud_2);

    // Filtering
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel;
    voxel.setInputCloud(cloud_1);
    voxel.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel.filter(*cloud_1_filtered);

    voxel.setInputCloud(cloud_2);
    voxel.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel.filter(*cloud_2_filtered);

    // Registration
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setMaxCorrespondenceDistance (50);
    icp.setMaximumIterations (50);
    icp.setTransformationEpsilon (1e-8);
    icp.setEuclideanFitnessEpsilon (1);
    icp.setInputSource(cloud_2_filtered);
    icp.setInputTarget(cloud_1_filtered);
    icp.align(*cloud_final);

    //pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGBA, pcl::PointXYZRGBA> reg;
    //reg.setTransformationEpsilon(1e-6);
    //reg.setMaxCorrespondenceDistance(0.1);
    //reg.setMaximumIterations(30);
    //reg.setInputSource(cloud_2_filtered);
    //reg.setInputTarget(cloud_1_filtered);
    //reg.align(*cloud_final);

    // Cloud concatenation
    *cloud_1_filtered += *cloud_final;

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    // Blocks until the cloud is actually rendered
    viewer.showCloud(cloud_1_filtered);
    while (!viewer.wasStopped ()) {
    }
    return 0;
}
