#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> KinectCloud;
typedef pcl::PointXYZRGBA KinectPoint;

// Filters a cloud using a Voxel Grid
void filterCloud(pcl::VoxelGrid<KinectPoint> filter, KinectCloud::Ptr cloud,
    KinectCloud::Ptr cloudFiltered) {
        filter.setInputCloud(cloud);
        filter.filter(*cloudFiltered);
}

// Initially aligns two clouds using SAC
void initialAlignment(KinectCloud cloudOne, KinectCloud cloudTwo) {

}

// Does second alignment of two clouds using ICP

void finalAlignment(pcl::IterativeClosestPoint<KinectPoint, KinectPoint> icp,
    KinectCloud::Ptr cloudOne, KinectCloud::Ptr cloudTwo, KinectCloud::Ptr cloudAligned) {
        icp.setInputSource(cloudTwo);
        icp.setInputTarget(cloudOne);
        icp.align(*cloudAligned);
}

int main () {
    // Declarations
    KinectCloud::Ptr cloudOne (new KinectCloud);
    KinectCloud::Ptr cloudTwo (new KinectCloud);
    KinectCloud::Ptr cloudOneFiltered (new KinectCloud);
    KinectCloud::Ptr cloudTwoFiltered (new KinectCloud);
    KinectCloud::Ptr cloudAligned (new KinectCloud);
    pcl::VoxelGrid<KinectPoint> voxel;
    pcl::IterativeClosestPoint<KinectPoint, KinectPoint> icp;

    // Parameterizations
    voxel.setLeafSize(0.05f, 0.05f, 0.05f);
    icp.setMaxCorrespondenceDistance (50);
    icp.setMaximumIterations (50);
    icp.setTransformationEpsilon (1e-8);
    icp.setEuclideanFitnessEpsilon (1);
    pcl::io::loadPCDFile ("cloud_new_1.pcd", *cloudOne);
    pcl::io::loadPCDFile ("cloud_new_2.pcd", *cloudTwo);

    // Filtering
    filterCloud(voxel, cloudOne, cloudOneFiltered);
    filterCloud(voxel, cloudTwo, cloudTwoFiltered);

    // Registration
    finalAlignment(cloudOneFiltered, cloudTwoFiltered, cloudAligned);

    // Cloud concatenation
    *cloudOneFiltered += *cloudAligned;

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    // Blocks until the cloud is actually rendered
    viewer.showCloud(cloudOneFiltered);
    while (!viewer.wasStopped ()) {
    }
    return 0;
}
