#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>

#define VOXEL_LEAF_SIZE 0.05f
#define ICP_MAX_CORRESPONDANCE_DIST 50
#define ICP_MAX_ITERATIONS 50
#define ICP_TRANSFORMATION_EPSILON 1e-8
#define ICP_EUCLIDEAN_EPSILON 1
#define RADIUS_FEATURES 0.05
#define RADIUS_NORMALS 0.05
#define SCIA_MAX_ITERATIONS 50

typedef pcl::PointXYZRGBA KinectPoint;
typedef pcl::FPFHSignature33 KinectFeature;
typedef pcl::PointCloud<KinectPoint> KinectCloud;
typedef pcl::PointCloud<KinectFeature> KinectFCloud;
typedef pcl::PointCloud<pcl::Normal> KinectNCloud;
typedef pcl::search::KdTree<KinectPoint> KinectKdTree;
typedef pcl::NormalEstimation<KinectPoint, pcl::Normal> KinectNormalEst;
typedef pcl::FPFHEstimation<KinectPoint, pcl::Normal,
    KinectFeature> KinectFeatureEst;
typedef pcl::SampleConsensusInitialAlignment<KinectPoint, KinectPoint,
    KinectFeature> KinectSCIA;

// Filters a cloud using a Voxel Grid
void filterCloud(KinectCloud::Ptr cloud, KinectCloud::Ptr cloudFiltered) {
    pcl::VoxelGrid<KinectPoint> voxel;
    voxel.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
    voxel.setInputCloud(cloud);
    voxel.filter(*cloudFiltered);
}

// Retrieves normals from cloud.
KinectNCloud::Ptr getCloudNormals(KinectCloud::Ptr cloud) {
    KinectNCloud::Ptr cloudNormals (new KinectNCloud);
    KinectNormalEst estimator;
    estimator.setInputCloud(cloud);
    estimator.setRadiusSearch(RADIUS_NORMALS);
    estimator.compute(*cloudNormals);
    return cloudNormals;
}

// Retrieves features from cloud using a KDTree
KinectFCloud::Ptr getCloudFeatures(KinectCloud::Ptr cloud, KinectNCloud::Ptr cloudNormals) {
    KinectFCloud::Ptr cloudFeatures (new KinectFCloud);
    KinectKdTree::Ptr searchMethod (new KinectKdTree);
    KinectFeatureEst estimator;
    estimator.setInputCloud(cloud);
    estimator.setInputNormals(cloudNormals);
    estimator.setSearchMethod(searchMethod);
    estimator.setRadiusSearch(RADIUS_FEATURES);
    estimator.compute(*cloudFeatures);
    return cloudFeatures;
}

// Initially aligns two clouds using SAC
KinectCloud::Ptr initialAlignment(KinectCloud::Ptr cloudOne, KinectCloud::Ptr cloudTwo) {
    KinectCloud::Ptr cloudTransformed (new KinectCloud);
    KinectCloud::Ptr cloudAligned (new KinectCloud);
    KinectSCIA scia;
    KinectNCloud::Ptr cloudOneNormals
        = getCloudNormals(cloudOne);
    KinectNCloud::Ptr cloudTwoNormals
        = getCloudNormals(cloudTwo);
    KinectFCloud::Ptr cloudOneFeatures
        = getCloudFeatures(cloudOne, cloudOneNormals);
    KinectFCloud::Ptr cloudTwoFeatures
        = getCloudFeatures(cloudTwo, cloudTwoNormals);
    scia.setMaximumIterations(SCIA_MAX_ITERATIONS);
    scia.setInputSource(cloudOne);
    scia.setSourceFeatures(cloudOneFeatures);
    scia.setInputTarget(cloudTwo);
    scia.setTargetFeatures(cloudTwoFeatures);
    scia.align(*cloudAligned);
    Eigen::Matrix4f transform = scia.getFinalTransformation();
    pcl::transformPointCloud(*cloudTwo, *cloudTransformed, transform);
    return cloudTransformed;
}

// Does second alignment of two clouds using ICP
void finalAlignment(KinectCloud::Ptr cloudOne, KinectCloud::Ptr cloudTwo,
    KinectCloud::Ptr cloudAligned) {
        pcl::IterativeClosestPoint<KinectPoint, KinectPoint> icp;
        icp.setMaxCorrespondenceDistance(ICP_MAX_CORRESPONDANCE_DIST);
        icp.setMaximumIterations(ICP_MAX_ITERATIONS);
        icp.setTransformationEpsilon(ICP_TRANSFORMATION_EPSILON);
        icp.setEuclideanFitnessEpsilon(ICP_EUCLIDEAN_EPSILON);
        icp.setInputSource(cloudTwo);
        icp.setInputTarget(cloudOne);
        icp.align(*cloudAligned);
}

int main() {
    // Declarations
    KinectCloud::Ptr cloudOne (new KinectCloud);
    KinectCloud::Ptr cloudTwo (new KinectCloud);
    KinectCloud::Ptr cloudOneFiltered (new KinectCloud);
    KinectCloud::Ptr cloudTwoFiltered (new KinectCloud);
    KinectCloud::Ptr cloudTransformed (new KinectCloud);

    pcl::io::loadPCDFile("cloud_new_1.pcd", *cloudOne);
    pcl::io::loadPCDFile("cloud_new_2.pcd", *cloudTwo);

    // Filtering
    filterCloud(cloudOne, cloudOneFiltered);
    filterCloud(cloudTwo, cloudTwoFiltered);

    // Registration
    cloudTransformed = initialAlignment(cloudOneFiltered, cloudTwoFiltered);
    // finalAlignment(cloudOneFiltered, cloudAligned, cloudAligned);

    // Cloud concatenation
    *cloudOneFiltered += *cloudTransformed;

    // Visualization
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    // Blocks until the cloud is actually rendered
    viewer.showCloud(cloudOneFiltered);
    while (!viewer.wasStopped ()) {
    }
    return 0;
}
