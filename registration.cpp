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
#define ICP_MAX_CORRESPONDANCE_DIST 0.1
#define ICP_MAX_ITERATIONS 50
#define ICP_TRANSFORMATION_EPSILON 1e-8
#define ICP_EUCLIDEAN_EPSILON 1
#define RADIUS_FEATURES 0.10
#define RADIUS_NORMALS 0.05
#define SCIA_MIN_SAMPLE_DIST 0.01
#define SCIA_MAX_CORRESPONDANCE_DIST 0.1
#define SCIA_MAX_ITERATIONS 100

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

KinectCloud::Ptr cloudOne (new KinectCloud);
KinectCloud::Ptr cloudTwo (new KinectCloud);
int index = 0;

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
KinectFCloud::Ptr getCloudFeatures(KinectCloud::Ptr cloud,
    KinectNCloud::Ptr cloudNormals) {
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
KinectCloud::Ptr initialAlignment(KinectCloud::Ptr cloudOne,
    KinectCloud::Ptr cloudTwo) {
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
        scia.setMinSampleDistance(SCIA_MIN_SAMPLE_DIST);
        scia.setMaxCorrespondenceDistance(SCIA_MAX_CORRESPONDANCE_DIST);
        scia.setMaximumIterations(SCIA_MAX_ITERATIONS);
        scia.setInputSource(cloudTwo);
        scia.setSourceFeatures(cloudTwoFeatures);
        scia.setInputTarget(cloudOne);
        scia.setTargetFeatures(cloudOneFeatures);
        scia.align(*cloudAligned);
        Eigen::Matrix4f transform = scia.getFinalTransformation();
        pcl::transformPointCloud(*cloudTwo, *cloudTransformed, transform);
        return cloudTransformed;
}

// Does second alignment of two clouds using ICP
KinectCloud::Ptr finalAlignment(KinectCloud::Ptr cloudOne,
    KinectCloud::Ptr cloudTwo) {
        KinectCloud::Ptr cloudTransformed (new KinectCloud);
        KinectCloud::Ptr cloudAligned (new KinectCloud);
        pcl::IterativeClosestPoint<KinectPoint, KinectPoint> icp;
        icp.setMaxCorrespondenceDistance(ICP_MAX_CORRESPONDANCE_DIST);
        icp.setMaximumIterations(ICP_MAX_ITERATIONS);
        icp.setTransformationEpsilon(ICP_TRANSFORMATION_EPSILON);
        icp.setEuclideanFitnessEpsilon(ICP_EUCLIDEAN_EPSILON);
        icp.setInputSource(cloudTwo);
        icp.setInputTarget(cloudOne);
        icp.align(*cloudAligned);
        Eigen::Matrix4f transform = icp.getFinalTransformation();
        pcl::transformPointCloud(*cloudTwo, *cloudTransformed, transform);
        return cloudTransformed;
}

void streamCallbackRobot1(const sensor_msgs::PointCloud2& cloud_ros) {
    pcl::PCLPointCloud2 cloud_temp;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_new;
    pcl_conversions::toPCL(cloud_ros, cloud_temp);
    pcl::fromPCLPointCloud2(cloud_temp, cloud_new);
    ROS_INFO("I received a point cloud from Robot 1...");
    if (cloudOne.size() == 0) {
        cloudOne = cloud_new;
    }
    else {
        cloudOne += cloud_new;
    }
    index++;
}

void streamCallbackRobot2(const sensor_msgs::PointCloud2& cloud_ros) {
    pcl::PCLPointCloud2 cloud_temp;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud_new;
    pcl_conversions::toPCL(cloud_ros, cloud_temp);
    pcl::fromPCLPointCloud2(cloud_temp, cloud_new);
    ROS_INFO("I received a point cloud from Robot 2...");
    pcl::io::savePCDFileASCII("test_cloud.pcd", cloud_new);
    if (cloudTwo.size() == 0) {
        cloudTwo = cloud_new;
    }
    else {
        cloudTwo += cloud_new;
    }
}

int main() {
    // Listen to ROS topics
    ros::init(argc, argv, "listener");
    ros::NodeHandle robot1;
    // ros::NodeHandle robot2;
    ros::Subscriber sub1 = robot1.subscribe("/rgbdslam/new_clouds", 1000, streamCallbackRobot1);
    // ros::Subscriber sub2 = robot2.subscribe("/rgbdslam/new_clouds", 1000, streamCallbackRobot2);
    ros::spin();

    while (index < 10) {

    }
    pcl::io::savePCDFileASCII("test_cloud.pcd", cloudOne);
    // // Declarations
    // KinectCloud::Ptr cloudOneFiltered (new KinectCloud);
    // KinectCloud::Ptr cloudTwoFiltered (new KinectCloud);
    // KinectCloud::Ptr cloudTransformed (new KinectCloud);
    //
    // pcl::io::loadPCDFile("cloud_new_1.pcd", *cloudOne);
    // pcl::io::loadPCDFile("cloud_new_2.pcd", *cloudTwo);
    //
    // // Filtering
    // filterCloud(cloudOne, cloudOneFiltered);
    // filterCloud(cloudTwo, cloudTwoFiltered);
    //
    // // Registration
    // cloudTransformed = initialAlignment(cloudOneFiltered, cloudTwoFiltered);
    // cloudTransformed = finalAlignment(cloudOneFiltered, cloudTransformed);
    //
    // // Cloud concatenation
    // *cloudOneFiltered += *cloudTransformed;
    //
    // // Visualization
    // pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //
    // // Blocks until the cloud is actually rendered
    // viewer.showCloud(cloudOneFiltered);
    // while (!viewer.wasStopped()) {
    // }
    // return 0;
}
