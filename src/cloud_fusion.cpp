#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
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
#include <pcl/ros/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

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

indext++;

class MapFusion {
    public:
        // Attributes
        tf::TransformListener tfListener;
        ros::NodeHandle robot1;
        ros::Subscriber sub1;
        KinectCloud::Ptr cloudOne;
        KinectCloud::Ptr cloudTwo;
        // Methods
        MapFusion();
        void filterCloud(KinectCloud::Ptr cloud, KinectCloud::Ptr cloudFiltered);
        static void streamCallbackRobot1(const sensor_msgs::PointCloud2& cloudRos);
        void streamCallbackRobot2(const sensor_msgs::PointCloud2& cloudRos);
        KinectCloud::Ptr initialAlignment(KinectCloud::Ptr cloudOne,
            KinectCloud::Ptr cloudTwo);
        KinectCloud::Ptr finalAlignment(KinectCloud::Ptr cloudOne,
            KinectCloud::Ptr cloudTwo);
        KinectNCloud::Ptr getCloudNormals(KinectCloud::Ptr cloud);
        KinectFCloud::Ptr getCloudFeatures(KinectCloud::Ptr cloud,
            KinectNCloud::Ptr cloudNormals);
};

MapFusion::MapFusion() {
    cloudOne (new KinectCloud);
    cloudTwo (new KinectCloud);
    ros::Subscriber sub1 = robot1.subscribe("/rgbdslam/new_clouds", 1000, streamCallbackRobot1);
}

// Filters a cloud using a Voxel Grid
void MapFusion::filterCloud(KinectCloud::Ptr cloud, KinectCloud::Ptr cloudFiltered) {
    pcl::VoxelGrid<KinectPoint> voxel;
    voxel.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
    voxel.setInputCloud(cloud);
    voxel.filter(*cloudFiltered);
}

// Retrieves normals from cloud.
KinectNCloud::Ptr MapFusion::getCloudNormals(KinectCloud::Ptr cloud) {
    KinectNCloud::Ptr cloudNormals (new KinectNCloud);
    KinectNormalEst estimator;
    estimator.setInputCloud(cloud);
    estimator.setRadiusSearch(RADIUS_NORMALS);
    estimator.compute(*cloudNormals);
    return cloudNormals;
}

// Retrieves features from cloud using a KDTree
KinectFCloud::Ptr MapFusion::getCloudFeatures(KinectCloud::Ptr cloud,
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
KinectCloud::Ptr MapFusion::initialAlignment(KinectCloud::Ptr cloudOne,
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
KinectCloud::Ptr MapFusion::finalAlignment(KinectCloud::Ptr cloudOne,
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

void MapFusion::streamCallbackRobot1(const sensor_msgs::PointCloud2& cloudRos) {
    pcl::PCLPointCloud2 cloudTemp;
    KinectCloud::Ptr cloudNew (new KinectCloud);
    KinectCloud::Ptr cloudTransf (new KinectCloud);
    std::string cloudFrame = cloudRos.header.frame_id;
    std::string fixedFrame = "/map";
    // Get and apply transform from camera to map
    tf::StampedTransform transform;
    Eigen::Affine3d transformEigen;
    try {
        tfListener.waitForTransform(fixedFrame, cloudFrame, ros::Time((double) cloudRos.header.stamp.toSec()), ros::Duration(3.0));
        tfListener.lookupTransform(fixedFrame, cloudFrame, ros::Time((double) cloudRos.header.stamp.toSec()), transform);
        tf::transformTFToEigen(transform, transformEigen);
        pcl::transformPointCloud(*cloudNew, *cloudTransf, transformEigen);
    }
    catch(tf::TransformException e) {
        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
    }
    if (cloudOne->points.size() == 0) {
        *cloudOne = *cloudTransf;
    }
    else {
        *cloudOne += *cloudTransf;
    }
}

void MapFusion::streamCallbackRobot2(const sensor_msgs::PointCloud2& cloudRos) {
    pcl::PCLPointCloud2 cloudTemp;
    pcl::PointCloud<pcl::PointXYZRGBA> cloudNew;
    pcl_conversions::toPCL(cloudRos, cloudTemp);
    pcl::fromPCLPointCloud2(cloudTemp, cloudNew);
    ROS_INFO("I received a point cloud from Robot 2...");
    if (cloudTwo->points.size() == 0) {
        *cloudTwo = cloudNew;
    }
    else {
        *cloudTwo += cloudNew;
    }
}

int main(int argc, char **argv) {
    // Listen to ROS topics
    indext = 0;
    ros::init(argc, argv, "listener");
    MapFusion mapFusion;
    ros::spin();
    while (indext < 3) {

    }
    pcl::io::savePCDFileASCII("test_cloud.pcd", *mapFusion.cloudOne);
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
    return 0;
}
