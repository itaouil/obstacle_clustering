// General imports
#include <cmath>
#include <limits>

// ROS imports
#include <ros/ros.h>
#include <tf/transform_listener.h>

// ROS messages imports
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <obstacle_clustering/Obstacle.h>
#include <obstacle_clustering/Obstacles.h>
#include <visualization_msgs/MarkerArray.h>

// PCL imports
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// Config
#include "../config/config.h"

// Global variables (init. in main)
ros::Publisher vis_pub;
ros::Publisher pub_plane;
ros::Publisher pub_filtered;
ros::Publisher obstacles_pub;
tf::TransformListener* listener;

// Callback function that receives PointCloud2
// message, segments the plane out and clusters
// the non planar points
void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert ROS message to PCL object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud_temp);

    // Filter PC
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_temp);
    sor.setFilterFieldName("z");
    sor.setFilterLimits(0.0f, 2.0f);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter(*cloud);

    static int idx = 0;
    static float coeff[4] = {0, 0, 0, 0};

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation <pcl::PointXYZRGB> seg;
    seg.setInputCloud(cloud);
    seg.setDistanceThreshold(0.02);
    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return;
    }

    coeff[0] = (coeff[0] * idx + coefficients->values[0]) / (float) (idx + 1);
    coeff[1] = (coeff[1] * idx + coefficients->values[1]) / (float) (idx + 1);
    coeff[2] = (coeff[2] * idx + coefficients->values[2]) / (float) (idx + 1);
    ++idx;

    coefficients->values[0] = coeff[0];
    coefficients->values[1] = coeff[1];
    coefficients->values[2] = coeff[2];
    coefficients->values[3] = coeff[3];

    // Color plane PC
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud <pcl::PointXYZRGB>);
    for (size_t i = 0; i < inliers->indices.size(); ++i) {
        cloud_plane->push_back(cloud->points[inliers->indices[i]]);
        cloud_plane->points.back().r = 165;
        cloud_plane->points.back().g = 165;
        cloud_plane->points.back().b = 100;
    }

    // Publish segmented plane
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_plane, cloud_msg);
    cloud_msg.header = msg->header;
    pub_plane.publish(cloud_msg);

    // Subtract inliers from input cloud
    pcl::ExtractIndices <pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZRGB>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    // Color non plane objects green
    for (size_t i = 0; i < cloud_filtered->points.size(); ++i) {
        cloud_filtered->points[i].r = 0;
        cloud_filtered->points[i].g = 255;
        cloud_filtered->points[i].b = 0;
    }

    // Publisher non-planar point cloud
    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_filtered_msg);
    cloud_filtered_msg.header = msg->header;
    pub_filtered.publish(cloud_filtered_msg);

    // Convert point cloud to robot frame
    sensor_msgs::PointCloud2 cloud_filtered_msg_robot_frame;
    pcl_ros::transformPointCloud(FRAME_ID, cloud_filtered_msg, cloud_filtered_msg_robot_frame, *listener);

    // Replace previous filtered point cloud
    // with new one in robot frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_robot_frame(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud_filtered_msg_robot_frame, *cloud_filtered_robot_frame);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered_robot_frame);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.01);
    ec.setMinClusterSize (300);
    ec.setMaxClusterSize (8000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered_robot_frame);
    ec.extract (cluster_indices);

    std::cout << "Number of points for centroid comp. : " << cloud_filtered_robot_frame->points.size() << std::endl;

    int j = 0;

    // Centroids found
    obstacle_clustering::Obstacles obstacles_msg;
    obstacles_msg.header = msg->header;
    obstacles_msg.header.frame_id = FRAME_ID;

    // Box markers for vis. purposes
    visualization_msgs::MarkerArray markers_msg;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Min/Max values for x,y and z
        const double max_value = std::numeric_limits<double>::max();
        const double min_value = std::numeric_limits<double>::min();
        double min_x = max_value, min_y = max_value, min_z = max_value;
        double max_x = min_value, max_y = min_value, max_z = min_value;

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            centroid.add(cloud_filtered_robot_frame->points[*pit]);
            cloud_cluster->points.push_back(cloud_filtered_robot_frame->points[*pit]);

            // Update max and min values for dimensions
            double x = cloud_filtered_robot_frame->points[*pit].x;
            double y = cloud_filtered_robot_frame->points[*pit].y;
            double z = cloud_filtered_robot_frame->points[*pit].z;

            // Update min/max x
            if (x < min_x) min_x = x;
            if (x > max_x) max_x = x;

            // Update min/max y
            if (y < min_y) min_y = y;
            if (y > max_y) max_y = y;

            // Update min/max z
            if (z < min_z) min_z = z;
            if (z > max_z) max_z = z;
        }

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::PointXYZ cc;
        centroid.get(cc);

        Eigen::Vector4f centroid_eigen(cc.x, cc.y, cc.z, 1.0);
        Eigen::Matrix3f covariance_matrix;
        pcl::computeCovarianceMatrix (*cloud_cluster, centroid_eigen, covariance_matrix);

        // Extract the eigenvalues and eigenvectors
        Eigen::Vector3f eigen_values;
        Eigen::Matrix3f eigen_vectors;
        pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

        Eigen::Vector3f principal_axis = eigen_vectors.col(2); // eigenvalues are sorted in ascending order -> last ev is the principal axis.
        Eigen::Vector3f normal(0, 0, 1.0); // z
        float dist = principal_axis.dot(normal);
        Eigen::Vector3f principal_axis_proj = principal_axis - dist * normal; // x
        principal_axis_proj.normalize();
        Eigen::Vector3f y_axis = normal.cross(principal_axis_proj);
        y_axis.normalize();

        Eigen::Matrix3f object_orientation;
        object_orientation.col(0) = y_axis;
        object_orientation.col(1) = principal_axis_proj;
        object_orientation.col(2) = normal;

        Eigen::Quaternionf qq(object_orientation);
        qq.normalize();

        // Populate centroids msg
        obstacle_clustering::Obstacle obstacle;
        obstacle.centroid.x = cc.x;
        obstacle.centroid.y = cc.y;
        obstacle.centroid.z = cc.z;
        obstacle.min_dimensions.x = min_x;
        obstacle.min_dimensions.y = min_y;
        obstacle.min_dimensions.z = min_z;
        obstacle.max_dimensions.x = max_x;
        obstacle.max_dimensions.y = max_y;
        obstacle.max_dimensions.z = max_z;
        obstacle.orientation.x = qq.x(); //cluster_rot.x();
        obstacle.orientation.y = qq.y(); //cluster_rot.y();
        obstacle.orientation.z = qq.z(); //cluster_rot.z();
        obstacle.orientation.w = qq.w(); //cluster_rot.w();
        obstacles_msg.obstacles.push_back(obstacle);

        // Populate markers msg
        visualization_msgs::Marker marker;
        marker.header.frame_id = FRAME_ID;
        marker.header.stamp = ros::Time();
        marker.ns = "mit_cheetah_obstacles";
        marker.id = j;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cc.x;
        marker.pose.position.y = cc.y;
        marker.pose.position.z = cc.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = std::abs(max_x - min_x);
        marker.scale.y = std::abs(max_y - min_y);
        marker.scale.z = std::abs(max_z - min_z);
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markers_msg.markers.push_back(marker);

        ROS_INFO("Cluster %d: centroid: [%f, %f, %f]; %zu data points.\n", j++, cc.x, cc.y, cc.z, centroid.getSize());
    }

    ROS_INFO("\n");

    // Publish messages
    vis_pub.publish(markers_msg);
    obstacles_pub.publish(obstacles_msg);
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "obstacles_clustering");
    ros::NodeHandle nh;

    // Initialize listener
    tf::TransformListener lr(ros::Duration(10));
    listener=&lr;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(POINT_CLOUD_TOPIC, 1, cloud_cb);

    // Publisher for segmented plane point cloud
    pub_plane = nh.advertise<pcl::PCLPointCloud2>(SEGMENTED_PLANE_TOPIC, 1);

    // Publisher for box markers that englobe clustered obstacles
    vis_pub = nh.advertise<visualization_msgs::MarkerArray>(BOX_MARKERS_TOPIC, 0);

    // Publisher for non planar clustered obstacles
    pub_filtered = nh.advertise<pcl::PCLPointCloud2>(CLUSTERED_NON_PLANAR_TOPIC, 1);

    // Publisher for custom obstacle messages that contain relevant obstacle info
    obstacles_pub = nh.advertise<obstacle_clustering::Obstacles>(OBSTACLES_INFORMATION_TOPIC, 1);

    // Spin
    ros::spin();

    return 0;
}