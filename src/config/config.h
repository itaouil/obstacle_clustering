#ifndef OBSTACLE_CLUSTERING_CONFIG_H
#define OBSTACLE_CLUSTERING_CONFIG_H

#include <utility>
#include <iostream>

// Frame ID reference for obstacle information
const std::string FRAME_ID("base_footprint");

// Topic name where the PointCloud2 message is published
const std::string POINT_CLOUD_TOPIC("/camera/depth/points");

// Publishers topic
const std::string SEGMENTED_PLANE_TOPIC("/segmented_plane");
const std::string OBSTACLES_INFORMATION_TOPIC("/obstacles");
const std::string BOX_MARKERS_TOPIC("/visualization_markers");
const std::string CLUSTERED_NON_PLANAR_TOPIC("/clustered_non_planar");

#endif //OBSTACLE_CLUSTERING_CONFIG_H
