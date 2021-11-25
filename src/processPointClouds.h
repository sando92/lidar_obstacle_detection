// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <boost/filesystem/operations.hpp>
#include <unordered_set>
#include "render/box.h"
#include "kdtree3D.h"

//BoxQ
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <math.h>

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    //Bounding box functions
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
    BoxQ BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster);
    BoxQ BoundingBoxQPlane(typename pcl::PointCloud<PointT>::Ptr cluster);
    
    // segmentation functions
    std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> HMSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    
    // clustering functions
    void proximity(typename pcl::PointCloud<PointT>::Ptr cloud, int id, std::unordered_set<int>& processed_points, std::vector<int>& cluster, KdTree3D* tree, float distanceTol);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> HMClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);
    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */