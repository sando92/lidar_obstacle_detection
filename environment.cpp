/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

// #include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

    bool b_renderPointClouds = true;
    bool b_render_seg_plane = false;
    bool b_render_seg_obstacles = false;
    bool b_render_clusters = true;
    bool b_render_boxes = true;
  
    // 1- filtering
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud 
    = pointProcessorI->FilterCloud(inputCloud, 0.5 , Eigen::Vector4f (-12, -5, -3, 1), Eigen::Vector4f ( 25, 7, 1, 1));
    
    if (b_renderPointClouds)
        renderPointCloud(viewer,filterCloud,"filterCloud");

    // 2 - segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
        segResult = pointProcessorI->HMSegmentPlane(filterCloud, 10, 0.2);

    if (b_render_seg_plane)
        renderPointCloud(viewer, segResult.first, "plane", Color(0.2,1,1));
    if (b_render_seg_obstacles)
        renderPointCloud(viewer, segResult.second, "obstacles", Color(1,0,0));

    std::cout << "Cloud with obstacles of size " << segResult.second->size() << std::endl;
    // 3 - clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters 
    = pointProcessorI->HMClustering(segResult.second, 0.67, 5, 300); // 0.67, 5, 300

    std::cout << "Found " << cloudClusters.size() << " clusters." << std::endl;

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    if (b_render_clusters) {
        for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
        {
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);

            //4 - bounding box
            if (b_render_boxes) {
                Box box = pointProcessorI->BoundingBox(cluster);
                renderBox(viewer, box, clusterId, colors[0], 70.0);
            }
            ++clusterId;
        }
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem(1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
        streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}