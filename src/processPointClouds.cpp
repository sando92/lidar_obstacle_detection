#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    // extract roof points
    std::vector<int> indices;
    
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f( -1.5, -1.7, -1, 1 ));
    roof.setMax(Eigen::Vector4f( 2.6, 1.7, -.4, 1 ));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int point: indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{

    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    // initialize random seed
    srand(time(NULL));
    
    while (maxIterations--) 
    {
        std::unordered_set<int> inliersTemp;

        // thanks to the "set" structure, impossible to insert twice the same number
        while (inliersTemp.size() < 3)
            inliersTemp.insert(rand()%(cloud->points.size()));

        // calculate line parameters between randomly chosen points of indexes f and s
        float x1, y1, z1;
        float x2, y2, z2;
        float x3, y3, z3;
        auto itr = inliersTemp.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        std::vector<float> v1, v2;
        v1 = { x2 - x1, y2 - y1, z2 - z1 };
        v2 = { x3 - x1, y3 - y1, z3 - z1 };

        // normal vector to the plane by taking  cross product of v1 x v2
        std::vector<float> norm_v = 
            { ( y2 - y1 ) * ( z3 - z1 ) - ( z2 - z1 ) * ( y3 - y1 ),
              ( z2 - z1 ) * ( x3 - x1 ) - ( x2 - x1 ) * ( z3 - z1 ),
              ( x2 - x1 ) * ( y3 - y1 ) - ( y2 - y1 ) * ( x3 - x1 )};

        float A = norm_v[0];
        float B = norm_v[1];
        float C = norm_v[2];
        float D = - ( A * x1 + B * y1 + C * z1);
    
        // calculate distance for each point
        for (int n = 0; n < cloud->points.size(); n++) {
            if (inliersTemp.count(n) > 0)
                continue;

            float x = cloud->points[n].x;
            float y = cloud->points[n].y;
            float z = cloud->points[n].z;
            float distance = std::fabs( ( A * x ) + ( B * y ) + ( C * z ) + D )
                            / std::sqrt( ( A * A ) + ( B * B ) + ( C * C ) );

            if (distance <= distanceTol) {
                inliersTemp.insert(n);
            }
        }
        if (inliersResult.size() < inliersTemp.size()) {
            inliersResult = inliersTemp;
        }
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac Plane took " << elapsedTime.count() << " milliseconds" << std::endl;

    return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::HMSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold){
    // Plane segmentation
    std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, int id, std::unordered_set<int>& processed_points, std::vector<int>& cluster, KdTree3D* tree, float distanceTol) {
    processed_points.insert(id);
    cluster.push_back( id );

    std::vector<int> nearby = tree->search( cloud->points[id], distanceTol );
    
    for (std::vector<int>::iterator itr = nearby.begin(); itr != nearby.end(); ++itr) {
        if (processed_points.find( *itr ) == processed_points.end())
            proximity(cloud, *itr, processed_points, cluster, tree, distanceTol);
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::HMClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();

    std::vector<Point>* point_arr(new std::vector<Point>);
    
    for (int i = 0; i < cloud->size(); i++) {
        std::vector<float> arr = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}; 
        point_arr->push_back(Point(arr, i));
    }

    // KDTree creation, to able a efficient search
    KdTree3D* tree3D = new KdTree3D(point_arr);

    auto partTime = std::chrono::steady_clock::now();
    auto partElapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(partTime - startTime);
    std::cout << "3DTree created, took " << partElapsedTime.count() << " milliseconds" << std::endl;

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::unordered_set<int> processed_points;

    for (int i = 0; i < cloud->points.size(); i++) {
        if (processed_points.find(i) == processed_points.end()) {
            std::vector<int> cluster;
            proximity(cloud, i, processed_points, cluster, tree3D, distanceTol);
            if ( cluster.size() >= minSize && cluster.size() <= maxSize){
                typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
                for(int indice: cluster) {
                    PointT point = cloud->points[indice];
                    clusterCloud->points.push_back(point);
                }
                clusterCloud->width = clusterCloud->size ();
                clusterCloud->height = 1;
                clusterCloud->is_dense = true;
                clusters.push_back(clusterCloud);
            }
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Total clustering process took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQPlane(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                    ///    the signs are different and the box doesn't get correctly oriented in some cases.

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    // The first 3 rows and columns (top left) components are the rotation matrix.
    // The first 3 rows of the last column is the translation
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();

    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);

    float Axx, Ayx;
    Axx = projectionTransform(0,0);
    Ayx = projectionTransform(1,0);

    // extract only rotation around Z axis
    float theta_z = atan2(Ayx, Axx);
    float Zxx, Zxy, Zxz, Zyx, Zyy, Zyz, Zzx, Zzy, Zzz;// = 0;
    Zxx = cos(theta_z);
    Zxy = -sin(theta_z);
    Zxz = 0;
    Zyx = sin(theta_z);
    Zyy = cos(theta_z);
    Zyz = 0;
    Zzx = 0;
    Zzy = 0;
    Zzz = 1;

    for (auto point : cluster->points) {
        PointT transformed_point;

        transformed_point.x = Zxx*point.x + Zxy*point.y + Zxz*point.z;
        transformed_point.y = Zyx*point.x + Zyy*point.y + Zyz*point.z;
        transformed_point.z = Zzx*point.x + Zzy*point.y + Zzz*point.z;

        cloudPointsProjected->points.push_back(transformed_point);
    }

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    // only around Z axis
    eigenVectorsPCA << Zxx, Zyx, Zzx, Zxy, Zyy, Zzy, Zxz, Zyz, Zzz;
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal;

    BoxQ box;
    // find parameters
    box.bboxTransform = bboxTransform;
    box.bboxQuaternion = bboxQuaternion;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                    ///    the signs are different and the box doesn't get correctly oriented in some cases.

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    // The first 3 rows and columns (top left) components are the rotation matrix.
    // The first 3 rows of the last column is the translation
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);

    float Axx, Axy, Axz, Ayx, Ayy, Ayz, Azx, Azy, Azz, Tx, Ty, Tz;
    Axx = projectionTransform(0,0);
    Axy = projectionTransform(0,1);
    Axz = projectionTransform(0,2);
    Ayx = projectionTransform(1,0);
    Ayy = projectionTransform(1,1);
    Ayz = projectionTransform(1,2);
    Azx = projectionTransform(2,0);
    Azy = projectionTransform(2,1);
    Azz = projectionTransform(2,2);
    Tx = projectionTransform(0,3);
    Ty = projectionTransform(1,3);
    Tz = projectionTransform(2,3);

    for (auto point : cluster->points) {
        PointT transformed_point;

        transformed_point.x = Axx*point.x + Axy*point.y + Axz*point.z + Tx;
        transformed_point.y = Ayx*point.x + Ayy*point.y + Ayz*point.z + Ty;
        transformed_point.z = Azx*point.x + Azy*point.y + Azz*point.z + Tz;

        cloudPointsProjected->points.push_back(transformed_point);
    }
    std::cout << "Result processed." << std::endl;

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    BoxQ box;
    // find parameters

    box.bboxTransform = bboxTransform;
    box.bboxQuaternion = bboxQuaternion;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}