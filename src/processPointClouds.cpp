// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
#include "helper.h"
#include "kdtree.h"
#include "cluster.h"


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                               float filterRes,
                                                                               Eigen::Vector4f minPoint,
                                                                               Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // for (auto& pt : cloud->points)
    //     std::cout << "x=" << pt.x << ", y=" << pt.y << ", z=" << pt.z << ", i=" << pt.intensity << "\n";

    // Do voxel grid point reduction and region based filtering
    //pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>() );
    
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter(*cloud);

    // filter out everything beyond region of interest
    pcl::CropBox<PointT> cropBox(true);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(cloud);
    cropBox.filter(*cloud);
    //cropBox.filter(indices);
    //std::vector<int> indices;

    // typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>());
    // for (auto& i : indices)
    //      filteredCloud->points.push_back(cloud->points[i]);


    // filter out the points from the roof of ego vehicle
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloud);
    std::vector<int> roofPtsIndices;
    roof.filter(roofPtsIndices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (auto point : roofPtsIndices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>() );
    typename pcl::PointCloud<PointT>::Ptr obsCloud (new pcl::PointCloud<PointT>() );

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*planeCloud);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obsCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planeCloud, obsCloud);
    return segResult;
}

struct Plane
{
    float A = 0;
    float B = 0;
    float C = 0;
    float D = 0;
};

template<typename PointT>
std::vector<PointT> GetThreeRandomPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::unordered_set<int> indices;
    while(indices.size() < 3)
        indices.insert(GetRandInt(cloud->size()));

    std::vector<PointT> pts;
    auto it = indices.begin();
    pts.push_back(cloud->points[*it]); it++;
    pts.push_back(cloud->points[*it]); it++;
    pts.push_back(cloud->points[*it]);

    return pts;
}

template<typename PointT>
std::unordered_set<int> GetInliersForPlane(Plane p, typename pcl::PointCloud<PointT>::Ptr cloud, float distanceToPlane)
{
    std::unordered_set<int> inliers;

    for (int idx = 0; idx < cloud->points.size(); ++idx)
    {
        auto pt = cloud->points[idx];
        float dist = std::fabs(p.A*pt.x + p.B*pt.y + p.C*pt.z + p.D) / std::sqrt(p.A*p.A + p.B*p.B + p.C*p.C);
        if (dist < distanceToPlane)
            inliers.insert(idx);
    }
    return inliers;
}

template<typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud,
                               int maxIterations,
                               float distanceThresh)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations
    for (int i = 0; i < maxIterations; ++i)
    {
        // Randomly sample subset and fit plane using three random points
        // from cloud
        std::vector<PointT> pts = GetThreeRandomPoints<PointT>(cloud);

        Plane plane;
        // A = (y1-y0)*(z2-z0) - (z1-z0)*(y2-y0)
        plane.A = (pts[1].y - pts[0].y)*(pts[2].z - pts[0].z) -
                  (pts[1].z - pts[0].z)*(pts[2].y - pts[0].y);
        // B = (z1-z0)*(x2-x0) - (x1-x0)*(z2-z0)
        plane.B = (pts[1].z - pts[0].z)*(pts[2].x - pts[0].x) -
                  (pts[1].x - pts[0].x)*(pts[2].z - pts[0].z);
        // C = (x1-x0)*(y2-y0) - (y1-y0)*(x2-x0)
        plane.C = (pts[1].x - pts[0].x)*(pts[2].y - pts[0].y) -
                  (pts[1].y - pts[0].y)*(pts[2].x - pts[0].x);
        plane.D = -1*(plane.A*pts[1].x + plane.B*pts[1].y + plane.C*pts[1].z);

        // Measure distance between every point and fitted line
        std::unordered_set<int> inliers = GetInliersForPlane<PointT>(plane, cloud, distanceThresh);

        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }
    // Return indicies of inliers from fitted line with most inliers
     return inliersResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    auto inliers = Ransac3D<PointT>(cloud, maxIterations, distanceThreshold);

    pcl::PointIndices::Ptr inliersPCL (new pcl::PointIndices ());
    for (auto inlier : inliers)
        inliersPCL->indices.push_back(inlier);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersPCL,cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    using pXYZ = PointT;
    
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
     pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
     pcl::SACSegmentation<pXYZ> seg;
     seg.setOptimizeCoefficients (true);
     // Mandatory
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setMaxIterations (maxIterations);
     seg.setDistanceThreshold (distanceThreshold);

     // Create the filtering object
     pcl::ExtractIndices<pXYZ> extract;
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MyClustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                          float clusterTolerance,
                                                                                          int minSize,
                                                                                          int maxSize)

{

    // convert cloud points into format usable by my kdtree
    std::vector<std::vector<float>> points;
    for (auto& pt : cloud->points)
    {
        std::vector<float> myPoint = {pt.x, pt.y, pt.z};
        points.push_back(myPoint);
    }

    // build kdtree from cloud points
    KdTree* tree = new KdTree;

    for (int i=0; i < points.size(); i++)
        tree->insert(points[i], i);

    // use euclidean clustering algorithm to find clusters from points using kdtree
    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, clusterTolerance);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterClouds;

    for(std::vector<int> cluster : clusters)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for(int indice: cluster)
        {
            //clusterCloud->points.push_back(PointT( points[indice][0], points[indice][1], points[indice][2] ));
            //PointT pointxyzi( points[indice][0], points[indice][1], points[indice][2] );
            PointT pointxyzi;
            pointxyzi.x = points[indice][0];
            pointxyzi.y = points[indice][1];
            pointxyzi.z = points[indice][2];
            clusterCloud->points.push_back(pointxyzi);
        }
        if (clusterCloud->points.size() <= maxSize && clusterCloud->points.size() >= minSize)
            clusterClouds.push_back(clusterCloud);
    }
    return clusterClouds;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                          float clusterTolerance,
                                                                                          int minSize,
                                                                                          int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> clustersIndices;

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clustersIndices);

    for (pcl::PointIndices clusterInds : clustersIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for (int index : clusterInds.indices)
            cloudCluster->points.push_back (cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
