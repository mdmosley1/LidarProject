/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

using PointI = pcl::PointXYZI;
using cloudPtr = pcl::PointCloud<PointI>::Ptr;


// global variables
float cluster_tol_g = 0.4;
int cluster_min_size_g = 25;

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


Color GetColor()
{
    static int idx = 0;
    int i = idx++ % 4;
    switch (i)
    {
    case 0:
        return Color(1,0,0); // red
        break;
    case 1:
        return Color(0,0,1); // blue
        break;
    case 2:
        return Color(1,0,1); // purple
        break;
    case 3:
        return Color(1,1,0); // yellow
        break;
    default:
        std::cout << __FUNCTION__ << ": Error" << "\n";
        return Color(0,0,0);
        break;
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    using PointT = pcl::PointXYZ;
    using cloudPtr = pcl::PointCloud<PointT>::Ptr;

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0.0);

    cloudPtr cloud = lidar->scan();
 
    //renderRays(viewer, lidar->position, cloud);
    //renderPointCloud(viewer, cloud, "points");
    // TODO:: Create point processor
    ProcessPointClouds<PointT> pp;
    int iterations = 20;
    float distThreshold = 0.5;

    // pcl plane segmentation
    //auto segCloud = pp.SegmentPlane(cloud, iterations, distThreshold);
    // my segmentation
    auto segCloud = pp.SegmentPlane(cloud, iterations, distThreshold);

    cloudPtr roadCloud = segCloud.first;
    cloudPtr obstaclesCloud = segCloud.second;
    renderPointCloud(viewer, roadCloud, "plane", Color(0,1,0));
    //renderPointCloud(viewer, segCloud.second, "obstacles", Color(1,0,0));
    int maxClusterSize = 100;
    auto clouds = pp.Clustering(obstaclesCloud, cluster_tol_g, cluster_min_size_g, maxClusterSize);

    std::cout << "We have " << clouds.size()  << " clusters!" << std::endl;
    // visualize the clusters as different colors
    int i = 0;
    for (auto cloud : clouds)
    {
        static int clusterId = 0;
        Box box = pp.BoundingBox(cloud);
        renderBox(viewer, box, clusterId++);
        renderPointCloud(viewer, cloud, "cloud" + std::to_string(i++), GetColor());
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
        viewer->addCoordinateSystem (1.0);
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //renderPointCloud(viewer,inputCloud,"inputCloud");

    double frontDistance = 30;
    double rearDistance = 10;
    double leftRightDistance = 6.5;

    Eigen::Vector4f minPoint {-rearDistance, -leftRightDistance, -3, 0};
    Eigen::Vector4f maxPoint {frontDistance, leftRightDistance, 0, 100};
    double filterResolution = 0.3;
    auto filteredCloud = pointProcessorI->FilterCloud(inputCloud,
                                                      filterResolution,
                                                      minPoint,
                                                      maxPoint);

    int iterations = 20;
    float distThreshold = 0.15;
    //auto segCloud = pointProcessorI->SegmentPlane(filteredCloud, iterations, distThreshold);
    auto segCloud = pointProcessorI->MySegmentPlane(filteredCloud, iterations, distThreshold);
    cloudPtr roadCloud = segCloud.first;
    cloudPtr obstaclesCloud = segCloud.second;
    renderPointCloud(viewer, roadCloud, "plane", Color(0,1,0));
    //renderPointCloud(viewer, obstaclesCloud, "obstacles", Color(1,0,0));
    //renderPointCloud(viewer, filteredCloud, "filteredCloud");

    int maxClusterSize = 500;
    //auto clouds = pointProcessorI->Clustering(obstaclesCloud, tolerance, minClusterSize, maxClusterSize);
    auto clouds = pointProcessorI->MyClustering(obstaclesCloud, cluster_tol_g, cluster_min_size_g, maxClusterSize);

    std::cout << "We have " << clouds.size()  << " clusters!" << std::endl;
    // visualize the clusters as different colors
    int i = 0;
    int clusterId = 0;
    for (auto cloud : clouds)
    {
        Box box = pointProcessorI->BoundingBox(cloud);
        renderBox(viewer, box, clusterId++);
        renderPointCloud(viewer, cloud, "cloud" + std::to_string(i++), GetColor());
    }
}

void PrintUsage(char* arg0)
{
    std::string programName = std::string(arg0);
    std::cout << "Usage: " << programName << " [-t cluster_tolerance] [-s cluster_min_size]"  << std::endl;
}

int main (int argc, char** argv)
{
    int c;
    while ((c = getopt (argc, argv, ":t:s:h")) != -1)
        switch (c)
        {
        case 't':
            cluster_tol_g = atof(optarg);
            std::cout << "tolerance = " << cluster_tol_g << "\n";
            break;
        case 's':
            cluster_min_size_g = atoi(optarg);
            break;
        case 'h':
            PrintUsage(argv[0]);
            return 0;
            break;
        case '?':
            std::cout << "Unknown option." << std::endl;
            PrintUsage(argv[0]);
            return 1;
            break;
        }
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //CameraAngle setAngle = XY;
    //CameraAngle setAngle = FPS;
    CameraAngle setAngle = TopDown;
    initCamera(setAngle, viewer);

    // Simple highway is a simulation
    //simpleHighway(viewer);
    ProcessPointClouds<PointI>* pointProcessorI = new ProcessPointClouds<PointI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}
