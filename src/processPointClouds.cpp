// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


// Function Name: Constructor for the class "ProcessPointClouds"
// Function Description: This class contains all methods that will be used to process the lidar pointcloud
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


// Function Name: de-constructor for the class "ProcessPointClouds"
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


// Function Name: numPoints
// Function Description: Prints the number of points available in the pointcloud
// Inputs:
        // - cloud: pointCloud to be processed
// Outputs: Non
template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}



// Function Name: FilterCloud
// Function Description: - Filter the point cloud data 
                            // - Downsample the data for faster processing
                            // - According to the region of interest
//                       - Compute a time cost of the function 
// Inputs:
        // - cloud: pointCloud to be processed
        // - filterRes: the resolution of the data that will be processed after that
// Outputs: Non
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}



// Function Name: SeparateClouds
// Function Description: Create two new point clouds, one cloud with obstacles and other with segmented plane
// Inputs:
        // - inliers: 
        // - cloud: pointCloud to be processed
// Outputs: 
        // - segResult: Outliers and inliers
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacle (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());

    // get cloud_plane
	// loop over the indices of interest in the cloud data and add them to cloud_plane
    for (int index: inliers->indices) {
        cloud_plane->points.push_back(cloud->points[index]);
    }

    // get cloud_obstacle
	
	// Create Indices extrator to be used to apply 
	// obstacles = cloud - inliers
    pcl::ExtractIndices<PointT> extract;

	// Set input Cloud
    extract.setInputCloud (cloud);

	// Set indicies to be subtracted from input
    extract.setIndices (inliers);

	// Set the substraction
    extract.setNegative (true);
    
	// Extract obstacles and save their PCD to cloud_obstacle
	extract.filter (*cloud_obstacle);

	// Create a variable of type pair to collect the outliers and inliers
    // outliers = >> cloud_obstacle
	// inliers = >> cloud_planes
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obstacle, cloud_plane);
	
    return segResult;
}



// Function Name: SegmentPlane
// Function Description: Do plane Segmentatio to detect the road carpet using RANSAC algorithm
// Inputs:
        // - cloud: pointCloud to be processed
        // - maxIterations: Maximum number of iletrations for the algorithm 
        // - distanceThreshold: Maximum distance, a way from the trial plane, for a point to be added to the cluster 
// Outputs: 
        // - segResult: 
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    // prepare segmentation
    // PCD of interest in segmentation process are called inliers, in our case inliers are point cloud on road surface
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Prepare a model coffecients to be used to store the coeffecients of the plan equation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

	// create an instance of the SAC segmentaion class of the PCL, to be used in segmentation after that 
    pcl::SACSegmentation<PointT> seg;

	// Try with true and false and notice the difference in the coefficients
	// True: A:B:C:D = 1:x:x:x
    seg.setOptimizeCoefficients (true);

	// Choose the model of the surface to be a plane 
    seg.setModelType (pcl::SACMODEL_PLANE);

	// Choose the type the algorithm/method for segmentation 
    seg.setMethodType (pcl::SAC_RANSAC);

	// Set the maximum number of iletrartions for the Ransac Alg.
	// The more is better but less time efficient
    seg.setMaxIterations (maxIterations);
    
	// Set the maximum distance for a point to be considered belonging to the plane 
	seg.setDistanceThreshold (distanceThreshold);

    // Set the input point cloud
	seg.setInputCloud (cloud);

    // start segmentation
	// output is written to inliers (PCD of interest), coefficients (A, B, C, D)
    seg.segment (*inliers, *coefficients);

	// Print out the coefficients of the Equation Ax+By+Cz+D = 0 
	std::cout << "\nPlane Coeff [A, B, C, D]: \n" << *coefficients << std::endl;

	// If there is too many traffic arround the host vehicle, the lidar rays willnot reach the ground
	// Segementation fails 
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // Get two clouds from inliners and cloud
	// first is the ouliers
	// second is the inliers
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds\n" << std::endl;

    return segResult;
}


// Function Name: Clustering
// Function Description: Do Euclidean clustering for detecting objects
// Inputs:
        // - cloud: pointCloud to be processed
        // - clusterTolerance: Maximum redius of the cluster
        // - minSize: Minimum no. of points to be considered as a cluster/object
        // - maxSize: Maximum no. of points to be considered as a cluster/object
// Outputs: 
        // - clusters: Vector of pointclouds pointers. i.e vector of pointers to objects' first points
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

	// Vector of clusters of PC
	// [cluster1[pointcloud], cluster2[pointcloud], cluster3[pointcloud]]
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Construct a kd tree to make Euclidean run faster 
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

	// Insert the cloud into the tree
    tree->setInputCloud (cloud);

    // Euclidean cluster

	// Prepare a vector of pointCloudIndicies to be used to store the indices of each cluster 
	// [cluster1[indices], cluster2[indices], cluster3[indices], ...]
    std::vector<pcl::PointIndices> cluster_indices;

	// Create an instance of the EuclideanClustering class
    pcl::EuclideanClusterExtraction<PointT> ec;

	// Set the maximum distance allowed for a point to away from the nearby point to be considered in the cluster
	// a part of the cluster
    ec.setClusterTolerance (clusterTolerance); 

	// Minimum number of points for a cluster to be considered a valid object
	// used to ignore noise
    ec.setMinClusterSize (minSize);
 
	// Maximum number of points for a cluster to be considered a valid object
	// used to ignore background objects like fences and buildings
    ec.setMaxClusterSize (maxSize);

	// Set the search method of the EuclideanClustering to Tree to be fast
	// Try to comment this out and notice the difference in clustering time 
    ec.setSearchMethod (tree);

	// Set the input to the Euclidean Clustering
    ec.setInputCloud (cloud);

	// Run Euclidean Clustering and store the output in cluster_indices
    ec.extract (cluster_indices);

    // get cluster cloud

	// Loop over the clusters and get the indices of each cluster
    for (pcl::PointIndices get_indices: cluster_indices)
    {
		// Create new PCD to hold PCD of the current cluster
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		
		// Construct the cluster PCD from the Cluster indices
		// Convert indices into PCD
        for (int i: get_indices.indices)
            cloud_cluster->points.push_back(cloud->points[i]); //*

		// Compute Cluster size, could be used for filtering
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

		// Push the current cluster to the vector of all clusters
        clusters.push_back(cloud_cluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


// Function Name: BoundingBox
// Function Description: Plotting 3D Boundary box around the detected cluster/object
// Inputs:
        // - cluster: group of points the belongto the same object
// Outputs: 
        // - box: 3d Bounding box (structure of [min, max][x;y;z])
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


// Function Name: savePcd
// Function Description: save pointcloud to a certain path
// Inputs:
        // - cloud: pointCloud to be processed
        // - file: output file path
// Outputs: Non
template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


// Function Name: loadPcd
// Function Description: load pointcloud from a certain path
// Inputs:
        // - file: input file path
// Outputs: 
        // - cloud: loaded pointCloud
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


// Function Name: streamPcd
// Function Description: sort files that are in a certain directory in accending order so playback is chronological
// Inputs:
        // - dataPath: input directory path
// Outputs: 
        // - paths: vector containing sorted paths of point cloud data in this directory
template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}