#include "city_analyzer_ros/CityAnalyzer.hpp" 


// PCL Logic Headers
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>

/**
 * @brief Constructor
 * Initializes the smart pointers for raw data, processed points, and normals.
 * This prevents segmentation faults when methods are called.
 */
CityAnalyzer::CityAnalyzer() : 
    raw_cloud_(new CloudT), 
    processed_cloud_(new VisuCloudT), 
    cloud_normals_(new NormalCloudT) {}

/**
 * @brief Detects file extension and loads point cloud into raw_cloud_.
 * Also sets the global offsets (offX, offY, offZ) based on the first point 
 * to handle large geographical coordinates.
 */
bool CityAnalyzer::loadCityFile(const std::string& filename) {
    if (filename.length() < 5) return false;
    std::string extension = filename.substr(filename.length() - 4);
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

    int status = -1;
    if (extension == ".ply") {
        status = pcl::io::loadPLYFile<PointT>(filename, *raw_cloud_);
    } 
    else if (extension == ".pcd") {
        status = pcl::io::loadPCDFile<PointT>(filename, *raw_cloud_);
    } 

    if (status == -1 || raw_cloud_->empty()) return false;

    // Use the first point as the anchor for the local coordinate system
    offX_ = raw_cloud_->points[0].x;
    offY_ = raw_cloud_->points[0].y;
    offZ_ = raw_cloud_->points[0].z;
    
    return true;
}

/**
 * @brief Converts raw double-precision data to float-precision for PCL/GPU usage.
 * Subtracts the offsets to center the cloud and maps the chosen field (like gps_time) 
 * to the 'intensity' channel for the visualizer.
 */
void CityAnalyzer::prepareData(const std::string& field_name) {
    processed_cloud_->clear();
    processed_cloud_->reserve(raw_cloud_->size());

    for (const auto& pt : raw_cloud_->points) {
        VisuPointT v;
        v.x = pt.x - offX_;
        v.y = pt.y - offY_;
        v.z = pt.z - offZ_;
        v.intensity = (field_name == "gps_time") ? pt.gps_time : pt.intensity;
        processed_cloud_->push_back(v);
}

}


/**
 * @brief Applies a VoxelGrid filter to reduce point density.
 * This is essential for performance on hardware like the IdeaPad S145.
 */
void CityAnalyzer::applyVoxelFilter(float leaf_size) {
    if (processed_cloud_->empty()) return;
    
    pcl::VoxelGrid<VisuPointT> sor;
    sor.setInputCloud(processed_cloud_);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    
    sor.filter(*processed_cloud_);
    cloud_normals_->clear(); // Invalidate normals as geometry has changed
    
    /*std::cout << "Voxel filter has been applied. Points remaining: " 
              << processed_cloud_->size() << std::endl;*/
}

/**
 * @brief Filters the cloud along a specific axis within a given range.
 * @param axis The axis to filter ("x", "y", or "z").
 * @param min Minimal value in meters.
 * @param max Maximal value in meters.
 */
void CityAnalyzer::filterByRange(const std::string& axis, float min, float max) {
    if (processed_cloud_->empty()) return;

    pcl::PassThrough<VisuPointT> pass;
    pass.setInputCloud(processed_cloud_);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(min, max);
    
    // This will overwrite the processed_cloud with the clipped version
    pass.filter(*processed_cloud_);

    // IMPORTANT: Since we changed the number of points, we MUST clear the normals
    // to keep the data synchronized.
    cloud_normals_->clear();

    /*std::cout << "PassThrough filter applied on " << axis << ". Points remaining: " 
              << processed_cloud_->size() << std::endl;*/
}

/**
 * @brief Crops the cloud into a specific 3D bounding box.
 * Ideal for isolating specific buildings or street sections.
 */
void CityAnalyzer::cropCloud(float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {
    if (processed_cloud_->empty()) return;

    pcl::CropBox<VisuPointT> box;
    box.setInputCloud(processed_cloud_);
    box.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0f));
    box.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0f));

    box.filter(*processed_cloud_);
    cloud_normals_->clear();

    /*std::cout << "CropCloud filter has been applied. Points remaining: " 
              << processed_cloud_->size() << std::endl;*/
}

/**
 * @brief Segments the processed cloud into ground and object points using RANSAC.
 * @param ground Output cloud for ground points.
 * @param objects Output cloud for object points (buildings, cars, etc.).
 */
void CityAnalyzer::extractGroundAndObjects(VisuCloudT::Ptr ground, VisuCloudT::Ptr objects) {
    if (processed_cloud_->empty()) return;

    // MAUVAISE PRATIQUE (Ancien style avec new)
    //pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // BONNE PRATIQUE (Style PCL moderne)
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();
    
    // Configurer RANSAC pour trouver un plan
    pcl::SACSegmentation<VisuPointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(150);
    seg.setDistanceThreshold(0.15); // Les points à moins de 15cm du plan sont considérés comme sol

    seg.setInputCloud(processed_cloud_);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        //std::cerr << "Aucun plan (sol) n'a pu être détecté." << std::endl;
        return;
    }

    // Extraction physique des points
    pcl::ExtractIndices<VisuPointT> extract;
    extract.setInputCloud(processed_cloud_);
    extract.setIndices(inliers);

    // 1. Extraire le sol (Inliers)
    extract.setNegative(false);
    extract.filter(*ground);

    // 2. Extraire les objets (Outliers : bâtiments, poteaux, voitures)
    extract.setNegative(true);
    extract.filter(*objects);
}

/**
 * @brief Segments the object cloud into individual clusters using Euclidean Cluster Extraction.
 * @param objects Input cloud containing only object points.
 * @param clusters Output vector of point cloud pointers, each representing a cluster.
 */
void CityAnalyzer::extractClusters(VisuCloudT::Ptr objects, std::vector<VisuCloudT::Ptr>& clusters) {
    if (objects->empty()) return;

    // Création d'un KdTree pour une recherche spatiale rapide
    auto tree = std::make_shared<pcl::search::KdTree<VisuPointT>>();
    tree->setInputCloud(objects);
    tree->setSortedResults(false);


    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<VisuPointT> ec;
    
    // PARAMÈTRES CRUCIAUX
    ec.setClusterTolerance(0.8); // 50cm entre deux points pour qu'ils appartiennent au même objet
    ec.setMinClusterSize(300);   // Minimum de points pour être un objet (évite le bruit)
    ec.setMaxClusterSize(25000); // Maximum de points
    
    ec.setSearchMethod(tree);
    ec.setInputCloud(objects);
    ec.extract(cluster_indices);

    // Extraction des nuages individuels
    for (const auto& indices : cluster_indices) {
        VisuCloudT::Ptr cluster = std::make_shared<VisuCloudT>();
        cluster->points.reserve(indices.indices.size());
        for (int idx : indices.indices) {
            cluster->points.push_back(objects->points[idx]);

        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
    }
}

/**
 * @brief Computes the axis-aligned bounding box for a given cluster.
 * @param cluster Input point cloud representing a single cluster.
 * @return BBox struct containing min and max coordinates along each axis.
 */
CityAnalyzer::BBox CityAnalyzer::computeClusterBBox(VisuCloudT::Ptr cluster) {
    VisuPointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);

    return BBox{min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z};
}

/**
 * @brief Estimates surface normals using a local neighborhood search.
 * @param radius The sphere radius (meters) used to find neighboring points for plane fitting.
 */
void CityAnalyzer::computeNormals(double radius) {
    if (processed_cloud_->empty()) return;

    pcl::NormalEstimation<VisuPointT, NormalT> ne;

    auto tree = std::make_shared<pcl::search::KdTree<VisuPointT>>();
    tree->setSortedResults(false);   
    
    ne.setSearchMethod(tree);
    ne.setInputCloud(processed_cloud_);
    ne.setRadiusSearch(radius);
    ne.compute(*cloud_normals_);
    ne.setViewPoint(0,0,1000);   // évite les inversions coûteuses

}


/** @return Shared pointer to the processed cloud for visualization. */
CityAnalyzer::VisuCloudT::Ptr CityAnalyzer::getVisuCloud() const {
    return processed_cloud_;
}

/** @return Shared pointer to the computed normal cloud. */
CityAnalyzer::NormalCloudT::Ptr CityAnalyzer::getNormals() const {
    return cloud_normals_;
}