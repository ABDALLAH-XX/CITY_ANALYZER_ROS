#ifndef CITY_ANALYZER_H
#define CITY_ANALYZER_H

#define PCL_NO_PRECOMPILE

// Core PCL types and containers
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/memory.h> // For Ptr types

// Standard and Math library
#include <string>
#include <vector>
#include <Eigen/Core>


// 1. Define the custom struct once
struct EIGEN_ALIGN16 CustomPointT {
    double x, y, z;
    uint8_t intensity;
    double gps_time;
    float scan_angle;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// 2. The Registration Macro
// IMPORTANT: This MUST stay in the header file, right after the struct.
// This tells PCL how to map your struct fields to XML/PLY/PCD tags.
POINT_CLOUD_REGISTER_POINT_STRUCT (CustomPointT,
    (double, x, X)
    (double, y, Y)
    (double, z, Z)
    (uint8_t, intensity, Intensity)
    (double, gps_time, GPS_Time)
    (float, scan_angle, Scan_Angle)
)



class CityAnalyzer {
public:
    // Aliases
    using PointT = CustomPointT;
    using CloudT = pcl::PointCloud<PointT>;

    using VisuPointT = pcl::PointXYZI;
    using VisuCloudT = pcl::PointCloud<VisuPointT>;

    using NormalT = pcl::Normal;
    using NormalCloudT = pcl::PointCloud<NormalT>;

private:
    CloudT::Ptr raw_cloud_;
    VisuCloudT::Ptr processed_cloud_;
    NormalCloudT::Ptr cloud_normals_;


    double offX_ = 0.0, offY_ = 0.0, offZ_ = 0.0;


public:
    CityAnalyzer();
    bool loadCityFile(const std::string& filename);
    void prepareData(const std::string& field_name);
    void filterByRange(const std::string& axis, float min, float max);
    void applyVoxelFilter(float leaf_size);
    void cropCloud(float minX, float maxX, float minY, float maxY, float minZ, float maxZ);
    void extractGroundAndObjects(VisuCloudT::Ptr ground, VisuCloudT::Ptr objects);
    void extractClusters(VisuCloudT::Ptr objects, std::vector<VisuCloudT::Ptr>& clusters);
    void computeNormals(double radius);

    struct BBox {
    float minX, maxX, minY, maxY, minZ, maxZ;
    };
    
    BBox computeClusterBBox(VisuCloudT::Ptr cluster);

    // Getters (Only declarations here)
    VisuCloudT::Ptr getVisuCloud() const;
    NormalCloudT::Ptr getNormals() const;
};

#endif