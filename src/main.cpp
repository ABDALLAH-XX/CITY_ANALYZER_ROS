#include <memory>
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <pcl_conversions/pcl_conversions.h>

#include "city_analyzer_ros/CityAnalyzer.hpp"

class CityAnalyzerNode : public rclcpp::Node
{
public:
    CityAnalyzerNode() : Node("city_analyzer_node")
    {
        // 1. Parameters declaration
        this->declare_parameter("voxel_leaf_size", 0.7); // Augmenté pour la fluidité
        this->declare_parameter("file_path", "/home/abdallah/Bureau/PointCloud/data/Lille_1.ply");
        this->declare_parameter("normal_radius", 1.2);
        this->declare_parameter("ground_distance_threshold", 0.15);

        // 2. Publishers setup

        // QoS configuration
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();


        publisher_cloud_   = this->create_publisher<sensor_msgs::msg::PointCloud2>("/city_output", qos);
        publisher_ground_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/city_ground", qos);
        publisher_objects_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/city_objects", qos);
        publisher_clusters_= this->create_publisher<sensor_msgs::msg::PointCloud2>("/city_clusters", qos);

        publisher_bboxes_  = this->create_publisher<visualization_msgs::msg::MarkerArray>("/city_bboxes", qos);
        publisher_normals_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/city_normals", qos);


        analyzer_ = std::make_unique<CityAnalyzer>();

        std::string path = this->get_parameter("file_path").as_string();
        
        RCLCPP_INFO(this->get_logger(), "Loading attempt : %s", path.c_str());
        
        if (analyzer_->loadCityFile(path)) {
            RCLCPP_INFO(this->get_logger(), "File loaded successfully. Waiting for spin...");
            startup_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&CityAnalyzerNode::process_and_publish, this));
            
        }

    }

private:
    void process_and_publish() {
        startup_timer_->cancel();

        // A. Processing and filtering
        analyzer_->prepareData("gps_time");
        double leaf = this->get_parameter("voxel_leaf_size").as_double();
        analyzer_->applyVoxelFilter(leaf);

        analyzer_->filterByRange("z", -3.0f, 5.0f);

        // Publishing filtered cloud
        auto full_cloud = analyzer_->getVisuCloud();
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*full_cloud, cloud_msg);
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = this->now();
        publisher_cloud_->publish(cloud_msg);
        
        
        // B. Ground/Objects segmentation
        auto ground_pcl = std::make_shared<CityAnalyzer::VisuCloudT>();
        auto objects_pcl = std::make_shared<CityAnalyzer::VisuCloudT>();
        analyzer_->extractGroundAndObjects(ground_pcl, objects_pcl);
        
        publish_segmented_clouds(ground_pcl, objects_pcl);

        // C. Clustering (Objects detection)
        std::vector<CityAnalyzer::VisuCloudT::Ptr> clusters;
        analyzer_->extractClusters(objects_pcl, clusters);

        // D. Bounding Boxes and Coloring
        auto colored_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        colored_cloud->reserve(objects_pcl->size());

        visualization_msgs::msg::MarkerArray bbox_markers;
        
        std::mt19937 gen(42); // fixed seed for reproducibility
        std::uniform_int_distribution<uint8_t> dis(50, 255);

        auto current_time = this->now();

        for (size_t i = 0; i < clusters.size(); ++i) {
            uint8_t r = dis(gen), g = dis(gen), b = dis(gen);

            // Cloud point coloring
            for (auto& pt : clusters[i]->points) {
                pcl::PointXYZRGB p;
                p.x = pt.x; p.y = pt.y; p.z = pt.z;
                p.r = r; p.g = g; p.b = b;
                colored_cloud->push_back(p);
            }

            // Bounding box computation
            auto box = analyzer_->computeClusterBBox(clusters[i]);
            
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = current_time;
            marker.ns = "detected_objects";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Box center position
            marker.pose.position.x = (box.minX + box.maxX) / 2.0;
            marker.pose.position.y = (box.minY + box.maxY) / 2.0;
            marker.pose.position.z = (box.minZ + box.maxZ) / 2.0;

            // Dimensions (with minimum size to ensure visibility)
            marker.scale.x = std::max(0.1f, box.maxX - box.minX);
            marker.scale.y = std::max(0.1f, box.maxY - box.minY);
            marker.scale.z = std::max(0.1f, box.maxZ - box.minZ);

            // Colors matching the cluster coloring with some transparency
            marker.color.r = r / 255.0f;
            marker.color.g = g / 255.0f;
            marker.color.b = b / 255.0f;
            marker.color.a = 0.4f;

            bbox_markers.markers.push_back(marker);
        }

        // E. Publishing clusters and bounding boxes
        colored_cloud->width = colored_cloud->size();
        colored_cloud->height = 1;
        colored_cloud->is_dense = true;

        sensor_msgs::msg::PointCloud2 cluster_msg;
        pcl::toROSMsg(*colored_cloud, cluster_msg);
        cluster_msg.header.frame_id = "map";
        cluster_msg.header.stamp = current_time;
        publisher_clusters_->publish(cluster_msg);
        publisher_bboxes_->publish(bbox_markers);

        // F. Normals computation and publishing
        double radius = this->get_parameter("normal_radius").as_double();
        analyzer_->computeNormals(radius);
        publish_normals_as_markers();
        
        RCLCPP_INFO(this->get_logger(), "Processing done : %ld objects found.", clusters.size());
        
    }

    void publish_segmented_clouds(CityAnalyzer::VisuCloudT::Ptr ground, 
                                 CityAnalyzer::VisuCloudT::Ptr objects) {
        sensor_msgs::msg::PointCloud2 g_msg, o_msg;
        pcl::toROSMsg(*ground, g_msg);
        pcl::toROSMsg(*objects, o_msg);

        auto now = this->now();
        g_msg.header.frame_id = o_msg.header.frame_id = "map";
        g_msg.header.stamp = o_msg.header.stamp = now;

        publisher_ground_->publish(g_msg);
        publisher_objects_->publish(o_msg);
    }

    void publish_normals_as_markers()
    {
        auto cloud = analyzer_->getVisuCloud();
        auto normals = analyzer_->getNormals();
        if(normals->empty()) return;

        visualization_msgs::msg::MarkerArray marker_array;
        int step = 100; // Display every 100th normal for clarity
        
        for (size_t i = 0; i < cloud->size(); i += step) {
            if (std::isnan(normals->points[i].normal_x)) continue;

            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp = this->now();
            arrow.ns = "normals";
            arrow.id = static_cast<int>(i);
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            
            geometry_msgs::msg::Point start, end;
            start.x = cloud->points[i].x;
            start.y = cloud->points[i].y;
            start.z = cloud->points[i].z;

            float len = 0.3f; 
            end.x = start.x + normals->points[i].normal_x * len;
            end.y = start.y + normals->points[i].normal_y * len;
            end.z = start.z + normals->points[i].normal_z * len;

            arrow.points.push_back(start);
            arrow.points.push_back(end);

            arrow.scale.x = 0.02; arrow.scale.y = 0.04; arrow.scale.z = 0.1;
            arrow.color.a = 0.8; arrow.color.r = 0.0; arrow.color.g = 1.0; arrow.color.b = 0.0;

            marker_array.markers.push_back(arrow);
        }
        publisher_normals_->publish(marker_array);
    }

    std::unique_ptr<CityAnalyzer> analyzer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_objects_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_clusters_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_bboxes_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_normals_;
    rclcpp::TimerBase::SharedPtr startup_timer_;

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CityAnalyzerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}