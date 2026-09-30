#pragma once

#include <se2_grid_core/SE2Grid.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <se2_grid_msgs/msg/se2_grid.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <mutex>
#include <cmath>

namespace terrain_analyzer
{

struct ClampVar
{
    float minVar, maxVar;
    ClampVar(float mn, float mx) : minVar(mn), maxVar(mx) {}
    float operator()(float x) const
    {
        return x < minVar ? minVar : (x > maxVar ? std::numeric_limits<float>::infinity() : x);
    }
};

class TerrainAnalyzerNode : public rclcpp::Node
{
public:
    TerrainAnalyzerNode();

private:
    // Callbacks
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Core algorithms
    bool addPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::VectorXf& variances);
    bool fuseAll();
    void computeTraversability();
    void inpaintSpiral(se2_grid::SE2Grid& map, const std::string& layer);

    // Helpers
    float cdf(float x, float mean, float stddev);
    void publishMap(const se2_grid::SE2Grid& map, rclcpp::Publisher<se2_grid_msgs::msg::SE2Grid>::SharedPtr pub);

    // Parameters
    double length_pos_x_ = 10.0;
    double length_pos_y_ = 10.0;
    double resolution_pos_ = 0.1;
    double resolution_yaw_ = 0.2;
    double min_var_ = 0.0001;
    double max_var_ = 0.05;
    double min_var_hori_ = 0.0001;
    double max_var_hori_ = 0.05;
    double mahalanobis_threshold_ = 2.5;
    double ellipsoid_x_ = 0.4;
    double ellipsoid_y_ = 0.3;
    double ellipsoid_offset_x_ = 0.13;
    double ellipsoid_offset_y_ = 0.0;
    double max_phx_ = 0.786;   // ~π/4
    double max_phy_ = 1.571;   // ~π/2
    double max_curvature_ = 1000.0;
    double weight_phi_ = 0.2;
    double weight_curvature_ = 0.6;
    double ignore_z_min_ = -2.0;
    double ignore_z_max_ = 0.5;
    double beam_sigma_ = 0.008;

    // Data
    se2_grid::SE2Grid raw_map_;
    se2_grid::SE2Grid fused_map_;
    se2_grid::SE2Grid sdf_map_;

    std::vector<std::string> raw_basic_layers_;
    std::vector<std::string> fused_basic_layers_;

    // Latest odometry
    Eigen::Vector3d latest_body_T_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d latest_body_R_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix4d body_cov_ZR_ = Eigen::Matrix4d::Identity() * 0.01;
    bool has_odom_ = false;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<se2_grid_msgs::msg::SE2Grid>::SharedPtr fused_pub_;
    rclcpp::Publisher<se2_grid_msgs::msg::SE2Grid>::SharedPtr sdf_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mtx_;
    int frame_count_ = 0;
};

} // namespace terrain_analyzer
