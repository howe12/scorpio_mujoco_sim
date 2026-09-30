#include "terrain_analyzer/terrain_analyzer.hpp"

#include <pcl/common/transforms.h>

namespace terrain_analyzer
{

TerrainAnalyzerNode::TerrainAnalyzerNode()
    : Node("terrain_analyzer"),
      raw_map_({"elevation", "var", "var_x", "var_y", "var_xy"},
               {false, false, false, false, false}),
      fused_map_({"elevation", "upper_bound", "lower_bound"},
                 {false, false, false}),
      sdf_map_({"sdf"}, {false})
{
    // Declare parameters
    length_pos_x_ = declare_parameter("length_pos_x", 10.0);
    length_pos_y_ = declare_parameter("length_pos_y", 10.0);
    resolution_pos_ = declare_parameter("resolution_pos", 0.1);
    resolution_yaw_ = declare_parameter("resolution_yaw", 0.2);
    min_var_ = declare_parameter("min_var", 0.0001);
    max_var_ = declare_parameter("max_var", 0.05);
    min_var_hori_ = declare_parameter("min_var_hori", 0.0001);
    max_var_hori_ = declare_parameter("max_var_hori", 0.05);
    mahalanobis_threshold_ = declare_parameter("mahalanobis_threshold", 2.5);
    ellipsoid_x_ = declare_parameter("ellipsoid_x", 0.4);
    ellipsoid_y_ = declare_parameter("ellipsoid_y", 0.3);
    ellipsoid_offset_x_ = declare_parameter("ellipsoid_offset_x", 0.13);
    ellipsoid_offset_y_ = declare_parameter("ellipsoid_offset_y", 0.0);
    max_phx_ = declare_parameter("max_phx", 0.786);
    max_phy_ = declare_parameter("max_phy", 1.571);
    max_curvature_ = declare_parameter("max_curvature", 1000.0);
    weight_phi_ = declare_parameter("weight_phi", 0.2);
    weight_curvature_ = declare_parameter("weight_curvature", 0.6);
    ignore_z_min_ = declare_parameter("ignore_z_min", -2.0);
    ignore_z_max_ = declare_parameter("ignore_z_max", 0.5);
    beam_sigma_ = declare_parameter("beam_sigma", 0.008);

    std::string cloud_topic = declare_parameter("cloud_topic", std::string("/livox/lidar"));
    std::string odom_topic = declare_parameter("odom_topic", std::string("/Odometry"));
    double update_rate = declare_parameter("update_rate", 5.0);

    raw_basic_layers_ = {"elevation", "var"};
    fused_basic_layers_ = {"elevation", "lower_bound", "upper_bound"};

    // Init geometry
    Eigen::Array2d length(length_pos_x_, length_pos_y_);
    raw_map_.initGeometry(length, resolution_pos_, resolution_yaw_);
    fused_map_.initGeometry(length, resolution_pos_, resolution_yaw_);
    sdf_map_.initGeometry(length, resolution_pos_, resolution_yaw_);
    sdf_map_.add("sdf", false, 0.0f);

    // Subscribers — FAST-LIO2 用 RELIABLE 发布, 必须匹配
    auto qos_reliable = rclcpp::QoS(10).reliable();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, qos_reliable,
        std::bind(&TerrainAnalyzerNode::cloudCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos_reliable,
        std::bind(&TerrainAnalyzerNode::odomCallback, this, std::placeholders::_1));

    // Publishers (transient local for latched behavior)
    auto qos_latch = rclcpp::QoS(1).reliable().transient_local();
    fused_pub_ = create_publisher<se2_grid_msgs::msg::SE2Grid>("fused_map", qos_latch);
    sdf_pub_ = create_publisher<se2_grid_msgs::msg::SE2Grid>("sdf_map", qos_latch);

    // Timer for periodic map update + publish
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(1000.0 / update_rate)),
        std::bind(&TerrainAnalyzerNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
        "TerrainAnalyzer initialized: %.1fx%.1fm, res=%.2fm, yaw_res=%.2frad",
        length_pos_x_, length_pos_y_, resolution_pos_, resolution_yaw_);
}

void TerrainAnalyzerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    latest_body_T_ = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

    Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    latest_body_R_ = q.toRotationMatrix();

    // Extract z + roll/pitch/yaw covariance (bottom-right 4x4 of 6x6)
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            body_cov_ZR_(i, j) = msg->pose.covariance[(i + 2) * 6 + (j + 2)];

    has_odom_ = true;
}

void TerrainAnalyzerNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!has_odom_) return;

    // Deserialize point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) return;

    std::lock_guard<std::mutex> lock(mtx_);

    // Transform points to world frame (they come in lidar_link frame)
    // FAST-LIO2 publishes /cloud_registered already in camera_init (world) frame
    // So no transform needed if subscribing to /cloud_registered
    // If subscribing to /livox/lidar, need lidar→world transform
    // For now, assume input is in world frame (camera_init)

    // Move map to robot position
    Eigen::Vector2d robot_pos(latest_body_T_.x(), latest_body_T_.y());
    raw_map_.move(robot_pos);
    raw_map_.convertToDefaultStartIndex();

    // Compute per-point variance and filter by height
    int n = cloud->size();
    Eigen::VectorXf variances(n);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::VectorXf filtered_vars;

    float beam_sigma2 = beam_sigma_ * beam_sigma_;
    Eigen::Vector3d sensor_pos = latest_body_T_; // approximate: lidar ≈ body for sim

    for (int i = 0; i < n; i++)
    {
        auto& pt = cloud->points[i];
        Eigen::Vector3d p_world(pt.x, pt.y, pt.z);

        // Transform to body frame for height filtering
        Eigen::Vector3d p_body = latest_body_R_.transpose() * (p_world - latest_body_T_);

        if (p_body.z() < ignore_z_min_ || p_body.z() > ignore_z_max_)
            continue;

        // Variance computation (simplified: use beam noise + localization uncertainty)
        double dist = p_body.norm();
        float sigma_s = static_cast<float>(dist * dist * beam_sigma2);
        float cov_z = static_cast<float>(body_cov_ZR_(0, 0));
        float var = std::max(sigma_s + cov_z, static_cast<float>(min_var_));

        filtered->push_back(pt);
        // We'll build filtered_vars after the loop
    }

    // Rebuild variances for filtered points
    filtered_vars.resize(filtered->size());
    int fi = 0;
    for (int i = 0; i < n && fi < (int)filtered->size(); i++)
    {
        auto& pt = cloud->points[i];
        Eigen::Vector3d p_world(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_body = latest_body_R_.transpose() * (p_world - latest_body_T_);
        if (p_body.z() < ignore_z_min_ || p_body.z() > ignore_z_max_)
            continue;

        double dist = p_body.norm();
        float sigma_s = static_cast<float>(dist * dist * beam_sigma_ * beam_sigma_);
        float cov_z = static_cast<float>(std::max(body_cov_ZR_(0, 0), 0.001));
        filtered_vars(fi) = std::max(sigma_s + cov_z, static_cast<float>(min_var_));
        fi++;
    }

    // Kalman fusion into raw_map
    addPoints(filtered, filtered_vars);

    // Inpaint NaN cells
    inpaintSpiral(raw_map_, "elevation");

    // Fuse raw → fused
    fuseAll();

    // TODO: computeTraversability() 暂时禁用 (CPU 版太慢, 320k SE(2) cells)
    // computeTraversability();

    // SDF 生成: 从高程图 + 障碍物检测
    // 1. 有高程数据且无高点 → free (SDF > 0)
    // 2. 无高程数据 → unknown (SDF = 0.5)
    // 3. 格子内有足够多的高点 (z > obstacle_z_threshold) → occupied (SDF < 0)
    {
        auto size = fused_map_.getSizePos();
        auto& sdf_data = sdf_map_["sdf"][0];
        auto& elev_data = fused_map_["elevation"][0];

        // 统计每个格子内的高点数量 (障碍物检测)
        float obstacle_z_threshold = 0.15f;  // body 系 z > 0.15m 视为障碍物 (车高 ~0.3m)
        int obstacle_point_threshold = 3;     // 至少 3 个高点才标记为 occupied
        Eigen::MatrixXi hit_count(size(0), size(1));
        hit_count.setZero();

        // 遍历当前帧点云, 统计高点
        for (size_t i = 0; i < filtered->size(); i++)
        {
            auto& pt = filtered->points[i];
            Eigen::Vector3d p_world(pt.x, pt.y, pt.z);
            Eigen::Vector3d p_body = latest_body_R_.transpose() * (p_world - latest_body_T_);

            if (p_body.z() > obstacle_z_threshold)
            {
                // 转换到地图格子坐标
                Eigen::Vector3d pos3d(p_world.x(), p_world.y(), 0.0);
                Eigen::Array3i idx;
                if (fused_map_.pos2Index(pos3d, idx))
                {
                    int r = idx(0), c = idx(1);
                    if (r >= 0 && r < size(0) && c >= 0 && c < size(1))
                        hit_count(r, c)++;
                }
            }
        }

        // 生成 SDF
        for (int r = 0; r < size(0); r++)
        {
            for (int c = 0; c < size(1); c++)
            {
                if (hit_count(r, c) >= obstacle_point_threshold)
                    sdf_data(r, c) = -1.0f;   // occupied (碰撞)
                else if (std::isfinite(elev_data(r, c)))
                    sdf_data(r, c) = 1.0f;    // free (可通行)
                else
                    sdf_data(r, c) = 0.5f;    // unknown
            }
        }
    }

    frame_count_++;
    if (frame_count_ % 10 == 0)
    {
        RCLCPP_INFO(get_logger(), "Frame %d: %zu pts, map pos=(%.1f,%.1f)",
                    frame_count_, filtered->size(),
                    raw_map_.getPosition().x(), raw_map_.getPosition().y());
    }
}

bool TerrainAnalyzerNode::addPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                     Eigen::VectorXf& variances)
{
    auto& elev = raw_map_["elevation"][0];
    auto& var = raw_map_["var"][0];
    auto& varX = raw_map_["var_x"][0];
    auto& varY = raw_map_["var_y"][0];
    auto& varXY = raw_map_["var_xy"][0];

    for (unsigned int i = 0; i < cloud->points.size(); i++)
    {
        auto& pt = cloud->points[i];
        Eigen::Array3i index;
        Eigen::Vector3d position(pt.x, pt.y, 0);

        if (!raw_map_.pos2Index(position, index))
            continue;

        float& elevation = elev(index(0), index(1));
        float& variance = var(index(0), index(1));
        float& vX = varX(index(0), index(1));
        float& vY = varY(index(0), index(1));
        float& vXY = varXY(index(0), index(1));
        float ptVar = std::max(std::abs(variances(i)), static_cast<float>(min_var_hori_));

        if (!raw_map_.isValid(index, raw_basic_layers_))
        {
            // First observation: initialize
            elevation = pt.z;
            variance = ptVar;
            vX = static_cast<float>(min_var_hori_);
            vY = static_cast<float>(min_var_hori_);
            vXY = 0.0f;
            continue;
        }

        // Mahalanobis distance test
        float dM = std::abs(pt.z - elevation) / std::sqrt(variance);
        if (dM > mahalanobis_threshold_)
            continue;  // outlier, skip

        // Kalman fusion
        elevation = (variance * pt.z + ptVar * elevation) / (variance + ptVar);
        variance = (ptVar * variance) / (ptVar + variance);

        vX = static_cast<float>(min_var_hori_);
        vY = static_cast<float>(min_var_hori_);
        vXY = 0.0f;
    }

    // Clamp variances
    raw_map_["var"][0] = raw_map_["var"][0].unaryExpr(ClampVar(min_var_, max_var_));
    raw_map_["var_x"][0] = raw_map_["var_x"][0].unaryExpr(ClampVar(min_var_hori_, max_var_hori_));
    raw_map_["var_y"][0] = raw_map_["var_y"][0].unaryExpr(ClampVar(min_var_hori_, max_var_hori_));

    return true;
}

bool TerrainAnalyzerNode::fuseAll()
{
    // 简化版: 直接从 raw_map 复制到 fused_map (跳过椭圆加权融合)
    // 原版用 95.45% 置信椭圆做多格加权融合, CPU 上太慢
    // 后续可优化为 OpenMP 并行或 GPU

    if (raw_map_.getPosition() != fused_map_.getPosition())
    {
        fused_map_.move(raw_map_.getPosition());
        fused_map_.convertToDefaultStartIndex();
    }

    auto size = raw_map_.getSizePos();
    auto& raw_elev = raw_map_["elevation"][0];
    auto& fused_elev = fused_map_["elevation"][0];
    auto& fused_lo = fused_map_["lower_bound"][0];
    auto& fused_hi = fused_map_["upper_bound"][0];

    for (int r = 0; r < size(0); r++)
    {
        for (int c = 0; c < size(1); c++)
        {
            if (std::isfinite(raw_elev(r, c)))
            {
                fused_elev(r, c) = raw_elev(r, c);
                fused_lo(r, c) = raw_elev(r, c);
                fused_hi(r, c) = raw_elev(r, c);
            }
        }
    }

    return true;
}

void TerrainAnalyzerNode::inpaintSpiral(se2_grid::SE2Grid& map, const std::string& layer)
{
    auto& data = map[layer][0];
    auto size = map.getSizePos();
    int max_radius = 5;  // 限制搜索半径 (原版无限制, CPU 上太慢)

    for (int r = 0; r < size(0); r++)
    {
        for (int c = 0; c < size(1); c++)
        {
            if (std::isfinite(data(r, c)))
                continue;

            for (int radius = 1; radius <= max_radius; radius++)
            {
                bool found = false;
                for (int dr = -radius; dr <= radius && !found; dr++)
                {
                    for (int dc = -radius; dc <= radius && !found; dc++)
                    {
                        if (std::abs(dr) != radius && std::abs(dc) != radius)
                            continue;
                        int nr = r + dr, nc = c + dc;
                        if (nr < 0 || nr >= size(0) || nc < 0 || nc >= size(1))
                            continue;
                        if (std::isfinite(data(nr, nc)))
                        {
                            data(r, c) = data(nr, nc);
                            found = true;
                        }
                    }
                }
                if (found) break;
            }
        }
    }
}

void TerrainAnalyzerNode::computeTraversability()
{
    // Algorithm 1: For each SE(2) cell, evaluate traversability risk
    // This is the CPU version of the GPU kernel compute_map_se2

    int size_yaw = fused_map_.getSizeYaw();
    auto size = fused_map_.getSizePos();

    // Add risk layer if not exists (with SO(2) dimension)
    if (!fused_map_.exists("risk"))
    {
        fused_map_.add("risk", true, 0.0f);
    }
    if (!fused_map_.exists("zbx"))
    {
        fused_map_.add("zbx", true, 0.0f);
    }
    if (!fused_map_.exists("zby"))
    {
        fused_map_.add("zby", true, 0.0f);
    }

    double ell_a = ellipsoid_x_ * 0.5;  // semi-axis
    double ell_b = ellipsoid_y_ * 0.5;

    for (int r = 0; r < size(0); r++)
    {
        for (int c = 0; c < size(1); c++)
        {
            Eigen::Array3i base_idx(r, c, 0);
            Eigen::Vector3d grid_center;
            fused_map_.index2Pos(base_idx, grid_center);

            for (int iy = 0; iy < size_yaw; iy++)
            {
                double yaw = iy * resolution_yaw_;
                Eigen::Array3i se2_idx(r, c, iy);

                // Body axes
                double xb_x = std::cos(yaw), xb_y = std::sin(yaw);
                double yb_x = -std::sin(yaw), yb_y = std::cos(yaw);

                // Ellipse center offset
                double ecx = grid_center.x() + ellipsoid_offset_x_ * xb_x + ellipsoid_offset_y_ * yb_x;
                double ecy = grid_center.y() + ellipsoid_offset_x_ * xb_y + ellipsoid_offset_y_ * yb_y;

                // Collect points inside ellipse
                std::vector<Eigen::Vector3d> pts;
                int search_r = static_cast<int>(std::ceil(std::max(ell_a, ell_b) / resolution_pos_)) + 1;

                for (int dr = -search_r; dr <= search_r; dr++)
                {
                    for (int dc = -search_r; dc <= search_r; dc++)
                    {
                        Eigen::Array3i nidx(r + dr, c + dc, 0);
                        if (!fused_map_.isValid(nidx, fused_basic_layers_))
                            continue;

                        Eigen::Vector3d npos;
                        fused_map_.index2Pos(nidx, npos);
                        double dx = npos.x() - ecx;
                        double dy = npos.y() - ecy;

                        // Rotate into ellipse frame
                        double rx = xb_x * dx + xb_y * dy;
                        double ry = yb_x * dx + yb_y * dy;

                        if ((rx * rx) / (ell_a * ell_a) + (ry * ry) / (ell_b * ell_b) <= 1.0)
                        {
                            float elev = fused_map_.at("elevation", nidx);
                            pts.push_back(Eigen::Vector3d(npos.x(), npos.y(), elev));
                        }
                    }
                }

                float risk = 1.0f;
                float zbx_val = 0.0f, zby_val = 0.0f;

                if (pts.size() > 7)
                {
                    // Compute covariance
                    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
                    for (auto& p : pts) mean += p;
                    mean /= pts.size();

                    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
                    for (auto& p : pts)
                    {
                        Eigen::Vector3d d = p - mean;
                        cov += d * d.transpose();
                    }
                    cov /= pts.size();

                    // Jacobi eigenvalue decomposition (use Eigen)
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
                    Eigen::Vector3d eigenvalues = solver.eigenvalues();
                    Eigen::Matrix3d eigenvectors = solver.eigenvectors();

                    // Smallest eigenvalue → surface normal
                    int min_idx = 0;
                    if (eigenvalues(1) < eigenvalues(min_idx)) min_idx = 1;
                    if (eigenvalues(2) < eigenvalues(min_idx)) min_idx = 2;

                    Eigen::Vector3d normal = eigenvectors.col(min_idx);
                    if (normal.z() < 0) normal = -normal;  // ensure upward

                    // Curvature = 3 * λ_min / Σλ
                    double curvature = 3.0 * eigenvalues(min_idx) /
                                       (eigenvalues.sum() + 1e-10);

                    zbx_val = static_cast<float>(normal.x());
                    zby_val = static_cast<float>(normal.y());

                    // Recompute body axes with estimated normal
                    Eigen::Vector3d zb = normal;
                    Eigen::Vector3d xyaw(xb_x, xb_y, 0);
                    Eigen::Vector3d yb_new = zb.cross(xyaw).normalized();
                    Eigen::Vector3d xb_new = yb_new.cross(zb).normalized();

                    // Tilt angles
                    double phi_x = std::abs(std::asin(std::min(1.0, std::abs(Eigen::Vector3d::UnitZ().dot(xb_new)))));
                    double phi_y = std::abs(std::asin(std::min(1.0, std::abs(Eigen::Vector3d::UnitZ().dot(yb_new)))));

                    // Risk computation
                    double risk_curv = curvature / max_curvature_;
                    double risk_phx = phi_x / max_phx_;
                    double risk_phy = phi_y / max_phy_;

                    risk = static_cast<float>(
                        weight_curvature_ * risk_curv +
                        weight_phi_ * risk_phx +
                        weight_phi_ * risk_phy);

                    // Hard limits
                    if (curvature > max_curvature_ || phi_x > max_phx_ || phi_y > max_phy_)
                        risk = 1.0f;

                    risk = std::min(risk, 1.0f);
                }

                // Store results
                fused_map_["risk"][iy](r, c) = risk;
                fused_map_["zbx"][iy](r, c) = zbx_val;
                fused_map_["zby"][iy](r, c) = zby_val;

                // SDF: binary obstacle map
                sdf_map_["sdf"][0](r, c) = (risk > 0.5f) ? 1.0f : 0.0f;
            }
        }
    }
}

float TerrainAnalyzerNode::cdf(float x, float mean, float stddev)
{
    return 0.5f * erfc(-(x - mean) / (stddev * std::sqrt(2.0)));
}

void TerrainAnalyzerNode::publishMap(const se2_grid::SE2Grid& map,
                                      rclcpp::Publisher<se2_grid_msgs::msg::SE2Grid>::SharedPtr pub)
{
    se2_grid_msgs::msg::SE2Grid msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "camera_init";

    msg.info.pos_resolution = map.getResolutionPos();
    msg.info.yaw_resolution = map.getResolutionYaw();
    msg.info.length_x = map.getLengthPos()(0);
    msg.info.length_y = map.getLengthPos()(1);
    msg.info.length_yaw = map.getLengthYaw();

    auto pos = map.getPosition();
    msg.info.pose.position.x = pos.x();
    msg.info.pose.position.y = pos.y();
    msg.info.pose.orientation.w = 1.0;

    auto layers = map.getLayers();
    auto has_so2 = map.getHasSO2();

    for (auto& layer : layers)
    {
        msg.layers.push_back(layer);
        msg.has_so2.push_back(has_so2.at(layer));

        auto& data_vec = map[layer];
        std_msgs::msg::Float32MultiArray arr;

        // Flatten all yaw slices into one array
        for (auto& mat : data_vec)
        {
            arr.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            arr.layout.dim.back().size = mat.rows();
            arr.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
            arr.layout.dim.back().size = mat.cols();

            for (int r = 0; r < mat.rows(); r++)
                for (int c = 0; c < mat.cols(); c++)
                    arr.data.push_back(mat(r, c));
        }
        msg.data.push_back(arr);
    }

    auto si = map.getStartIndex();
    msg.start_index_row = static_cast<uint16_t>(si(0));
    msg.start_index_col = static_cast<uint16_t>(si(1));

    pub->publish(msg);
}

void TerrainAnalyzerNode::timerCallback()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (frame_count_ == 0) return;

    publishMap(fused_map_, fused_pub_);
    publishMap(sdf_map_, sdf_pub_);
}

} // namespace terrain_analyzer

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<terrain_analyzer::TerrainAnalyzerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
