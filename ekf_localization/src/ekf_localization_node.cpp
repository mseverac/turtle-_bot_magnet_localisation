// ekf_localization_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>
#include <optional>

using std::placeholders::_1;

// Shared state variables
Eigen::Vector3d x_ = Eigen::Vector3d::Zero();
Eigen::Matrix3d P_ = Eigen::Matrix3d::Identity();
Eigen::Matrix2d Qgamma_;
Eigen::Matrix2d Qwheels_;
Eigen::Matrix2d Qbeta_;
Eigen::Matrix2d jointToCartesian_;
Eigen::Matrix2d cartesianToJoint_;

// Mutex for protecting shared state variables
std::mutex mtx;

// Parameters
double sigma_x, sigma_y, sigma_theta;
double sigma_x_measurement, sigma_y_measurement, sigma_tuning;
double mahalanobis_threshold_;
double initial_x, initial_y, initial_theta;
double wheel_radius_;       // [m]
double track_gauge_;        // [m]
double magnet_spacing_x_;   // [m]
double magnet_spacing_y_;   // [m]

// helper to convert degrees->radians if necessary (heuristic)
inline double convert_wheel_angle_if_needed(double a) {
    // if magnitude is larger than 2pi (≈6.28), assume input in degrees and convert
    if (std::abs(a) > 2.0 * M_PI) {
        return a * M_PI / 180.0;
    }
    return a;
}

void declare_and_get_parameters(rclcpp::Node::SharedPtr node) {
    // DEFAULTS are now in METERS and radians where relevant
    node->declare_parameter<double>("sigma_x", 0.3/1000);
    node->declare_parameter<double>("sigma_y", 0.3/1000);
    node->declare_parameter<double>("sigma_theta", 0.0698);
    node->declare_parameter<double>("sigma_x_measurement", 5.77/1000);
    node->declare_parameter<double>("sigma_y_measurement", 2.8868/1000);
    node->declare_parameter<double>("sigma_tuning", 0.14);
    // Mahalanobis threshold (no scaling) - typical 95% -> 2.4477
    node->declare_parameter<double>("mahalanobis_threshold", 2.4477);

    node->declare_parameter<double>("initial_x", 0.0);
    node->declare_parameter<double>("initial_y", 0.0);  
    node->declare_parameter<double>("initial_theta", 0.0);

    // LENGTHS: declare in meters (provide meters in YAML), but keep heuristic for mm->m if someone wrote mm
    node->declare_parameter<double>("wheel_radius", 0.0215);    // default 0.033 m (33 mm)
    node->declare_parameter<double>("track_gauge", 0.112);     // default 0.283 m (283 mm)
    node->declare_parameter<double>("magnet_spacing_x", 0.055); // default 0.055 m (55 mm)
    node->declare_parameter<double>("magnet_spacing_y", 0.055);

    node->get_parameter("sigma_x", sigma_x);
    node->get_parameter("sigma_y", sigma_y);
    node->get_parameter("sigma_theta", sigma_theta);
    node->get_parameter("sigma_x_measurement", sigma_x_measurement);
    node->get_parameter("sigma_y_measurement", sigma_y_measurement);
    node->get_parameter("sigma_tuning", sigma_tuning);
    node->get_parameter("mahalanobis_threshold", mahalanobis_threshold_);
    node->get_parameter("initial_x", initial_x);
    node->get_parameter("initial_y", initial_y);
    node->get_parameter("initial_theta", initial_theta);
    node->get_parameter("wheel_radius", wheel_radius_);
    node->get_parameter("track_gauge", track_gauge_);
    node->get_parameter("magnet_spacing_x", magnet_spacing_x_);
    node->get_parameter("magnet_spacing_y", magnet_spacing_y_);

    // Sanity checks and warnings if values look like mm (heuristic)
    if (wheel_radius_ > 1.0) {
        RCLCPP_WARN(node->get_logger(), "wheel_radius looks > 1.0 — assuming mm, converting to meters.");
        wheel_radius_ /= 1000.0;
    }
    if (track_gauge_ > 1.0) {
        RCLCPP_WARN(node->get_logger(), "track_gauge looks > 1.0 — assuming mm, converting to meters.");
        track_gauge_ /= 1000.0;
    }
    if (magnet_spacing_x_ > 1.0) {
        RCLCPP_WARN(node->get_logger(), "magnet_spacing_x looks > 1.0 — assuming mm, converting to meters.");
        magnet_spacing_x_ /= 1000.0;
    }
    if (magnet_spacing_y_ > 1.0) {
        RCLCPP_WARN(node->get_logger(), "magnet_spacing_y looks > 1.0 — assuming mm, converting to meters.");
        magnet_spacing_y_ /= 1000.0;
    }

    // init state and covariance
    x_ << initial_x, initial_y, initial_theta;
    P_ = Eigen::Matrix3d::Zero();
    P_(0, 0) = sigma_x * sigma_x;
    P_(1, 1) = sigma_y * sigma_y;
    P_(2, 2) = sigma_theta * sigma_theta;

    // Initialize jointToCartesian and cartesianToJoint matrices
    // Map wheel rotational increments [rad] -> [linear, angular] increments
    jointToCartesian_ << wheel_radius_ / 2.0, wheel_radius_ / 2.0,
                         wheel_radius_ / track_gauge_, -wheel_radius_ / track_gauge_;

    // compute inverse safely
    if (jointToCartesian_.determinant() == 0.0) {
        RCLCPP_WARN(node->get_logger(), "jointToCartesian matrix singular! Check wheel_radius/track_gauge.");
        cartesianToJoint_ = Eigen::Matrix2d::Identity();
    } else {
        cartesianToJoint_ = jointToCartesian_.inverse();
    }

    // Initialize process and measurement noise covariance matrices
    Qwheels_ = sigma_tuning * sigma_tuning * Eigen::Matrix2d::Identity();
    Qbeta_ = jointToCartesian_ * Qwheels_ * jointToCartesian_.transpose();

    Qgamma_ = Eigen::Matrix2d::Zero();
    Qgamma_(0, 0) = sigma_x_measurement * sigma_x_measurement;
    Qgamma_(1, 1) = sigma_y_measurement * sigma_y_measurement;

    RCLCPP_INFO(node->get_logger(), "EKF initialized: wheel_radius=%.6f m, track_gauge=%.6f m", wheel_radius_, track_gauge_);
    // Summary logs of all relevant parameters and matrices
    RCLCPP_INFO(node->get_logger(), "EKF initialized: wheel_radius=%.6f m, track_gauge=%.6f m", wheel_radius_, track_gauge_);
    RCLCPP_INFO(node->get_logger(),
                "Parameters: sigma_x=%.6e, sigma_y=%.6e, sigma_theta=%.6e, sigma_x_measurement=%.6e, sigma_y_measurement=%.6e, sigma_tuning=%.6e, mahalanobis_threshold=%.6e",
                sigma_x, sigma_y, sigma_theta, sigma_x_measurement, sigma_y_measurement, sigma_tuning, mahalanobis_threshold_);
    RCLCPP_INFO(node->get_logger(), "Initial state: initial_x=%.6f, initial_y=%.6f, initial_theta=%.6f", initial_x, initial_y, initial_theta);
    RCLCPP_INFO(node->get_logger(), "Magnet spacing: magnet_spacing_x=%.6f m, magnet_spacing_y=%.6f m", magnet_spacing_x_, magnet_spacing_y_);
    RCLCPP_INFO(node->get_logger(), "Wheel geometry: wheel_radius=%.6f m, track_gauge=%.6f m", wheel_radius_, track_gauge_);

    // Log matrices using stream-style macros
    RCLCPP_INFO_STREAM(node->get_logger(), "Initial covariance P:\n" << P_);
    RCLCPP_INFO_STREAM(node->get_logger(), "jointToCartesian:\n" << jointToCartesian_);
    RCLCPP_INFO_STREAM(node->get_logger(), "cartesianToJoint:\n" << cartesianToJoint_);
    RCLCPP_INFO_STREAM(node->get_logger(), "Qwheels:\n" << Qwheels_);
    RCLCPP_INFO_STREAM(node->get_logger(), "Qbeta:\n" << Qbeta_);
    RCLCPP_INFO_STREAM(node->get_logger(), "Qgamma (measurement noise):\n" << Qgamma_);
   
}

// Joint state callback function (now also publishes PoseWithCovarianceStamped)
void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg,
                          rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub,
                          rclcpp::Node::SharedPtr node) {

    std::lock_guard<std::mutex> lock(mtx); // Lock the mutex

    // Find the left and right wheel positions
    auto left_it = std::find(msg->name.begin(), msg->name.end(), "wheel_left_joint");
    auto right_it = std::find(msg->name.begin(), msg->name.end(), "wheel_right_joint");

    if (left_it == msg->name.end() || right_it == msg->name.end()) {
        RCLCPP_WARN(node->get_logger(), "Wheel joint names not found in JointState (expected 'wheel_left_joint' and 'wheel_right_joint').");
        return;
    }

    size_t left_index = std::distance(msg->name.begin(), left_it);
    size_t right_index = std::distance(msg->name.begin(), right_it);

    double left_pos_raw = msg->position[left_index];
    double right_pos_raw = msg->position[right_index];

    // convert if needed (heuristic: if values > 2pi assume degrees)
    double left_pos = convert_wheel_angle_if_needed(left_pos_raw);
    double right_pos = convert_wheel_angle_if_needed(right_pos_raw);

    // static prev positions initialized on first callback
    static bool first = true;
    static double prev_left_pos_ = 0.0;
    static double prev_right_pos_ = 0.0;
    static rclcpp::Time last_time_ = rclcpp::Clock().now();

    if (first) {
        prev_left_pos_ = left_pos;
        prev_right_pos_ = right_pos;
        last_time_ = rclcpp::Clock().now();
        first = false;
        RCLCPP_DEBUG(node->get_logger(), "First joint_state received, initializing previous positions.");
        // Publish initial pose as well (optional)
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = rclcpp::Clock().now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = x_(0);
        pose_msg.pose.pose.position.y = x_(1);
        pose_msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, x_(2));
        pose_msg.pose.pose.orientation = tf2::toMsg(q);
        // fill 6x6 covariance (only top-left 3x3 used)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                pose_msg.pose.covariance[i * 6 + j] = P_(i, j);
            }
        }
        pose_pub->publish(pose_msg);
        return; // no prediction on first message
    }

    rclcpp::Time current_time = rclcpp::Clock().now();
    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0) dt = 1e-6; // avoid zero

    // Calculate wheel displacements (delta angles * radius) -> linear distance
    double delta_left = left_pos - prev_left_pos_;
    double delta_right = right_pos - prev_right_pos_;

    prev_left_pos_ = left_pos;
    prev_right_pos_ = right_pos;

    double dist_left = delta_left * wheel_radius_;
    double dist_right = delta_right * wheel_radius_;

    double delta_d = (dist_left + dist_right) / 2.0;
    double delta_theta = (dist_right - dist_left) / track_gauge_;

    // Update the state estimate (prediction step)
    double theta = x_(2);
    x_(0) += delta_d * std::cos(theta);
    x_(1) += delta_d * std::sin(theta);
    x_(2) += delta_theta;

    // Normalize theta to [-pi, pi]
    x_(2) = std::atan2(std::sin(x_(2)), std::cos(x_(2)));

    // Jacobians
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A(0, 2) = -delta_d * std::sin(theta);
    A(1, 2) =  delta_d * std::cos(theta);

    Eigen::Matrix<double, 3, 2> B;
    B << std::cos(theta), 0,
         std::sin(theta), 0,
         0, 1;

    Eigen::Matrix3d Qalpha_ = Eigen::Matrix3d::Zero();

    // Propagate covariance
    P_ = A * P_ * A.transpose() + B * Qbeta_ * B.transpose() + Qalpha_;

    last_time_ = current_time;

    // --- Publish PoseWithCovarianceStamped after prediction ---
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = current_time;
    pose_msg.header.frame_id = "map";

    pose_msg.pose.pose.position.x = x_(0);
    pose_msg.pose.pose.position.y = x_(1);
    pose_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, x_(2));
    pose_msg.pose.pose.orientation = tf2::toMsg(q);

    // Fill covariance into the 6x6 pose.covariance (only top-left 3x3 used)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pose_msg.pose.covariance[i * 6 + j] = P_(i, j);
        }
    }

    pose_pub->publish(pose_msg);
    RCLCPP_DEBUG(node->get_logger(), "Published pose after prediction: x=%.3f y=%.3f theta=%.3f", x_(0), x_(1), x_(2));
}


// Measurement callback function
void measurement_callback(const geometry_msgs::msg::Point::SharedPtr msg,
                          rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub,
                          rclcpp::Node::SharedPtr node) {

    RCLCPP_INFO_STREAM(node->get_logger(), "in cb2");

    std::lock_guard<std::mutex> lock(mtx); // Lock the mutex

    Eigen::Vector2d z_(msg->x/1000, msg->y/1000); // convert mm->m

    // Build transforms
    Eigen::Matrix3d oTm;
    oTm << std::cos(x_(2)), -std::sin(x_(2)), x_(0),
           std::sin(x_(2)),  std::cos(x_(2)), x_(1),
           0,                0,               1;
    Eigen::Matrix3d mTo = oTm.inverse();

    // Measurement in robot frame
    Eigen::Vector3d mMeasMagnet(z_(0), z_(1), 1.0);

    // Position in world frame
    Eigen::Vector3d oMeasMagnet = oTm * mMeasMagnet;

    // Snap to nearest magnet grid (magnet_spacing_* in meters)
    Eigen::Vector3d oRealMagnet;
    // protect against 0 spacing
    if (magnet_spacing_x_ <= 0.0 || magnet_spacing_y_ <= 0.0) {
        RCLCPP_WARN(node->get_logger(), "magnet spacing <= 0, skipping measurement.");
        return;
    }
    oRealMagnet(0) = std::round(oMeasMagnet(0) / magnet_spacing_x_) * magnet_spacing_x_;
    oRealMagnet(1) = std::round(oMeasMagnet(1) / magnet_spacing_y_) * magnet_spacing_y_;
    oRealMagnet(2) = oMeasMagnet(2);

    // Back to robot frame
    Eigen::Vector3d mRealMagnet = mTo * oRealMagnet;
    Eigen::Vector2d Yhat = mRealMagnet.head<2>();

    // Jacobian C (dh/dx)
    Eigen::Matrix<double, 2, 3> C;
    double c = std::cos(x_(2));
    double s = std::sin(x_(2));
    double dx = oRealMagnet(0) - x_(0);
    double dy = oRealMagnet(1) - x_(1);

    // careful with signs: derivative of R^T*(oReal - x)
    C << -c, -s, -s * dx + c * dy,
          s, -c, -s * dy - c * dx;

    // Innovation
    Eigen::Vector2d innov = z_ - Yhat;

    // Innovation covariance
    Eigen::Matrix2d S = C * P_ * C.transpose() + Qgamma_;

    // check invertibility (use fullPivLu)
    Eigen::FullPivLU<Eigen::Matrix2d> lu(S);
    if (!lu.isInvertible()) {
        RCLCPP_WARN(node->get_logger(), "Innovation covariance S not invertible, skipping measurement update.");
        return;
    }

    double dMaha = std::sqrt(innov.transpose() * S.inverse() * innov);

    if (dMaha <= mahalanobis_threshold_) {
        // Kalman gain
        Eigen::Matrix<double, 3, 2> K = P_ * C.transpose() * S.inverse();

        // Update
        x_ = x_ + K * innov;
        P_ = (Eigen::Matrix3d::Identity() - K * C) * P_;

        // Normalize theta
        x_(2) = std::atan2(std::sin(x_(2)), std::cos(x_(2)));

        // Publish updated pose
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.stamp = rclcpp::Clock().now();
        pose_msg.header.frame_id = "map";

        pose_msg.pose.pose.position.x = x_(0);
        pose_msg.pose.pose.position.y = x_(1);
        pose_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, x_(2));
        pose_msg.pose.pose.orientation = tf2::toMsg(q);

        // Fill covariance into the 6x6 pose.covariance (only top-left 3x3 used)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                pose_msg.pose.covariance[i * 6 + j] = P_(i, j);
            }
        }

        pose_pub->publish(pose_msg);
        RCLCPP_INFO_STREAM(node->get_logger(), "pubed");
        RCLCPP_INFO(node->get_logger(), "z=(%.3f,%.3f) Yhat=(%.3f,%.3f) innov=(%.3f,%.3f) S_diag=(%.6e,%.6e) dMaha=%.3f",
    z_(0), z_(1), Yhat(0), Yhat(1), innov(0), innov(1), S(0,0), S(1,1), dMaha);


    } else {
        RCLCPP_INFO_STREAM(node->get_logger(),
                   "Measurement rejected by Mahalanobis (d=" << dMaha
                   << " threshold=" << mahalanobis_threshold_ << ")");
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("ekf_localization_node");

    declare_and_get_parameters(node);

    auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("robot_pose", 10);

    // subscriptions: capture node shared_ptr to log and for parameter checks
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, [pose_pub, node](const sensor_msgs::msg::JointState::SharedPtr msg) {
            joint_state_callback(msg, pose_pub, node);
        });

    auto measurement_sub = node->create_subscription<geometry_msgs::msg::Point>(
        "measurement_topic", 10, [pose_pub,node](const geometry_msgs::msg::Point::SharedPtr msg) {
            measurement_callback(msg, pose_pub, node);
        });

    RCLCPP_INFO(node->get_logger(), "EKF node started.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
