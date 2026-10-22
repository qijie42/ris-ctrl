#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class OdomToCameraPose : public rclcpp::Node
{
public:
    OdomToCameraPose()
        : Node("odom_to_camera_pose")
    {
        // AirSim Settings.json 中 CameraDepth 的安装位置：
        // X = 0.4, Y = 0.0, Z = -0.2
        // 按 ROS base_link 的 FLU 坐标约定转换：
        // 前方 +X，左方 +Y，上方 +Z。
        this->declare_parameter<double>("camera_offset_x", 0.4);
        this->declare_parameter<double>("camera_offset_y", 0.0);
        this->declare_parameter<double>("camera_offset_z", 0.2);

        offset_x_ = this->get_parameter("camera_offset_x").as_double();
        offset_y_ = this->get_parameter("camera_offset_y").as_double();
        offset_z_ = this->get_parameter("camera_offset_z").as_double();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_in",
            rclcpp::QoS(10),
            std::bind(
                &OdomToCameraPose::odomCallback,
                this,
                std::placeholders::_1));

        camera_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "camera_pose_out",
                rclcpp::QoS(10));

        RCLCPP_INFO(
            this->get_logger(),
            "AirSim body-to-camera pose bridge started");

        RCLCPP_INFO(
            this->get_logger(),
            "Camera offset in body frame: [%.3f, %.3f, %.3f] m",
            offset_x_, offset_y_, offset_z_);
    }

private:
    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;

        // 归一化机体四元数。
        double qx = q.x;
        double qy = q.y;
        double qz = q.z;
        double qw = q.w;

        const double norm = std::sqrt(
            qx * qx + qy * qy + qz * qz + qw * qw);

        if (!std::isfinite(norm) || norm < 1e-9) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Invalid odometry quaternion; skipping camera pose");
            return;
        }

        qx /= norm;
        qy /= norm;
        qz /= norm;
        qw /= norm;

        // 与 EGO-Planner 原版 pcl_render_node.cpp 中的
        // cam02body 矩阵一致：
        //
        // [ 0   0   1 ]
        // [-1   0   0 ]
        // [ 0  -1   0 ]
        //
        // 对应 camera -> body 的四元数 (x,y,z,w)：
        // (-0.5, 0.5, -0.5, 0.5)
        constexpr double cx = -0.5;
        constexpr double cy =  0.5;
        constexpr double cz = -0.5;
        constexpr double cw =  0.5;

        // q_world_camera = q_world_body * q_body_camera
        double cqx = qw * cx + qx * cw + qy * cz - qz * cy;
        double cqy = qw * cy - qx * cz + qy * cw + qz * cx;
        double cqz = qw * cz + qx * cy - qy * cx + qz * cw;
        double cqw = qw * cw - qx * cx - qy * cy - qz * cz;

        const double cnorm = std::sqrt(
            cqx * cqx + cqy * cqy + cqz * cqz + cqw * cqw);

        cqx /= cnorm;
        cqy /= cnorm;
        cqz /= cnorm;
        cqw /= cnorm;

        // 将相机安装位置从 body 坐标系旋转到 world 坐标系。
        // v_world = v + 2*w*(q_vec x v)
        //             + 2*(q_vec x (q_vec x v))
        const double tx = offset_x_;
        const double ty = offset_y_;
        const double tz = offset_z_;

        const double cross1_x = qy * tz - qz * ty;
        const double cross1_y = qz * tx - qx * tz;
        const double cross1_z = qx * ty - qy * tx;

        const double cross2_x = qy * cross1_z - qz * cross1_y;
        const double cross2_y = qz * cross1_x - qx * cross1_z;
        const double cross2_z = qx * cross1_y - qy * cross1_x;

        const double offset_world_x =
            tx + 2.0 * (qw * cross1_x + cross2_x);
        const double offset_world_y =
            ty + 2.0 * (qw * cross1_y + cross2_y);
        const double offset_world_z =
            tz + 2.0 * (qw * cross1_z + cross2_z);

        geometry_msgs::msg::PoseStamped camera_pose;

        camera_pose.header = msg->header;
        camera_pose.header.frame_id = msg->header.frame_id;

        camera_pose.pose.position.x = p.x + offset_world_x;
        camera_pose.pose.position.y = p.y + offset_world_y;
        camera_pose.pose.position.z = p.z + offset_world_z;

        camera_pose.pose.orientation.x = cqx;
        camera_pose.pose.orientation.y = cqy;
        camera_pose.pose.orientation.z = cqz;
        camera_pose.pose.orientation.w = cqw;

        camera_pose_pub_->publish(camera_pose);

        if (!first_message_logged_) {
            RCLCPP_INFO(
                this->get_logger(),
                "Camera pose published. frame=%s, "
                "position=[%.3f, %.3f, %.3f], "
                "orientation=[%.3f, %.3f, %.3f, %.3f]",
                camera_pose.header.frame_id.c_str(),
                camera_pose.pose.position.x,
                camera_pose.pose.position.y,
                camera_pose.pose.position.z,
                cqx, cqy, cqz, cqw);

            first_message_logged_ = true;
        }
    }

    double offset_x_{0.4};
    double offset_y_{0.0};
    double offset_z_{0.2};

    bool first_message_logged_{false};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        camera_pose_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToCameraPose>());
    rclcpp::shutdown();
    return 0;
}
