#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using ImageCapture = msr::airlib::ImageCaptureBase;

class AirSimImageBridge : public rclcpp::Node
{
public:
    AirSimImageBridge()
        : Node("airsim_image_bridge")
    {
        this->declare_parameter<std::string>("airsim_host", "127.0.0.1");
        this->declare_parameter<std::string>("vehicle_name", "PX4");
        this->declare_parameter<std::string>("left_camera", "CameraImage");
        this->declare_parameter<std::string>("right_camera", "CameraRight");
        this->declare_parameter<std::string>("depth_camera", "CameraDepth");
        this->declare_parameter<double>("camera_rate", 30.0);

        const auto host =
            this->get_parameter("airsim_host").as_string();

        vehicle_name_ =
            this->get_parameter("vehicle_name").as_string();

        const auto left_camera =
            this->get_parameter("left_camera").as_string();

        const auto right_camera =
            this->get_parameter("right_camera").as_string();

        const auto depth_camera =
            this->get_parameter("depth_camera").as_string();

        double camera_rate =
            this->get_parameter("camera_rate").as_double();

        if (camera_rate <= 0.0) {
            camera_rate = 30.0;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Connecting to AirSim: %s, vehicle=%s",
            host.c_str(), vehicle_name_.c_str());

        client_ =
            std::make_unique<msr::airlib::MultirotorRpcLibClient>(host);

        // 仅等待 RPC 连接状态，不执行可能阻塞的版本查询。
        const auto connect_deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(10);

        while (client_->getConnectionState() !=
               msr::airlib::RpcLibClientBase::ConnectionState::Connected) {
            if (std::chrono::steady_clock::now() >= connect_deadline) {
                throw std::runtime_error("AirSim RPC connection timed out");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(
            this->get_logger(),
            "AirSim RPC connection established; version handshake skipped");

        // RGB 和深度分开指定相机名称。
        // 深度使用 float32，不使用压缩。
        requests_ = {
            ImageCapture::ImageRequest(
                left_camera,
                ImageCapture::ImageType::Scene,
                false,
                false),

            ImageCapture::ImageRequest(
                right_camera,
                ImageCapture::ImageType::Scene,
                false,
                false),

            ImageCapture::ImageRequest(
                depth_camera,
                ImageCapture::ImageType::DepthPlanar,
                true,
                false)
        };

        // 保持与原 Python 桥接相同的话题名称及可靠 QoS。
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

        left_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/left/image_raw", qos);

        right_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/right/image_raw", qos);

        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", qos);

        const auto period =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / camera_rate));

        timer_ = this->create_wall_timer(
            period,
            std::bind(&AirSimImageBridge::publish_images, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Image bridge started, target rate=%.1f Hz",
            camera_rate);
    }

private:
    template<typename Response>
    void publish_rgb(
        Response &response,
        const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub,
        const std::string &frame_id)
    {
        if (response.width == 0 || response.height == 0) {
            return;
        }

        const size_t expected =
            static_cast<size_t>(response.width) *
            static_cast<size_t>(response.height) * 3;

        if (response.image_data_uint8.size() != expected) {
            RCLCPP_WARN(
                this->get_logger(),
                "RGB size mismatch: %ux%u, bytes=%zu, expected=%zu",
                response.width,
                response.height,
                response.image_data_uint8.size(),
                expected);
            return;
        }

        auto msg = std::make_unique<sensor_msgs::msg::Image>();

        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = frame_id;
        msg->height = response.height;
        msg->width = response.width;
        msg->encoding = "bgr8";
        msg->is_bigendian = 0;
        msg->step = msg->width * 3;

        // 接管 SDK 返回的字节数组，避免再复制一次 RGB 数据。
        auto pixels = std::move(response.image_data_uint8);

        msg->data = std::move(pixels);
        pub->publish(std::move(msg));
    }

    template<typename Response>
    void publish_depth(
        const Response &response,
        const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub)
    {
        if (response.width == 0 || response.height == 0) {
            return;
        }

        const auto &depth = response.image_data_float;

        const size_t expected =
            static_cast<size_t>(response.width) *
            static_cast<size_t>(response.height);

        if (depth.size() != expected) {
            RCLCPP_WARN(
                this->get_logger(),
                "Depth size mismatch: %ux%u, floats=%zu, expected=%zu",
                response.width,
                response.height,
                depth.size(),
                expected);
            return;
        }

        auto msg = std::make_unique<sensor_msgs::msg::Image>();

        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "camera_depth";
        msg->height = response.height;
        msg->width = response.width;
        msg->encoding = "32FC1";
        msg->is_bigendian = 0;
        msg->step = msg->width * sizeof(float);

        msg->data.resize(depth.size() * sizeof(float));

        std::memcpy(
            msg->data.data(),
            depth.data(),
            msg->data.size());

        pub->publish(std::move(msg));
    }

    void publish_images()
    {
        using Clock = std::chrono::steady_clock;
        const auto start = Clock::now();

        try {
            // 与原 Python 节点一致：使用飞行器相机，而非 external camera。
            static bool first_image_request = true;

            if (first_image_request) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Calling AirSim simGetImages...");
            }

            auto responses =
                client_->simGetImages(requests_, vehicle_name_, false);

            if (first_image_request) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "simGetImages returned %zu images",
                    responses.size());
                first_image_request = false;
            }

            if (responses.size() != 3) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Expected 3 images, received %zu",
                    responses.size());
                return;
            }

            publish_rgb(
                responses[0], left_pub_, "camera_left");

            publish_rgb(
                responses[1], right_pub_, "camera_right");

            publish_depth(
                responses[2], depth_pub_);

            ++frame_count_;

            if (frame_count_ % 10 == 0) {
                const auto elapsed =
                    std::chrono::duration<double, std::milli>(
                        Clock::now() - start).count();

                RCLCPP_INFO(
                    this->get_logger(),
                    "simGetImages + publish: %.1f ms; "
                    "left=%ux%u right=%ux%u depth=%ux%u",
                    elapsed,
                    responses[0].width, responses[0].height,
                    responses[1].width, responses[1].height,
                    responses[2].width, responses[2].height);
            }
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Image bridge error: %s",
                e.what());
        }
    }

    std::string vehicle_name_;

    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> client_;

    std::vector<ImageCapture::ImageRequest> requests_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    uint64_t frame_count_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<AirSimImageBridge>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(
            rclcpp::get_logger("airsim_image_bridge"),
            "Fatal error: %s",
            e.what());

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
