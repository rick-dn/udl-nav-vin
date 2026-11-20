#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <vin_fusion/sensors.hpp>


class FusionNode : public rclcpp::Node {
public:
    FusionNode() : Node("fusion_node") {

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/vio/imu", 200,
            std::bind(&FusionNode::on_imu, this, std::placeholders::_1)
        );
    };
private:
    void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {

        ImuSample s;
        s.t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        s.gyro = Eigen::Vector3d(msg->angular_velocity.x,
                                 msg->angular_velocity.y,
                                 msg->angular_velocity.z);

        s.accel = Eigen::Vector3d(msg->linear_acceleration.x,
                                  msg->linear_acceleration.y,
                                  msg->linear_acceleration.z);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "imu t=%.3f gyro=[%.3f %.3f %.3f] accel=[%.3f %.3f %.3f]",
            s.t, s.gyro.x(), s.gyro.y(), s.gyro.z(),
            s.accel.x(), s.accel.y(), s.accel.z());
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FusionNode>());
    rclcpp::shutdown();
    return 0;
}