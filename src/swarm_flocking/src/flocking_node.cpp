#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "boids_calculator.hpp"

#include <chrono>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <string>
#include <vector>
#include <map>

using std::placeholders::_1;

class FlockingNode : public rclcpp::Node {
    public:
        FlockingNode() : Node("flocking_node") {
            /* === Declare and read parameters === */
            this->declare_parameter<std::string>("robot_id", "robot_1");
            this->declare_parameter<std::vector<std::string>>(
                "neighbor_ids", std::vector<std::string>{"robot_2", "robot_3", "robot_4"}
            );
            this->declare_parameter<double>("radius", 2.0);
            this->declare_parameter<double>("max_speed", 1.0);
            this->declare_parameter<double>("weight_separation", 1.0);
            this->declare_parameter<double>("weight_alignment", 1.0);
            this->declare_parameter<double>("weight_cohesion", 1.0);
            this->declare_parameter<double>("weight_goal", 1.0);

            robot_id_ = this->get_parameter("robot_id").as_string();
            neighbor_ids_ = this->get_parameter("neighbor_ids").as_string_array();
            radius_ = this->get_parameter("radius").as_double();
            max_speed_ = this->get_parameter("max_speed").as_double();

            weights_.separation  = this->get_parameter("weight_separation").as_double();
            weights_.alignment   = this->get_parameter("weight_alignment").as_double();
            weights_.cohesion    = this->get_parameter("weight_cohesion").as_double();
            weights_.goal        = this->get_parameter("weight_goal").as_double();

            for (const auto& id : neighbor_ids_) {
                /* Capture 'ids' by value to avoid dangling reference in lambda */
                auto pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/" + id + "/pose", 10,
                    [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                        this->on_neighbor_pose(msg, id);
                    }
                );
                neighbor_pose_subs_.push_back(pose_sub);

                auto odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/" + id + "/odom", 10,
                    [this, id](const nav_msgs::msg::Odometry::SharedPtr msg) {
                        this->on_neighbor_odom(msg, id);
                    }
                );
                neighbor_odom_subs_.push_back(odom_sub);

                /* Initialize neighbor entry so it exists even before first message arrives */
                neighbors_[id] = NeighborState{Vec2(0.0, 0.0), Vec2(0.0, 0.0)};
            }

            /* === Subscriptions for this robot's own state === */
            my_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/" + robot_id_ + "/odom", 10,
                std::bind(&FlockingNode::on_my_odom, this, _1)
            );

            goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/" + robot_id_ + "/goal_pose", 10,
                std::bind(&FlockingNode::on_goal, this, _1)
            );

            /* === Publisher === */
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/" + robot_id_ + "/cmd_vel_flocking", 10);

            /* === Control loop timer (10Hz) === */
            control_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&FlockingNode::compute_and_publish, this)
            );

            RCLCPP_INFO(this->get_logger(), "FlockingNode started for %s", robot_id_.c_str());
        }

    private:
        /* === Parameters / state === */
        std::string robot_id_;
        std::vector<std::string> neighbor_ids_;
        double radius_;
        double max_speed_;
        FlockingWeights weights_;

        std::map<std::string, NeighborState> neighbors_;
        Vec2 my_pos_{0.0, 0.0};
        Vec2 my_vel_{0.0, 0.0};
        Vec2 goal_pos_{0.0, 0.0};

        /* === ROS2 interfaces === */
        std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> neighbor_pose_subs_;
        std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> neighbor_odom_subs_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr my_odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

        /* === Callbacks: only update raw data, no computation here === */
        void on_neighbor_pose(
            const geometry_msgs::msg::PoseStamped::SharedPtr msg, 
            const std::string& id
        ) {
            neighbors_[id].pos = Vec2(msg->pose.position.x, msg->pose.position.y);
        }

        void on_neighbor_odom(
            const nav_msgs::msg::Odometry::SharedPtr msg,
            const std::string& id
        ) {
            neighbors_[id].vel = Vec2(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        }

        void on_my_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
            my_pos_ = Vec2(msg->pose.pose.position.x, msg->pose.pose.position.y);
            my_vel_ = Vec2(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        }

        void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            goal_pos_ = Vec2(msg->pose.position.x, msg->pose.position.y);
        }

        /* === Main control loop: delegate all math to Boids calculator === */
        void compute_and_publish() {
            std::vector<NeighborState> neighbor_list;
            neighbor_list.reserve(neighbors_.size());
            for (const auto& [id, state] : neighbors_) {
                neighbor_list.push_back(state);
            }

            Vec2 v = BoidsCore::compute_velocity(
                my_pos_, my_vel_, neighbor_list, goal_pos_,
                weights_, radius_, max_speed_
            );

            geometry_msgs::msg::Twist msg;
            msg.linear.x = v.x;
            msg.linear.y = v.y;
            cmd_vel_pub_->publish(msg);
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlockingNode>());
    rclcpp::shutdown();
    return 0;
}