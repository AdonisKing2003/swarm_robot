#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "boids_calculator.hpp"

#include <chrono>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

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
            this->declare_parameter<double>("max_angular", 1.5);
            this->declare_parameter<double>("k_angular", 2.0);
            this->declare_parameter<double>("weight_separation", 1.0);
            this->declare_parameter<double>("weight_alignment", 1.0);
            this->declare_parameter<double>("weight_cohesion", 1.0);
            this->declare_parameter<double>("weight_goal", 1.0);
            this->declare_parameter<std::string>("common_frame", "world");

            robot_id_ = this->get_parameter("robot_id").as_string();
            neighbor_ids_ = this->get_parameter("neighbor_ids").as_string_array();
            radius_ = this->get_parameter("radius").as_double();
            max_speed_ = this->get_parameter("max_speed").as_double();
            max_angular_ = this->get_parameter("max_angular").as_double();
            k_angular_   = this->get_parameter("k_angular").as_double();

            weights_.separation  = this->get_parameter("weight_separation").as_double();
            weights_.alignment   = this->get_parameter("weight_alignment").as_double();
            weights_.cohesion    = this->get_parameter("weight_cohesion").as_double();
            weights_.goal        = this->get_parameter("weight_goal").as_double();

            common_frame_ = this->get_parameter("common_frame").as_string();

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            for (const auto& id : neighbor_ids_) {
                auto odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/" + id + "/odom", 10,
                    [this, id](const nav_msgs::msg::Odometry::SharedPtr msg) {
                        this->on_neighbor_odom(msg, id);
                    }
                );
                neighbor_odom_subs_.push_back(odom_sub);
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
                "/" + robot_id_ + "/cmd_vel", 10);

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
        double max_angular_;
        double k_angular_;
        FlockingWeights weights_;

        // std::map<std::string, NeighborState> neighbors_;
        std::map<std::string,double> speeds_;
        double my_speed_{0.0};
        Vec2 goal_pos_{0.0, 0.0};

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::string common_frame_;
        bool have_goal_{false};

        /* === ROS2 interfaces === */
        std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> neighbor_odom_subs_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr my_odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::TimerBase::SharedPtr control_timer_;

        /* === Callbacks: only update raw data, no computation here === */
        void on_neighbor_odom(
            const nav_msgs::msg::Odometry::SharedPtr msg,
            const std::string& id
        ) {
            speeds_[id] = msg->twist.twist.linear.x;
        }

        void on_my_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
            my_speed_ = msg->twist.twist.linear.x;
        }

        void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            goal_pos_ = Vec2(msg->pose.position.x, msg->pose.position.y);
            have_goal_ = true;
        }

        /* === Main control loop: delegate all math to Boids calculator === */
        void compute_and_publish() {
            Vec2 my_pos;
            double my_yaw;

            if( !lookup_pose(robot_id_, my_pos, my_yaw) ) {
                /* Can't act without knowing where you are */
                return;
            }

            Vec2 my_vel = body_vel_to_world(my_speed_, my_yaw);

            /* Build the neighbor list in the same frame */
            std::vector<NeighborState> neighbor_list;
            neighbor_list.reserve(neighbor_ids_.size());
            for( const auto& id : neighbor_ids_ ) {
                Vec2 n_pos;
                double n_yaw;
                if(!lookup_pose(id, n_pos, n_yaw)) {
                    continue; /* Can't locate this neighbor yet -> just skip it */
                }
                NeighborState ns;
                ns.pos = n_pos;
                ns.vel = body_vel_to_world(speeds_[id], n_yaw);
                neighbor_list.push_back(ns);
            }

            /* Boids math */
            Vec2 goal = have_goal_ ? goal_pos_ : my_pos; /* no goal yet -> zero pull */
            Vec2 v = BoidsCore::compute_velocity(my_pos, my_vel, neighbor_list, goal, weights_, radius_, max_speed_);

            /* Turn that world vector into a diff-drive command */
            geometry_msgs::msg::Twist msg;
            to_diff_drive(v, my_yaw, msg);
            cmd_vel_pub_->publish(msg);
        }

        bool lookup_pose(const std::string& id, Vec2& pos, double& yaw) {
            try {
                auto tf = tf_buffer_->lookupTransform(
                    common_frame_, id + "/base_footprint", tf2::TimePointZero);
                pos = Vec2(tf.transform.translation.x, tf.transform.translation.y);
                yaw = tf2::getYaw(tf.transform.rotation);
                return true;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF fail: %s", ex.what());
                return false; /* robot not up yet -> caller skips it*/
            }
        }

        /* A diff-drive robot can only move along its heading, so its velocity in the
            world frame is its forward speed pointed in the direction it's facing */
        static Vec2 body_vel_to_world(double speed, double yaw) {
            return Vec2(speed * std::cos(yaw), speed * std::sin(yaw));
        }

        static double normalize_angle(double a) {
            while (a > M_PI) a -= 2.0 * M_PI;
            while (a < -M_PI) a += 2.0 * M_PI;
            return a;
        }

        void to_diff_drive(const Vec2& v, double my_yaw, geometry_msgs::msg::Twist& cmd) {
            double speed = v.length();
            if( speed < 1e-3 ) {
                return; /* v ~ 0: leave cmd at its zeros -> robot stops */
            }

            /* 1. which way does Boids want me to go, and how for off is my nose? */
            double desired_yaw = std::atan2(v.y, v.x); /* direction of v in world */
            double yaw_err = normalize_angle(desired_yaw - my_yaw);

            /* 2. Steer: turn rate proportional to heading error (a P-controller),
                clamped so we never spin faster than max_angular_ */
            cmd.angular.z = std::max(-max_angular_, std::min(max_angular_, k_angular_ * yaw_err));

            /* 3. Drive forward only when roughly facing the target. cos(yaw_err) 
                smoothly scales speed: full when aligned, 0 at 90 deg, and we
                refuse to drive forward when facing away (pivot in place first) */
            if( std::fabs(yaw_err) < M_PI / 2.0 ) {
                cmd.linear.x = std::min(speed, max_speed_) * std::cos(yaw_err);
            } else {
                cmd.linear.x = 0.0;
            }
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlockingNode>());
    rclcpp::shutdown();
    return 0;
}