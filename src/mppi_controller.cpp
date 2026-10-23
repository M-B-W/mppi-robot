#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/time.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>
class MPPIController : public rclcpp::Node
{
public:
    MPPIController()
        : Node("mppi_controller"),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_),
          rng_(std::random_device{}())
    {
        auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .transient_local();
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/path", path_qos,
        [this](nav_msgs::msg::Path::SharedPtr msg) {
            path_ = *msg;
            have_path_ = !path_.poses.empty();
            RCLCPP_INFO(
                get_logger(),
                "Received path with %zu poses",
                path_.poses.size());
        });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                x_ = msg->pose.pose.position.x;
                y_ = msg->pose.pose.position.y;
                const auto &q = msg->pose.pose.orientation;
                yaw_ = std::atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                have_odom_ = true;
            });
        scan_sub_ =
            create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    scan_ = *msg;
                    have_scan_ = true;
                });
        cmd_pub_ =
            create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10);
        declare_parameter("samples", 300);
        declare_parameter("horizon_steps", 20);
        declare_parameter("dt", 0.1);
        declare_parameter("max_linear", 0.18);
        declare_parameter("max_angular", 1.0);
        declare_parameter("obstacle_margin", 0.45);
        declare_parameter("lookahead_distance", 0.45);
        declare_parameter("goal_tolerance", 0.20);
        declare_parameter("mppi_lambda", 10.0);
        declare_parameter("noise_linear_std", 0.06);
        declare_parameter("noise_angular_std", 0.35);
        samples_ = static_cast<int>(
            std::max<int64_t>(
                1, get_parameter("samples").as_int()));
        horizon_ = static_cast<int>(
            std::max<int64_t>(
                1, get_parameter("horizon_steps").as_int()));
        dt_ = get_parameter("dt").as_double();
        max_v_ = get_parameter("max_linear").as_double();
        max_w_ = get_parameter("max_angular").as_double();
        obstacle_margin_ =
            get_parameter("obstacle_margin").as_double();
        lookahead_distance_ =
            get_parameter("lookahead_distance").as_double();
        goal_tolerance_ =
            get_parameter("goal_tolerance").as_double();
        mppi_lambda_ = get_parameter("mppi_lambda").as_double();
        noise_linear_std_ = get_parameter("noise_linear_std").as_double();
        noise_angular_std_ = get_parameter("noise_angular_std").as_double();
        nominal_sequence_.assign(static_cast<std::size_t>(horizon_), Control{0.06, 0.0});
        if (dt_ <= 0.0 || max_v_ <= 0.0 || max_w_ <= 0.0 ||
            mppi_lambda_ <= 0.0 || noise_linear_std_ <= 0.0 ||
            noise_angular_std_ <= 0.0) {
            throw std::invalid_argument(
                "dt, max_linear and max_angular must be positive");
        }
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MPPIController::control, this));
        RCLCPP_INFO(
            get_logger(),
            "Cost-weighted MPPI controller started: samples=%d, horizon=%d, lambda=%.2f",
            samples_, horizon_, mppi_lambda_);
    }
private:
    struct Control
    {
        double v;
        double w;
    };
    struct Point
    {
        double x;
        double y;
    };
    static constexpr double PI = 3.14159265358979323846;
    double normalize_angle(double angle) const
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }
    double distance(double x1, double y1,
                    double x2, double y2) const
    {
        return std::hypot(x1 - x2, y1 - y2);
    }
    std::size_t nearest_path_index()
    {
        if (path_.poses.empty()) {
            last_nearest_index_ = 0;
            return 0;
        }
        const std::size_t last_index = path_.poses.size() - 1;
        const std::size_t start =
            (last_nearest_index_ > 3)
                ? last_nearest_index_ - 3
                : 0;
        const std::size_t end =
            std::min(last_nearest_index_ + 15, last_index);
        std::size_t best_index = start;
        double best_distance = std::numeric_limits<double>::max();
        for (std::size_t i = start; i <= end; ++i) {
            const auto &p = path_.poses[i].pose.position;
            const double d = distance(x_, y_, p.x, p.y);
            if (d < best_distance) {
                best_distance = d;
                best_index = i;
            }
        }
        last_nearest_index_ = best_index;
        return best_index;
    }
    Point get_target(std::size_t nearest) const
    {
        const auto &start =
            path_.poses[nearest].pose.position;
        for (std::size_t i = nearest;
             i < path_.poses.size(); ++i) {
            const auto &p = path_.poses[i].pose.position;
            if (distance(start.x, start.y, p.x, p.y)
                >= lookahead_distance_) {
                return {p.x, p.y};
            }
        }
        const auto &last = path_.poses.back().pose.position;
        return {last.x, last.y};
    }
    std::vector<Point> obstacle_points()
    {
        std::vector<Point> points;
        obstacle_transform_valid_ = false;
        if (!have_scan_ || scan_.header.frame_id.empty()) {
            return points;
        }
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(
                "odom",
                scan_.header.frame_id,
                tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "LiDAR transform unavailable: %s", ex.what());
            return points;
        }
        obstacle_transform_valid_ = true;
        const auto &t = transform.transform.translation;
        const auto &q = transform.transform.rotation;
        const double sensor_yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        points.reserve(scan_.ranges.size());
        for (std::size_t i = 0; i < scan_.ranges.size(); ++i) {
            const double r = scan_.ranges[i];
            if (!std::isfinite(r) ||
                r < scan_.range_min ||
                r > scan_.range_max) {
                continue;
            }
            const double angle =
                sensor_yaw + scan_.angle_min +
                static_cast<double>(i) * scan_.angle_increment;
            points.push_back({
                t.x + r * std::cos(angle),
                t.y + r * std::sin(angle)
            });
        }
        return points;
    }
    double obstacle_cost(
        double px, double py,
        const std::vector<Point> &obstacles) const
    {
        double cost = 0.0;
        for (const auto &obstacle : obstacles) {
            const double d =
                distance(px, py, obstacle.x, obstacle.y);
            if (d < obstacle_margin_) {
                const double penetration = obstacle_margin_ - d;
                cost += 200.0 + 400.0 * penetration * penetration;
            } else if (d < obstacle_margin_ + 0.30) {
                const double gap = d - obstacle_margin_;
                cost += 2.0 / std::max(0.05, gap);
            }
        }
        return cost;
    }
    double evaluate(
        const std::vector<Control> &sequence,
        const Point &target,
        const std::vector<Point> &obstacles) const
    {
        double px = x_;
        double py = y_;
        double theta = yaw_;
        double cost = 0.0;
        double previous_w = 0.0;
        for (const auto &u : sequence) {
            px += u.v * std::cos(theta) * dt_;
            py += u.v * std::sin(theta) * dt_;
            theta = normalize_angle(theta + u.w * dt_);
            const double target_distance =
                distance(px, py, target.x, target.y);
            const double target_heading =
                std::atan2(target.y - py, target.x - px);
            const double heading_error =
                normalize_angle(target_heading - theta);
            cost += 5.0 * target_distance * target_distance;
            cost += 0.8 * heading_error * heading_error;
            const double obs_cost = obstacle_cost(px, py, obstacles);
            if (obs_cost >= 200.0) {
                return std::numeric_limits<double>::infinity();
            }
            cost += obs_cost;
            cost += 0.12 * u.w * u.w;
            cost += 0.05 * u.v * u.v;
            const double delta_w = u.w - previous_w;
            cost += 0.8 * delta_w * delta_w;
            previous_w = u.w;
        }
        return cost;
    }
    bool path_goal_reached() const
    {
        if (path_.poses.empty()) {
            return false;
        }
        const auto &goal = path_.poses.back().pose.position;
        return distance(x_, y_, goal.x, goal.y) <= goal_tolerance_;
    }
    void publish_stop()
    {
        cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }
    void control()
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Inputs: path=%d, odom=%d, scan=%d",
            have_path_, have_odom_, have_scan_);
        if (!have_path_ || !have_odom_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for path or odometry");
            publish_stop();
            return;
        }
        if (path_goal_reached()) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 3000,
                "Final path goal reached");
            publish_stop();
            return;
        }
        const std::size_t nearest = nearest_path_index();
        const Point target = get_target(nearest);
        const auto obstacles = obstacle_points();
        if (!obstacle_transform_valid_) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Stopping: LiDAR transform is unavailable");
            publish_stop();
            return;
        }
        std::vector<Control> best_sequence;
        double best_cost = std::numeric_limits<double>::max();
        std::normal_distribution<double> linear_noise(0.0, noise_linear_std_);
        std::normal_distribution<double> angular_noise(0.0, noise_angular_std_);
        std::vector<std::vector<Control>> sampled_sequences;
        std::vector<std::vector<Control>> sampled_noise;
        std::vector<double> costs;
        sampled_sequences.reserve(static_cast<std::size_t>(samples_));
        sampled_noise.reserve(static_cast<std::size_t>(samples_));
        costs.reserve(static_cast<std::size_t>(samples_));
        for (int sample = 0; sample < samples_; ++sample) {
            std::vector<Control> sequence;
            std::vector<Control> noise_sequence;
            sequence.reserve(static_cast<std::size_t>(horizon_));
            noise_sequence.reserve(static_cast<std::size_t>(horizon_));
            for (int step = 0; step < horizon_; ++step) {
                const auto &nominal = nominal_sequence_[static_cast<std::size_t>(step)];
                const double noise_v = linear_noise(rng_);
                const double noise_w = angular_noise(rng_);
                const double sampled_v =
                    std::clamp(nominal.v + noise_v, 0.0, max_v_);
                const double sampled_w =
                    std::clamp(nominal.w + noise_w, -max_w_, max_w_);
                sequence.push_back({sampled_v, sampled_w});
                noise_sequence.push_back({
                    sampled_v - nominal.v,
                    sampled_w - nominal.w
                });
            }
            const double cost = evaluate(sequence, target, obstacles);
            sampled_sequences.push_back(std::move(sequence));
            sampled_noise.push_back(std::move(noise_sequence));
            costs.push_back(cost);
            if (std::isfinite(cost) && cost < best_cost) {
                best_cost = cost;
                best_sequence = sampled_sequences.back();
            }
        }
        std::vector<double> weights(costs.size(), 0.0);
        double weight_sum = 0.0;
        if (std::isfinite(best_cost)) {
            for (std::size_t sample = 0; sample < costs.size(); ++sample) {
                if (!std::isfinite(costs[sample])) {
                    continue;
                }
                const double exponent =
                    -(costs[sample] - best_cost) / mppi_lambda_;
                const double weight = std::exp(std::max(-700.0, exponent));
                weights[sample] = weight;
                weight_sum += weight;
            }
        }
        std::vector<Control> updated_sequence = nominal_sequence_;
        if (weight_sum > 0.0) {
            for (int step = 0; step < horizon_; ++step) {
                double weighted_delta_v = 0.0;
                double weighted_delta_w = 0.0;
                for (std::size_t sample = 0; sample < weights.size(); ++sample) {
                    if (weights[sample] <= 0.0) {
                        continue;
                    }
                    const double normalized_weight = weights[sample] / weight_sum;
                    const auto &noise =
                        sampled_noise[sample][static_cast<std::size_t>(step)];
                    weighted_delta_v += normalized_weight * noise.v;
                    weighted_delta_w += normalized_weight * noise.w;
                }
                auto &u = updated_sequence[static_cast<std::size_t>(step)];
                u.v = std::clamp(u.v + weighted_delta_v, 0.0, max_v_);
                u.w = std::clamp(u.w + weighted_delta_w, -max_w_, max_w_);
            }
            const double updated_cost =
                evaluate(updated_sequence, target, obstacles);
            if (std::isfinite(updated_cost)) {
                best_sequence = updated_sequence;
                best_cost = updated_cost;
                nominal_sequence_ = updated_sequence;
            } else if (!best_sequence.empty()) {
                nominal_sequence_ = best_sequence;
            }
        }
        auto nearest_obstacle_distance =
            [](double px, double py,
               const std::vector<Point> &points) {
                double nearest = std::numeric_limits<double>::infinity();
                for (const auto &p : points) {
                    nearest = std::min(
                        nearest, std::hypot(px - p.x, py - p.y));
                }
                return nearest;
            };
        bool recovery_mode = false;
        if (best_sequence.empty()) {
            recovery_mode = true;
            double recovery_best_cost =
                std::numeric_limits<double>::max();
            std::vector<Control> recovery_sequence;
            const double initial_clearance =
                nearest_obstacle_distance(x_, y_, obstacles);
            const std::vector<double> speeds = {
                0.0, 0.025, 0.05, -0.025, -0.05
            };
            const std::vector<double> turns = {
                -max_w_, -0.7 * max_w_, -0.4 * max_w_,
                -0.2 * max_w_, 0.2 * max_w_, 0.4 * max_w_,
                0.7 * max_w_, max_w_
            };
            for (double v : speeds) {
                for (double w : turns) {
                    std::vector<Control> candidate;
                    candidate.reserve(static_cast<std::size_t>(horizon_));
                    double px = x_;
                    double py = y_;
                    double theta = yaw_;
                    double cost = 0.0;
                    double minimum_clearance =
                        std::numeric_limits<double>::infinity();
                    bool safe = true;
                    for (int step = 0; step < horizon_; ++step) {
                        const double phase =
                            static_cast<double>(step + 1) /
                            static_cast<double>(horizon_);
                        const double shaped_w = w * phase;
                        px += v * std::cos(theta) * dt_;
                        py += v * std::sin(theta) * dt_;
                        theta = normalize_angle(theta + shaped_w * dt_);
                        const double clearance =
                            nearest_obstacle_distance(px, py, obstacles);
                        minimum_clearance =
                            std::min(minimum_clearance, clearance);
                        if (initial_clearance >= obstacle_margin_) {
                            if (clearance < obstacle_margin_) {
                                safe = false;
                                break;
                            }
                        } else {
                            if (clearance < initial_clearance - 0.03) {
                                safe = false;
                                break;
                            }
                        }
                        const double target_distance =
                            distance(px, py, target.x, target.y);
                        const double target_heading =
                            std::atan2(target.y - py, target.x - px);
                        const double heading_error =
                            normalize_angle(target_heading - theta);
                        cost += 5.0 * target_distance * target_distance;
                        cost += 1.2 * heading_error * heading_error;
                        cost += 0.12 * shaped_w * shaped_w;
                        cost += 0.08 * v * v;
                        if (clearance < obstacle_margin_ + 0.35) {
                            const double deficit =
                                obstacle_margin_ + 0.35 - clearance;
                            cost += 8.0 * deficit * deficit;
                        }
                        candidate.push_back({v, shaped_w});
                    }
                    if (!safe || candidate.empty()) {
                        continue;
                    }
                    const double initial_target_distance =
                        distance(x_, y_, target.x, target.y);
                    const double final_target_distance =
                        distance(px, py, target.x, target.y);
                    cost += 12.0 * std::max(
                        0.0, final_target_distance - initial_target_distance);
                    const double final_clearance =
                        nearest_obstacle_distance(px, py, obstacles);
                    if (initial_clearance < obstacle_margin_ &&
                        final_clearance < initial_clearance - 0.03) {
                        continue;
                    }
                    if (cost < recovery_best_cost) {
                        recovery_best_cost = cost;
                        recovery_sequence = std::move(candidate);
                    }
                }
            }
            if (recovery_sequence.empty()) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "No safe recovery candidate found; stopping");
                publish_stop();
                return;
            }
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1500,
                "Recovery selected: v=%.3f, w=%.3f, cost=%.3f",
                recovery_sequence.front().v,
                recovery_sequence.front().w,
                recovery_best_cost);
            best_cost = recovery_best_cost;
            best_sequence = std::move(recovery_sequence);
            nominal_sequence_ = best_sequence;
        }
                const Control first = best_sequence.front();
        const double next_x =
            x_ + first.v * std::cos(yaw_) * dt_;
        const double next_y =
            y_ + first.v * std::sin(yaw_) * dt_;
        const double next_obstacle_cost =
            obstacle_cost(next_x, next_y, obstacles);
        const double current_clearance =
            nearest_obstacle_distance(x_, y_, obstacles);
        const double next_clearance =
            nearest_obstacle_distance(next_x, next_y, obstacles);
        if (!recovery_mode && next_obstacle_cost >= 200.0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Safety stop: first predicted pose enters obstacle margin "
                "(current clearance=%.3f, next=%.3f)",
                current_clearance, next_clearance);
            publish_stop();
            return;
        }
        geometry_msgs::msg::Twist command;
        command.linear.x = first.v;
        command.angular.z = first.w;
        cmd_pub_->publish(command);
        if (!nominal_sequence_.empty()) {
            for (std::size_t i = 0; i + 1 < nominal_sequence_.size(); ++i) {
                nominal_sequence_[i] = nominal_sequence_[i + 1];
            }
            nominal_sequence_.back().v = std::clamp(nominal_sequence_.back().v, 0.0, max_v_);
            nominal_sequence_.back().w = std::clamp(nominal_sequence_.back().w, -max_w_, max_w_);
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Target=(%.2f, %.2f), pose=(%.2f, %.2f), "
            "velocity=(%.3f, %.3f), cost=%.3f, obstacles=%zu",
            target.x, target.y, x_, y_,
            first.v, first.w, best_cost, obstacles.size());
    }
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    nav_msgs::msg::Path path_;
    sensor_msgs::msg::LaserScan scan_;
    bool have_path_{false};
    bool have_odom_{false};
    bool have_scan_{false};
    bool obstacle_transform_valid_{false};
    double x_{0.0};
    double y_{0.0};
    double yaw_{0.0};
    int samples_{300};
    int horizon_{20};
    double dt_{0.1};
    double max_v_{0.18};
    double max_w_{1.0};
    double obstacle_margin_{0.45};
    double lookahead_distance_{0.45};
    double goal_tolerance_{0.20};
    double mppi_lambda_{10.0};
    double noise_linear_std_{0.06};
    double noise_angular_std_{0.35};
    std::vector<Control> nominal_sequence_;
    std::size_t last_nearest_index_{0};
    std::mt19937 rng_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPPIController>());
    rclcpp::shutdown();
    return 0;
}