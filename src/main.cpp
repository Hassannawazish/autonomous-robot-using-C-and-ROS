#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
  ObstacleAvoidanceNode(const std::string &node_name = "obstacle_avoidance_node")
  : Node(node_name) {
    // ---- Parameters (can be overridden via ROS params) ----
    declare_parameter("safe_distance", 0.50);        // front clear if > this
    declare_parameter("emergency_distance", 0.25);   // immediate backup if < this
    declare_parameter("clear_margin", 0.05);         // hysteresis margin
    declare_parameter("forward_speed_max", 0.22);    // ~TB3 safe speed
    declare_parameter("forward_speed_min", 0.05);
    declare_parameter("turn_speed", 0.8);            // rad/s when turning
    declare_parameter("latch_turn_sec", 0.9);        // lock-in turn decision duration
    declare_parameter("backup_sec", 0.5);            // time to move backward in emergency
    declare_parameter("control_rate_hz", 15.0);      // main loop rate
    declare_parameter("ema_alpha", 0.35);            // smoothing factor
    declare_parameter("steer_gain", 0.5);            // small bias while going forward
    declare_parameter("front_deg", 30.0);            // +/- degrees for "front" sector
    declare_parameter("side_deg_min", 40.0);         // start of left/right sectors (deg)
    declare_parameter("side_deg_max", 90.0);         // end of left/right sectors (deg)

    // QoS: TurtleBot3 /scan is typically BestEffort
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos, std::bind(&ObstacleAvoidanceNode::scanCb, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    double hz = get_parameter("control_rate_hz").as_double();
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, hz)) ),
      std::bind(&ObstacleAvoidanceNode::controlStep, this));

    RCLCPP_INFO(get_logger(), "Obstacle avoidance node ready for TurtleBot3.");
  }

private:
  // --- Types & state ---
  enum class Mode { FORWARD, TURN_LEFT, TURN_RIGHT, BACKUP };

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  bool have_scan_ = false;

  // Smoothed (EMA) sector distances
  float ema_front_ = std::numeric_limits<float>::infinity();
  float ema_left_  = std::numeric_limits<float>::infinity();
  float ema_right_ = std::numeric_limits<float>::infinity();

  Mode mode_ = Mode::FORWARD;
  rclcpp::Time mode_start_{0,0, RCL_ROS_TIME};

  // --- Helpers ---
  static int angleToIndex(const sensor_msgs::msg::LaserScan &s, double angle_rad) {
    // Clamp to scan bounds
    double a = std::min(std::max(angle_rad, static_cast<double>(s.angle_min)),
                        static_cast<double>(s.angle_max));
    int idx = static_cast<int>( std::round((a - s.angle_min) / s.angle_increment) );
    idx = std::max(0, std::min(idx, static_cast<int>(s.ranges.size()) - 1));
    return idx;
  }

  static float minInSector(const sensor_msgs::msg::LaserScan &s,
                           double deg_min, double deg_max) {
    // Convert degrees to radians relative to scan frame
    const double rmin = deg_min * M_PI / 180.0;
    const double rmax = deg_max * M_PI / 180.0;
    int i0 = angleToIndex(s, rmin);
    int i1 = angleToIndex(s, rmax);
    if (i0 > i1) std::swap(i0, i1);

    float best = std::numeric_limits<float>::infinity();
    for (int i = i0; i <= i1; ++i) {
      float r = s.ranges[i];
      if (std::isfinite(r)) best = std::min(best, r);
    }
    return best;
  }

  static float emaUpdate(float prev, float value, float alpha) {
    if (!std::isfinite(prev)) return value;  // first assignment
    return static_cast<float>(alpha * value + (1.0 - alpha) * prev);
  }

  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_ = msg;
    have_scan_ = true;
  }

  void setMode(Mode m) {
    if (m != mode_) {
      mode_ = m;
      mode_start_ = now();
      RCLCPP_INFO(get_logger(), "Mode -> %s",
                  m == Mode::FORWARD ? "FORWARD" :
                  m == Mode::TURN_LEFT ? "TURN_LEFT" :
                  m == Mode::TURN_RIGHT ? "TURN_RIGHT" : "BACKUP");
    }
  }

  bool modeElapsed(double sec) const {
    return (now() - mode_start_).seconds() >= sec;
  }

  void controlStep() {
    if (!have_scan_) return;
    const auto &s = *last_scan_;

    // --- Read params (allows runtime tuning with ros2 param set) ---
    const float safe_d      = get_parameter("safe_distance").as_double();
    const float emerg_d     = get_parameter("emergency_distance").as_double();
    const float margin      = get_parameter("clear_margin").as_double();
    const float v_max       = get_parameter("forward_speed_max").as_double();
    const float v_min       = get_parameter("forward_speed_min").as_double();
    const float w_turn      = get_parameter("turn_speed").as_double();
    const float latch_turn  = get_parameter("latch_turn_sec").as_double();
    const float backup_sec  = get_parameter("backup_sec").as_double();
    const float alpha       = get_parameter("ema_alpha").as_double();
    const float steer_gain  = get_parameter("steer_gain").as_double();
    const float front_deg   = get_parameter("front_deg").as_double();
    const float side_min    = get_parameter("side_deg_min").as_double();
    const float side_max    = get_parameter("side_deg_max").as_double();

    // --- Sector mins (in degrees relative to forward) ---
    // Front = [-front_deg, +front_deg], Left = [side_min, side_max], Right = [-side_max, -side_min]
    const float min_front_raw = minInSector(s, -front_deg, +front_deg);
    const float min_left_raw  = minInSector(s, +side_min, +side_max);
    const float min_right_raw = minInSector(s, -side_max, -side_min);

    // EMA smoothing
    ema_front_ = emaUpdate(ema_front_, min_front_raw, alpha);
    ema_left_  = emaUpdate(ema_left_ , min_left_raw , alpha);
    ema_right_ = emaUpdate(ema_right_, min_right_raw, alpha);

    RCLCPP_DEBUG(get_logger(), "Front(raw/ema)=%.2f/%.2f Left=%.2f/%.2f Right=%.2f/%.2f",
                 min_front_raw, ema_front_, min_left_raw, ema_left_, min_right_raw, ema_right_);

    geometry_msgs::msg::Twist cmd;

    // --- Emergency: too close anywhere ahead -> backup then turn ---
    if (ema_front_ < emerg_d && mode_ != Mode::BACKUP) {
      setMode(Mode::BACKUP);
    }

    // --- State machine ---
    switch (mode_) {
      case Mode::BACKUP: {
        cmd.linear.x = -v_min;       // gentle backward
        cmd.angular.z = 0.0;
        if (modeElapsed(backup_sec)) {
          // After backing up, choose a turn away from the closer side
          if (ema_left_ > ema_right_ + margin) setMode(Mode::TURN_LEFT);
          else setMode(Mode::TURN_RIGHT);
        }
        break;
      }

      case Mode::TURN_LEFT: {
        cmd.angular.z = +w_turn;
        // keep turning for latching time before reevaluation
        if (modeElapsed(latch_turn)) {
          // If front is now safely clear (with margin), go forward
          if (ema_front_ > safe_d + margin) setMode(Mode::FORWARD);
          else if (ema_right_ > ema_left_ + margin) setMode(Mode::TURN_RIGHT); // flip if clearly better
          else mode_start_ = now(); // continue turning (re-latch) if still blocked
        }
        break;
      }

      case Mode::TURN_RIGHT: {
        cmd.angular.z = -w_turn;
        if (modeElapsed(latch_turn)) {
          if (ema_front_ > safe_d + margin) setMode(Mode::FORWARD);
          else if (ema_left_ > ema_right_ + margin) setMode(Mode::TURN_LEFT);
          else mode_start_ = now();
        }
        break;
      }

      case Mode::FORWARD:
      default: {
        // If blocked in front -> pick a turn and latch
        if (ema_front_ < safe_d) {
          if (ema_left_ > ema_right_ + margin) setMode(Mode::TURN_LEFT);
          else setMode(Mode::TURN_RIGHT);
          // Don’t publish forward cmd this tick; next loop will apply turn
          break;
        }

        // Forward speed scales with front clearance
        const float span = std::max(0.001f, safe_d);  // avoid divide-by-zero
        float k = std::min(1.0f, (ema_front_ - safe_d) / span); // 0..1
        float v = v_min + k * (v_max - v_min);
        v = std::clamp(v, v_min, v_max);
        cmd.linear.x = v;

        // Small steering bias away from the closer side
        float side_diff = (ema_left_ - ema_right_); // if right closer -> negative -> steer left
        cmd.angular.z = std::clamp(steer_gain * (-side_diff), -0.6f, 0.6f);
        break;
      }
    }

    cmd_pub_->publish(cmd);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}
