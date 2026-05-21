#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstdint>
#include <vector>
#include <string>
#include <mutex>

using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;

class MergeCustomMsgNode : public rclcpp::Node {
public:
  MergeCustomMsgNode() : Node("merge_point_node"),
                         tf_buffer_(this->get_clock()),
                         tf_listener_(tf_buffer_) {
    // QoS 配置
    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    // 独立 subscriber + cache
    sub1_ = this->create_subscription<CustomMsg>(
        "/livox/lidar_192_168_1_187", qos,
        [this](CustomMsg::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lk(cache_mutex_);
            cache1_ = msg;
            try_merge();
        });

    sub2_ = this->create_subscription<CustomMsg>(
        "/livox/lidar_192_168_1_198", qos,
        [this](CustomMsg::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lk(cache_mutex_);
            cache2_ = msg;
            try_merge();
        });

    // publisher 设置
    pub_merged_ = this->create_publisher<CustomMsg>("/merged_cloud", 20); // 配合 fast_lio 的 qos 设置
    
    // 【新增】可视化 Marker 发布器
    pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/filter_box_marker", 10);

    // 过滤参数声明
    this->declare_parameter<bool>("enable_box_filter", true);
    this->declare_parameter<std::string>("filter_target_frame", "base_link");
    this->declare_parameter<double>("filter_box_min_x", -1.0);
    this->declare_parameter<double>("filter_box_max_x",  2.0);
    this->declare_parameter<double>("filter_box_min_y", -1.0);
    this->declare_parameter<double>("filter_box_max_y",  1.0);
    this->declare_parameter<double>("filter_box_min_z", -0.5);
    this->declare_parameter<double>("filter_box_max_z",  1.5);
    
    // 盲区过滤 parameter (m)
    this->declare_parameter<double>("lidar_blind_zone", 0.05);

    // 初始化本地存储的参数
    update_internal_parameters();

    // 注册参数更新回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MergeCustomMsgNode::on_set_parameters, this, std::placeholders::_1));

    // 定时发布 Marker (1Hz)
    marker_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        if (params_.enable_box_filter) {
          publishFilterBoxMarker(params_.filter_target_frame, 
                                 params_.filter_box_min_x, params_.filter_box_max_x, 
                                 params_.filter_box_min_y, params_.filter_box_max_y, 
                                 params_.filter_box_min_z, params_.filter_box_max_z);
        }
      });
  }

private:
  // 参数更新回调
  rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters) {
      if (param.get_name() == "enable_box_filter") {
        params_.enable_box_filter = param.as_bool();
      } else if (param.get_name() == "filter_target_frame") {
        params_.filter_target_frame = param.as_string();
      } else if (param.get_name() == "filter_box_min_x") {
        params_.filter_box_min_x = param.as_double();
      } else if (param.get_name() == "filter_box_max_x") {
        params_.filter_box_max_x = param.as_double();
      } else if (param.get_name() == "filter_box_min_y") {
        params_.filter_box_min_y = param.as_double();
      } else if (param.get_name() == "filter_box_max_y") {
        params_.filter_box_max_y = param.as_double();
      } else if (param.get_name() == "filter_box_min_z") {
        params_.filter_box_min_z = param.as_double();
      } else if (param.get_name() == "filter_box_max_z") {
        params_.filter_box_max_z = param.as_double();
      } else if (param.get_name() == "lidar_blind_zone") {
        params_.lidar_blind_zone = param.as_double();
      }
    }
    return result;
  }

  void update_internal_parameters() {
    params_.enable_box_filter = this->get_parameter("enable_box_filter").as_bool();
    params_.filter_target_frame = this->get_parameter("filter_target_frame").as_string();
    params_.filter_box_min_x = this->get_parameter("filter_box_min_x").as_double();
    params_.filter_box_max_x = this->get_parameter("filter_box_max_x").as_double();
    params_.filter_box_min_y = this->get_parameter("filter_box_min_y").as_double();
    params_.filter_box_max_y = this->get_parameter("filter_box_max_y").as_double();
    params_.filter_box_min_z = this->get_parameter("filter_box_min_z").as_double();
    params_.filter_box_max_z = this->get_parameter("filter_box_max_z").as_double();
    params_.lidar_blind_zone = this->get_parameter("lidar_blind_zone").as_double();
  }

  void try_merge() {
    // 需在 lock_guard 范围内调用或确保线程安全
    if (!cache1_ || !cache2_) return;

    double dt = std::abs(
        rclcpp::Time(cache1_->header.stamp).seconds() -
        rclcpp::Time(cache2_->header.stamp).seconds());
    
    // PTP 对齐后 stamp 差应 <5ms，超出则认为不是同一帧
    if (dt > 0.005) {
      // 如果时间差太大，丢弃旧的一帧（简单策略）
      if (rclcpp::Time(cache1_->header.stamp).seconds() < rclcpp::Time(cache2_->header.stamp).seconds()) {
        cache1_ = nullptr;
      } else {
        cache2_ = nullptr;
      }
      return;
    }

    auto msg1 = cache1_;
    auto msg2 = cache2_;
    cache1_ = nullptr;
    cache2_ = nullptr;

    do_merge(msg1, msg2);
  }

  void do_merge(const CustomMsg::ConstSharedPtr& msg1, const CustomMsg::ConstSharedPtr& msg2) {
    if (!msg1 || !msg2 || msg1->points.empty() || msg2->points.empty()) return;

    auto start_time = this->now(); // 用于测速

    // 1. 获取所有需要的 TF (设置超时为0，查不到直接用上一帧或丢弃，绝不能阻塞)
    geometry_msgs::msg::TransformStamped tf_lidar2_to_1_msg;
    geometry_msgs::msg::TransformStamped tf_lidar1_to_target_msg;
    geometry_msgs::msg::TransformStamped tf_lidar2_to_target_msg;
    bool tf_ok = true;
    
    try {
        // 只查询最新可用的TF，不要等待
        tf_lidar2_to_1_msg = tf_buffer_.lookupTransform(
            "livox_frame_192_168_1_187", "livox_frame_192_168_1_198", tf2::TimePointZero);
        tf_lidar1_to_target_msg = tf_buffer_.lookupTransform(
            params_.filter_target_frame, "livox_frame_192_168_1_187", tf2::TimePointZero);
        tf_lidar2_to_target_msg = tf_buffer_.lookupTransform(
            params_.filter_target_frame, "livox_frame_192_168_1_198", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Sync Error: %s", ex.what());
        tf_ok = false;
    }

    if (!tf_ok) return; // 如果没有TF，直接丢弃该帧，防止错位累积

    // 将 Msg 转换为 tf2 格式
    tf2::Transform tf2_lidar2_to_1;
    tf2::Transform tf2_lidar1_to_target;
    tf2::Transform tf2_lidar2_to_target;
    tf2::fromMsg(tf_lidar2_to_1_msg.transform, tf2_lidar2_to_1);
    tf2::fromMsg(tf_lidar1_to_target_msg.transform, tf2_lidar1_to_target);
    tf2::fromMsg(tf_lidar2_to_target_msg.transform, tf2_lidar2_to_target);

    // 计算时间差
    uint64_t dt_ns = 0;
    uint8_t earlier_lidar_id = (msg2->timebase > msg1->timebase) ? 1 : 2;
    dt_ns = (earlier_lidar_id == 1) ? (msg2->timebase - msg1->timebase) : (msg1->timebase - msg2->timebase);

    CustomMsg merged;
    merged.lidar_id = 0;
    // 预分配内存，极大提升 push_back 效率 (非常关键!)
    merged.points.reserve(msg1->points.size() + msg2->points.size()); 

    double blind_sq = params_.lidar_blind_zone * params_.lidar_blind_zone;
    bool check_blind = (params_.lidar_blind_zone > 1e-4);
    bool enable_box = params_.enable_box_filter;

    // 辅助 Lambda 表达式：用于判断是否在包围盒内
    auto is_in_box = [&](const tf2::Vector3& pt_target) {
        return (pt_target.x() >= params_.filter_box_min_x && pt_target.x() <= params_.filter_box_max_x &&
                pt_target.y() >= params_.filter_box_min_y && pt_target.y() <= params_.filter_box_max_y &&
                pt_target.z() >= params_.filter_box_min_z && pt_target.z() <= params_.filter_box_max_z);
    };

    // ========== 一次性处理 msg1 ==========
    for (const auto& pt : msg1->points) {
        if (check_blind && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) < blind_sq) continue;
        
        if (enable_box) {
            tf2::Vector3 pt_target = tf2_lidar1_to_target * tf2::Vector3(pt.x, pt.y, pt.z);
            if (is_in_box(pt_target)) continue; // 命中车身，丢弃
        }
        
        CustomPoint new_pt = pt;
        new_pt.offset_time = pt.offset_time + (earlier_lidar_id == 1 ? 0 : dt_ns);
        merged.points.push_back(new_pt);
    }

    // ========== 一次性处理 msg2 ==========
    for (const auto& pt : msg2->points) {
        if (check_blind && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) < blind_sq) continue;

        if (enable_box) {
            // 直接用 lidar2 -> target 的矩阵判断，省去中间转到 lidar1 的步骤
            tf2::Vector3 pt_target = tf2_lidar2_to_target * tf2::Vector3(pt.x, pt.y, pt.z);
            if (is_in_box(pt_target)) continue; // 命中车身，丢弃
        }

        // 转到 lidar1 坐标系 (因为 merged 以 lidar1 为主)
        tf2::Vector3 pt_in_lidar1 = tf2_lidar2_to_1 * tf2::Vector3(pt.x, pt.y, pt.z);
        
        CustomPoint new_pt = pt;
        new_pt.x = pt_in_lidar1.x();
        new_pt.y = pt_in_lidar1.y();
        new_pt.z = pt_in_lidar1.z();
        new_pt.offset_time = pt.offset_time + (earlier_lidar_id == 2 ? 0 : dt_ns);
        merged.points.push_back(new_pt);
    }

    merged.point_num = merged.points.size();
    merged.header.stamp = (earlier_lidar_id == 1) ? msg1->header.stamp : msg2->header.stamp;
    merged.header.frame_id = "livox_frame_192_168_1_187";
    merged.timebase = (earlier_lidar_id == 1) ? msg1->timebase : msg2->timebase;

    pub_merged_->publish(merged);

    // 计算并打印耗时，确保它在 10ms 以内
    //double cost_ms = (this->now() - start_time).seconds() * 1000.0;
    //RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Merge callback cost: %.2f ms", cost_ms);
}

  // 【新增函数】发布可视化 Marker
  void publishFilterBoxMarker(const std::string& frame_id, double min_x, double max_x, 
                              double min_y, double max_y, double min_z, double max_z) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    
    marker.header.frame_id = frame_id; // 以 base_link 或设置的 target_frame 为基准
    marker.header.stamp = this->now();
    marker.ns = "filter_box";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 计算中心点
    marker.pose.position.x = (min_x + max_x) / 2.0;
    marker.pose.position.y = (min_y + max_y) / 2.0;
    marker.pose.position.z = (min_z + max_z) / 2.0;
    marker.pose.orientation.w = 1.0;

    // 计算长宽高
    marker.scale.x = std::abs(max_x - min_x);
    marker.scale.y = std::abs(max_y - min_y);
    marker.scale.z = std::abs(max_z - min_z);

    // 设置颜色：半透明红色
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.4; // 0.4 透明度

    marker_array.markers.push_back(marker);
    pub_marker_->publish(marker_array);
  }

  rclcpp::Subscription<CustomMsg>::SharedPtr sub1_, sub2_;
  CustomMsg::ConstSharedPtr cache1_, cache2_;
  std::mutex cache_mutex_;
  
  rclcpp::Publisher<CustomMsg>::SharedPtr pub_merged_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 参数相关的结构体
  struct Parameters {
    bool enable_box_filter;
    std::string filter_target_frame;
    double filter_box_min_x;
    double filter_box_max_x;
    double filter_box_min_y;
    double filter_box_max_y;
    double filter_box_min_z;
    double filter_box_max_z;
    double lidar_blind_zone;
  } params_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MergeCustomMsgNode>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}