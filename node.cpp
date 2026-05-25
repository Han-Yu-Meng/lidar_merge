/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include <fins/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
  #include <tf2_eigen/tf2_eigen.hpp>
#else
  #include <tf2_eigen/tf2_eigen.h>
#endif

#include "livox_driver2/msg/custom_msg.hpp"
#include <Eigen/Dense>
#include <mutex>

using CustomMsg = livox_driver2::msg::CustomMsg;
using CustomPoint = livox_driver2::msg::CustomPoint;

class MergeCustomMsgNode : public fins::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Config {
        bool enable_box_filter = true;
        std::string filter_target_frame = "base_lidar";
        double filter_box_min_x = -1.0;
        double filter_box_max_x = 2.0;
        double filter_box_min_y = -1.0;
        double filter_box_max_y = 1.0;
        double filter_box_min_z = -0.5;
        double filter_box_max_z = 1.5;
        double lidar_blind_zone = 0.05;
    };

    void define() override {
        set_name("LidarMerge");
        set_category("Driver");

        register_input<CustomMsg>("lidar1", &MergeCustomMsgNode::on_lidar1_callback);
        register_input<CustomMsg>("lidar2", &MergeCustomMsgNode::on_lidar2_callback);
        register_input<geometry_msgs::msg::TransformStamped>("$T_{target}^{lidar1}$", &MergeCustomMsgNode::on_tf_lidar1_to_target_callback);
        register_input<geometry_msgs::msg::TransformStamped>("$T_{target}^{lidar2}$", &MergeCustomMsgNode::on_tf_lidar2_to_target_callback);

        register_output<CustomMsg>("merged_cloud");
        register_output<visualization_msgs::msg::MarkerArray>("filter_box_marker");
    }

    void initialize() override {
        fins::ParamLoader config("LidarMerge");
        
        config_.enable_box_filter = config.get("enable_box_filter", true);
        config_.filter_target_frame = config.get("filter_target_frame", std::string("base_lidar"));
        config_.filter_box_min_x = config.get("filter_box_min_x", -1.0);
        config_.filter_box_max_x = config.get("filter_box_max_x", 2.0);
        config_.filter_box_min_y = config.get("filter_box_min_y", -1.0);
        config_.filter_box_max_y = config.get("filter_box_max_y", 1.0);
        config_.filter_box_min_z = config.get("filter_box_min_z", -0.5);
        config_.filter_box_max_z = config.get("filter_box_max_z", 1.5);
        config_.lidar_blind_zone = config.get("lidar_blind_zone", 0.05);

        target_frame_id_ = config_.filter_target_frame;
    }

private:
    void on_lidar1_callback(const fins::Msg<CustomMsg>& msg) {
        std::lock_guard<std::mutex> lock(data_mtx_);
        cache1_ = msg.ptr();
        cache1_ts_ = msg.acq_time;
        try_merge();
    }

    void on_lidar2_callback(const fins::Msg<CustomMsg>& msg) {
        std::lock_guard<std::mutex> lock(data_mtx_);
        cache2_ = msg.ptr();
        cache2_ts_ = msg.acq_time;
        try_merge();
    }

    void on_tf_lidar1_to_target_callback(const fins::Msg<geometry_msgs::msg::TransformStamped>& msg) {
        std::lock_guard<std::mutex> lock(data_mtx_);
        tf_lidar1_to_target_ = tf2::transformToEigen(*msg);
        target_frame_id_ = msg->header.frame_id;
        tf_lidar1_to_target_received_ = true;
    }

    void on_tf_lidar2_to_target_callback(const fins::Msg<geometry_msgs::msg::TransformStamped>& msg) {
        std::lock_guard<std::mutex> lock(data_mtx_);
        tf_lidar2_to_target_ = tf2::transformToEigen(*msg);
        tf_lidar2_to_target_received_ = true;
    }

    void try_merge() {
        if (!cache1_ || !cache2_) return;

        double t1 = fins::to_seconds(cache1_ts_);
        double t2 = fins::to_seconds(cache2_ts_);
        double dt = std::abs(t1 - t2);

        if (dt > 0.005) {
            if (t1 < t2) {
                cache1_.reset();
            } else {
                cache2_.reset();
            }
            return;
        }

        if (!tf_lidar1_to_target_received_ || !tf_lidar2_to_target_received_) {
            return;
        }

        auto msg1 = cache1_;
        auto msg2 = cache2_;
        auto ts1 = cache1_ts_;
        cache1_.reset();
        cache2_.reset();

        // T_lidar1_lidar2 = T_target_lidar1^-1 * T_target_lidar2
        Eigen::Isometry3d tf_lidar2_to_1 = tf_lidar1_to_target_.inverse() * tf_lidar2_to_target_;

        do_merge(msg1, msg2, ts1, tf_lidar2_to_1);
    }

    void do_merge(const CustomMsg::ConstSharedPtr& msg1, const CustomMsg::ConstSharedPtr& msg2, fins::AcqTime ts, const Eigen::Isometry3d& tf_lidar2_to_1) {
        if (msg1->points.empty() || msg2->points.empty()) return;

        uint64_t dt_ns = 0;
        uint8_t earlier_lidar_id = (msg2->timebase > msg1->timebase) ? 1 : 2;
        dt_ns = (earlier_lidar_id == 1) ? (msg2->timebase - msg1->timebase) : (msg1->timebase - msg2->timebase);

        CustomMsg merged;
        merged.lidar_id = 0;
        merged.points.reserve(msg1->points.size() + msg2->points.size());

        double blind_sq = config_.lidar_blind_zone * config_.lidar_blind_zone;
        bool check_blind = (config_.lidar_blind_zone > 1e-4);
        bool enable_box = config_.enable_box_filter;

        auto is_in_box = [&](const Eigen::Vector3d& pt_target) {
            return (pt_target.x() >= config_.filter_box_min_x && pt_target.x() <= config_.filter_box_max_x &&
                    pt_target.y() >= config_.filter_box_min_y && pt_target.y() <= config_.filter_box_max_y &&
                    pt_target.z() >= config_.filter_box_min_z && pt_target.z() <= config_.filter_box_max_z);
        };

        // Process msg1
        for (const auto& pt : msg1->points) {
            if (check_blind && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) < blind_sq) continue;
            
            if (enable_box) {
                Eigen::Vector3d pt_target = tf_lidar1_to_target_ * Eigen::Vector3d(pt.x, pt.y, pt.z);
                if (is_in_box(pt_target)) continue;
            }
            
            CustomPoint new_pt = pt;
            new_pt.offset_time = pt.offset_time + (earlier_lidar_id == 1 ? 0 : dt_ns);
            merged.points.push_back(new_pt);
        }

        // Process msg2
        for (const auto& pt : msg2->points) {
            if (check_blind && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) < blind_sq) continue;

            if (enable_box) {
                Eigen::Vector3d pt_target = tf_lidar2_to_target_ * Eigen::Vector3d(pt.x, pt.y, pt.z);
                if (is_in_box(pt_target)) continue;
            }

            Eigen::Vector3d pt_in_lidar1 = tf_lidar2_to_1 * Eigen::Vector3d(pt.x, pt.y, pt.z);
            
            CustomPoint new_pt = pt;
            new_pt.x = pt_in_lidar1.x();
            new_pt.y = pt_in_lidar1.y();
            new_pt.z = pt_in_lidar1.z();
            new_pt.offset_time = pt.offset_time + (earlier_lidar_id == 2 ? 0 : dt_ns);
            merged.points.push_back(new_pt);
        }

        merged.point_num = merged.points.size();
        merged.header.stamp = (earlier_lidar_id == 1) ? msg1->header.stamp : msg2->header.stamp;
        merged.header.frame_id = msg1->header.frame_id;
        merged.timebase = (earlier_lidar_id == 1) ? msg1->timebase : msg2->timebase;

        send("merged_cloud", merged, ts);

        if (enable_box && required("filter_box_marker")) {
            publish_filter_box_marker(ts);
        }
    }

    void publish_filter_box_marker(fins::AcqTime ts) {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;
    
        marker.header.frame_id = target_frame_id_;
        marker.ns = "filter_box";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = (config_.filter_box_min_x + config_.filter_box_max_x) / 2.0;
        marker.pose.position.y = (config_.filter_box_min_y + config_.filter_box_max_y) / 2.0;
        marker.pose.position.z = (config_.filter_box_min_z + config_.filter_box_max_z) / 2.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = std::abs(config_.filter_box_max_x - config_.filter_box_min_x);
        marker.scale.y = std::abs(config_.filter_box_max_y - config_.filter_box_min_y);
        marker.scale.z = std::abs(config_.filter_box_max_z - config_.filter_box_min_z);

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.4;

        marker_array.markers.push_back(marker);
        send("filter_box_marker", marker_array, ts);
    }

    Config config_;
    std::mutex data_mtx_;
    
    CustomMsg::ConstSharedPtr cache1_;
    CustomMsg::ConstSharedPtr cache2_;
    fins::AcqTime cache1_ts_;
    fins::AcqTime cache2_ts_;
    
    std::string target_frame_id_;
    Eigen::Isometry3d tf_lidar1_to_target_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d tf_lidar2_to_target_{Eigen::Isometry3d::Identity()};
    
    bool tf_lidar1_to_target_received_{false};
    bool tf_lidar2_to_target_received_{false};
};

EXPORT_NODE(MergeCustomMsgNode)
DEFINE_PLUGIN_ENTRY(fins::STATELESS)
