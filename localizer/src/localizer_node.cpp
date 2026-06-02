#include <queue>
#include <deque>
#include <mutex>
#include <cmath>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    double update_hz = 1.0;
    bool fix_tf_z = false;
    double fixed_tf_z = 0.0;
    bool fix_robot_z = false;
    double fixed_robot_z = 0.0;
    bool enable_pose_guard = true;
    double max_translation_jump = 0.8;
    double max_yaw_jump = 15.0 * 3.14159265358979323846 / 180.0;
    double max_tf_translation_jump = 0.35;
    double max_tf_yaw_jump = 8.0 * 3.14159265358979323846 / 180.0;
    int recovery_stable_count = 4;
    double recovery_stable_translation = 0.15;
    double recovery_stable_yaw = 3.0 * 3.14159265358979323846 / 180.0;
    bool enable_drift_guard = true;
    double drift_window_sec = 5.0;
    double max_correction_drift = 0.4;
    double max_correction_yaw = 10.0 * 3.14159265358979323846 / 180.0;
    double drift_cooldown_sec = 5.0;
    bool enable_velocity_guard = true;
    double max_robot_speed = 1.0;
    double max_robot_yaw_rate = 100.0 * 3.14159265358979323846 / 180.0;
    double velocity_history_sec = 3.0;
    double velocity_recovery_age_sec = 0.7;
    double velocity_recovery_cooldown_sec = 2.0;
};

struct CorrectionSample
{
    double stamp = 0.0;
    V3D translation = V3D::Zero();
    double yaw = 0.0;
};

struct PoseSample
{
    double stamp = 0.0;
    M3D rotation = M3D::Identity();
    V3D translation = V3D::Zero();
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
    bool has_good_pose = false;
    int rejected_alignment_count = 0;
    bool has_rejected_candidate = false;
    int stable_rejected_candidate_count = 0;
    M3D last_rejected_offset_r = M3D::Identity();
    V3D last_rejected_offset_t = V3D::Zero();
    std::deque<CorrectionSample> correction_history;
    double drift_cooldown_until = 0.0;
    std::deque<PoseSample> good_pose_history;
    double velocity_recovery_cooldown_until = 0.0;
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        loadParameters();
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);

        m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        m_config.update_hz = config["update_hz"].as<double>();
        m_config.fix_tf_z = config["fix_tf_z"] ? config["fix_tf_z"].as<bool>() : false;
        m_config.fixed_tf_z = config["fixed_tf_z"] ? config["fixed_tf_z"].as<double>() : 0.0;
        m_config.fix_robot_z = config["fix_robot_z"] ? config["fix_robot_z"].as<bool>() : false;
        m_config.fixed_robot_z = config["fixed_robot_z"] ? config["fixed_robot_z"].as<double>() : 0.0;
        m_config.enable_pose_guard = config["enable_pose_guard"] ? config["enable_pose_guard"].as<bool>() : true;
        m_config.max_translation_jump = config["max_translation_jump"] ? config["max_translation_jump"].as<double>() : 0.8;
        double max_yaw_jump_deg = config["max_yaw_jump_deg"] ? config["max_yaw_jump_deg"].as<double>() : 15.0;
        m_config.max_yaw_jump = max_yaw_jump_deg * 3.14159265358979323846 / 180.0;
        m_config.max_tf_translation_jump = config["max_tf_translation_jump"] ? config["max_tf_translation_jump"].as<double>() : 0.35;
        double max_tf_yaw_jump_deg = config["max_tf_yaw_jump_deg"] ? config["max_tf_yaw_jump_deg"].as<double>() : 8.0;
        m_config.max_tf_yaw_jump = max_tf_yaw_jump_deg * 3.14159265358979323846 / 180.0;
        m_config.recovery_stable_count = config["recovery_stable_count"] ? config["recovery_stable_count"].as<int>() : 4;
        m_config.recovery_stable_translation = config["recovery_stable_translation"] ? config["recovery_stable_translation"].as<double>() : 0.15;
        double recovery_stable_yaw_deg = config["recovery_stable_yaw_deg"] ? config["recovery_stable_yaw_deg"].as<double>() : 3.0;
        m_config.recovery_stable_yaw = recovery_stable_yaw_deg * 3.14159265358979323846 / 180.0;
        m_config.enable_drift_guard = config["enable_drift_guard"] ? config["enable_drift_guard"].as<bool>() : true;
        m_config.drift_window_sec = config["drift_window_sec"] ? config["drift_window_sec"].as<double>() : 5.0;
        m_config.max_correction_drift = config["max_correction_drift"] ? config["max_correction_drift"].as<double>() : 0.4;
        double max_correction_yaw_deg = config["max_correction_yaw_deg"] ? config["max_correction_yaw_deg"].as<double>() : 10.0;
        m_config.max_correction_yaw = max_correction_yaw_deg * 3.14159265358979323846 / 180.0;
        m_config.drift_cooldown_sec = config["drift_cooldown_sec"] ? config["drift_cooldown_sec"].as<double>() : 5.0;
        m_config.enable_velocity_guard = config["enable_velocity_guard"] ? config["enable_velocity_guard"].as<bool>() : true;
        m_config.max_robot_speed = config["max_robot_speed"] ? config["max_robot_speed"].as<double>() : 1.0;
        double max_robot_yaw_rate_deg = config["max_robot_yaw_rate_deg"] ? config["max_robot_yaw_rate_deg"].as<double>() : 100.0;
        m_config.max_robot_yaw_rate = max_robot_yaw_rate_deg * 3.14159265358979323846 / 180.0;
        m_config.velocity_history_sec = config["velocity_history_sec"] ? config["velocity_history_sec"].as<double>() : 3.0;
        m_config.velocity_recovery_age_sec = config["velocity_recovery_age_sec"] ? config["velocity_recovery_age_sec"].as<double>() : 0.7;
        m_config.velocity_recovery_cooldown_sec = config["velocity_recovery_cooldown_sec"] ? config["velocity_recovery_cooldown_sec"].as<double>() : 2.0;

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();
        m_localizer_config.rough_max_correspondence_distance = config["rough_max_correspondence_distance"] ? config["rough_max_correspondence_distance"].as<double>() : 1.5;

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();
        m_localizer_config.refine_max_correspondence_distance = config["refine_max_correspondence_distance"] ? config["refine_max_correspondence_distance"].as<double>() : 0.7;
    }
    void timerCB()
    {
        if (!m_state.message_received)
            return;

        rclcpp::Duration diff = rclcpp::Clock().now() - m_state.last_send_tf_time;

        bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_received;

        if (!update_tf)
        {
            sendBroadCastTF(m_state.last_message_time);
            return;
        }

        m_state.last_send_tf_time = rclcpp::Clock().now();

        M4F initial_guess = M4F::Identity();
        bool using_service_guess = false;
        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
        }
        if (m_state.service_received)
        {
            std::lock_guard<std::mutex>(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
            using_service_guess = true;
            // m_state.service_received = false;
        }
        else
        {
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * current_local_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * current_local_t + m_state.last_offset_t).cast<float>();
        }
        M4F predicted_pose = initial_guess;

        bool result = m_localizer->align(initial_guess);
        if (result)
        {
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            M3D proposed_offset_r = map_body_r * current_local_r.transpose();
            V3D proposed_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
            double now_sec = this->get_clock()->now().seconds();

            double translation_jump = 0.0;
            double yaw_jump = 0.0;
            double tf_translation_jump = 0.0;
            double tf_yaw_jump = 0.0;
            bool pose_jump_detected = m_config.enable_pose_guard && m_state.has_good_pose && !using_service_guess &&
                                      isPoseJump(initial_guess, predicted_pose, translation_jump, yaw_jump);
            bool tf_jump_detected = m_config.enable_pose_guard && m_state.has_good_pose && !using_service_guess &&
                                    isTfJump(proposed_offset_r, proposed_offset_t, tf_translation_jump, tf_yaw_jump);
            bool jump_detected = pose_jump_detected || tf_jump_detected;
            if (jump_detected)
            {
                updateRejectedCandidate(proposed_offset_r, proposed_offset_t);
                if (m_state.stable_rejected_candidate_count >= m_config.recovery_stable_count)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 1000,
                        "Accept stable recovery correction after %d rejects: pose_translation=%.3fm pose_yaw=%.1fdeg tf_translation=%.3fm tf_yaw=%.1fdeg",
                        m_state.stable_rejected_candidate_count,
                        translation_jump,
                        yaw_jump * 180.0 / 3.14159265358979323846,
                        tf_translation_jump,
                        tf_yaw_jump * 180.0 / 3.14159265358979323846);
                }
                else
                {
                    ++m_state.rejected_alignment_count;
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 1000,
                        "Reject localization jump: pose_translation=%.3fm pose_yaw=%.1fdeg tf_translation=%.3fm tf_yaw=%.1fdeg rejected_count=%d stable_candidate_count=%d/%d",
                        translation_jump,
                        yaw_jump * 180.0 / 3.14159265358979323846,
                        tf_translation_jump,
                        tf_yaw_jump * 180.0 / 3.14159265358979323846,
                        m_state.rejected_alignment_count,
                        m_state.stable_rejected_candidate_count,
                        m_config.recovery_stable_count);
                    sendBroadCastTF(current_time);
                    publishMapCloud(current_time);
                    return;
                }
            }

            double drift_translation = 0.0;
            double drift_yaw = 0.0;
            if (m_config.enable_drift_guard && m_state.has_good_pose && !using_service_guess)
            {
                if (now_sec < m_state.drift_cooldown_until)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 1000,
                        "Skip localization correction during drift cooldown: remaining=%.2fs",
                        m_state.drift_cooldown_until - now_sec);
                    sendBroadCastTF(current_time);
                    publishMapCloud(current_time);
                    return;
                }

                if (isCorrectionDrifting(proposed_offset_r, proposed_offset_t, now_sec, drift_translation, drift_yaw))
                {
                    m_state.drift_cooldown_until = now_sec + m_config.drift_cooldown_sec;
                    m_state.correction_history.clear();
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Freeze localization correction: accumulated_translation=%.3fm accumulated_yaw=%.1fdeg cooldown=%.1fs",
                        drift_translation,
                        drift_yaw * 180.0 / 3.14159265358979323846,
                        m_config.drift_cooldown_sec);
                    sendBroadCastTF(current_time);
                    publishMapCloud(current_time);
                    return;
                }
            }

            double robot_speed = 0.0;
            double robot_yaw_rate = 0.0;
            if (m_config.enable_velocity_guard && m_state.has_good_pose && !using_service_guess &&
                isVelocityJump(map_body_r, map_body_t, now_sec, robot_speed, robot_yaw_rate))
            {
                ++m_state.rejected_alignment_count;
                recoverFromLastGoodPose(now_sec);
                RCLCPP_WARN(
                    this->get_logger(),
                    "Reject impossible localization velocity: speed=%.3fm/s yaw_rate=%.1fdeg/s rejected_count=%d",
                    robot_speed,
                    robot_yaw_rate * 180.0 / 3.14159265358979323846,
                    m_state.rejected_alignment_count);
                sendBroadCastTF(current_time);
                publishMapCloud(current_time);
                return;
            }

            m_state.last_offset_r = proposed_offset_r;
            m_state.last_offset_t = proposed_offset_t;
            m_state.has_good_pose = true;
            m_state.rejected_alignment_count = 0;
            m_state.has_rejected_candidate = false;
            m_state.stable_rejected_candidate_count = 0;
            saveGoodPose(map_body_r, map_body_t, now_sec);
            if (!m_state.localize_success && m_state.service_received)
            {
                std::lock_guard<std::mutex>(m_state.service_mutex);
                m_state.localize_success = true;
                m_state.service_received = false;
            }
        }
        sendBroadCastTF(current_time);
        publishMapCloud(current_time);
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex>(m_state.message_mutex);

        pcl::fromROSMsg(*cloud_msg, *m_state.last_cloud);

        m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
        m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
        m_state.last_message_time = cloud_msg->header.stamp;
        if (!m_state.message_received)
        {
            m_state.message_received = true;
            m_config.local_frame = odom_msg->header.frame_id;
        }
    }

    bool isPoseJump(const M4F &candidate_pose, const M4F &predicted_pose, double &translation_jump, double &yaw_jump) const
    {
        V3D candidate_t = candidate_pose.block<3, 1>(0, 3).cast<double>();
        V3D predicted_t = predicted_pose.block<3, 1>(0, 3).cast<double>();
        translation_jump = (candidate_t - predicted_t).norm();

        M3D candidate_r = candidate_pose.block<3, 3>(0, 0).cast<double>();
        M3D predicted_r = predicted_pose.block<3, 3>(0, 0).cast<double>();
        M3D delta_r = predicted_r.transpose() * candidate_r;
        yaw_jump = std::abs(std::atan2(delta_r(1, 0), delta_r(0, 0)));

        return translation_jump > m_config.max_translation_jump || yaw_jump > m_config.max_yaw_jump;
    }

    bool isTfJump(const M3D &candidate_offset_r, const V3D &candidate_offset_t, double &translation_jump, double &yaw_jump) const
    {
        translation_jump = (candidate_offset_t - m_state.last_offset_t).norm();

        M3D delta_r = m_state.last_offset_r.transpose() * candidate_offset_r;
        yaw_jump = std::abs(std::atan2(delta_r(1, 0), delta_r(0, 0)));

        return translation_jump > m_config.max_tf_translation_jump || yaw_jump > m_config.max_tf_yaw_jump;
    }

    void updateRejectedCandidate(const M3D &candidate_offset_r, const V3D &candidate_offset_t)
    {
        double translation_delta = 0.0;
        double yaw_delta = 0.0;
        bool stable_with_previous = false;
        if (m_state.has_rejected_candidate)
        {
            translation_delta = (candidate_offset_t - m_state.last_rejected_offset_t).norm();
            M3D delta_r = m_state.last_rejected_offset_r.transpose() * candidate_offset_r;
            yaw_delta = std::abs(std::atan2(delta_r(1, 0), delta_r(0, 0)));
            stable_with_previous = translation_delta < m_config.recovery_stable_translation &&
                                   yaw_delta < m_config.recovery_stable_yaw;
        }

        if (stable_with_previous)
            ++m_state.stable_rejected_candidate_count;
        else
            m_state.stable_rejected_candidate_count = 1;

        m_state.has_rejected_candidate = true;
        m_state.last_rejected_offset_r = candidate_offset_r;
        m_state.last_rejected_offset_t = candidate_offset_t;
    }

    bool isCorrectionDrifting(const M3D &candidate_offset_r,
                              const V3D &candidate_offset_t,
                              double now_sec,
                              double &drift_translation,
                              double &drift_yaw)
    {
        V3D delta_t = candidate_offset_t - m_state.last_offset_t;
        M3D delta_r = m_state.last_offset_r.transpose() * candidate_offset_r;
        double delta_yaw = std::atan2(delta_r(1, 0), delta_r(0, 0));

        m_state.correction_history.push_back({now_sec, delta_t, delta_yaw});
        while (!m_state.correction_history.empty() &&
               now_sec - m_state.correction_history.front().stamp > m_config.drift_window_sec)
        {
            m_state.correction_history.pop_front();
        }

        V3D accumulated_t = V3D::Zero();
        double accumulated_yaw = 0.0;
        for (const auto &sample : m_state.correction_history)
        {
            accumulated_t += sample.translation;
            accumulated_yaw += sample.yaw;
        }

        drift_translation = accumulated_t.norm();
        drift_yaw = std::abs(accumulated_yaw);
        return drift_translation > m_config.max_correction_drift ||
               drift_yaw > m_config.max_correction_yaw;
    }

    bool isVelocityJump(const M3D &candidate_r,
                        const V3D &candidate_t,
                        double now_sec,
                        double &speed,
                        double &yaw_rate) const
    {
        if (m_state.good_pose_history.empty())
            return false;

        const PoseSample &last_good = m_state.good_pose_history.back();
        double dt = now_sec - last_good.stamp;
        if (dt < 0.05)
            return false;

        speed = (candidate_t - last_good.translation).norm() / dt;
        M3D delta_r = last_good.rotation.transpose() * candidate_r;
        yaw_rate = std::abs(std::atan2(delta_r(1, 0), delta_r(0, 0))) / dt;

        return speed > m_config.max_robot_speed ||
               yaw_rate > m_config.max_robot_yaw_rate;
    }

    void saveGoodPose(const M3D &pose_r, const V3D &pose_t, double now_sec)
    {
        m_state.good_pose_history.push_back({now_sec, pose_r, pose_t});
        while (!m_state.good_pose_history.empty() &&
               now_sec - m_state.good_pose_history.front().stamp > m_config.velocity_history_sec)
        {
            m_state.good_pose_history.pop_front();
        }
    }

    bool pickRecoveryPose(double now_sec, PoseSample &recovery_pose) const
    {
        if (m_state.good_pose_history.empty())
            return false;

        double target_stamp = now_sec - m_config.velocity_recovery_age_sec;
        recovery_pose = m_state.good_pose_history.front();
        for (const auto &sample : m_state.good_pose_history)
        {
            if (sample.stamp <= target_stamp)
                recovery_pose = sample;
            else
                break;
        }
        return true;
    }

    void recoverFromLastGoodPose(double now_sec)
    {
        if (now_sec < m_state.velocity_recovery_cooldown_until)
            return;

        PoseSample recovery_pose;
        if (!pickRecoveryPose(now_sec, recovery_pose))
            return;

        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = recovery_pose.rotation.cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = recovery_pose.translation.cast<float>();
            m_state.service_received = true;
            m_state.localize_success = false;
        }

        m_state.velocity_recovery_cooldown_until = now_sec + m_config.velocity_recovery_cooldown_sec;
        m_state.correction_history.clear();
        m_state.drift_cooldown_until = 0.0;
        RCLCPP_WARN(
            this->get_logger(),
            "Recovery initial guess from last good pose: x=%.3f y=%.3f yaw=%.1fdeg cooldown=%.1fs",
            recovery_pose.translation.x(),
            recovery_pose.translation.y(),
            std::atan2(recovery_pose.rotation(1, 0), recovery_pose.rotation(0, 0)) * 180.0 / 3.14159265358979323846,
            m_config.velocity_recovery_cooldown_sec);
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        if (m_config.fix_robot_z)
            t.z() = m_config.fixed_robot_z - (m_state.last_offset_r * m_state.last_t).z();
        else if (m_config.fix_tf_z)
            t.z() = m_config.fixed_tf_z;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        bool load_flag = m_localizer->loadMap(pcd_path);
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            return;
        }
        {
            std::lock_guard<std::mutex>(m_state.service_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            m_state.localize_success = false;
            m_state.correction_history.clear();
            m_state.drift_cooldown_until = 0.0;
            m_state.good_pose_history.clear();
            m_state.velocity_recovery_cooldown_until = 0.0;
        }

        response->success = true;
        response->message = "relocalize success";
        return;
    }

    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex>(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_state.localize_success;
        return;
    }
    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1)
            return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1)
            return;
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

private:
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
