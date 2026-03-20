#include "utils.h"

struct HesaiPoint {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(HesaiPoint,
    (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::velo2PCL(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<HesaiPoint> cloud_raw;
    pcl::fromROSMsg(*msg, cloud_raw);

    double scan_start_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZINormal>);
    output->reserve(cloud_raw.size() / filter_num + 1);

    for (size_t i = 0; i < cloud_raw.size(); i += filter_num)
    {
        const auto &pt = cloud_raw.points[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;
        float dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (dist_sq < min_range * min_range || dist_sq > max_range * max_range)
            continue;

        pcl::PointXYZINormal p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        p.normal_x = 0.0;
        p.normal_y = 0.0;
        p.normal_z = 0.0;

        // ★ 핵심: 상대 시간(ms) → curvature에 저장
        double rel_time = (pt.timestamp - scan_start_time) * 1000.0;  // ms
        p.curvature = static_cast<float>(rel_time);

        output->push_back(p);
    }

    std::cout << "Output point cloud size: " << output->size() << std::endl;
    return output;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
