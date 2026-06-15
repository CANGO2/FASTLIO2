#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

class DynamicObjectFilterNode : public rclcpp::Node
{
public:
  DynamicObjectFilterNode() : Node("dynamic_object_filter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/points_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/points_raw_static");
    enabled_ = declare_parameter<bool>("enabled", true);
    remove_min_range_ = declare_parameter<double>("remove_min_range", 0.2);
    remove_max_range_ = declare_parameter<double>("remove_max_range", 1.5);
    remove_min_z_ = declare_parameter<double>("remove_min_z", 0.1);
    remove_max_z_ = declare_parameter<double>("remove_max_z", 1.8);

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      rclcpp::QoS(10).reliable());

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DynamicObjectFilterNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Dynamic object filter: %s -> %s, enabled=%s, range=[%.2f, %.2f], z=[%.2f, %.2f]",
      input_topic_.c_str(),
      output_topic_.c_str(),
      enabled_ ? "true" : "false",
      remove_min_range_,
      remove_max_range_,
      remove_min_z_,
      remove_max_z_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (!enabled_)
    {
      cloud_pub_->publish(*msg);
      return;
    }

    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;

    for (const auto &field : msg->fields)
    {
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32)
        continue;

      if (field.name == "x")
        x_offset = static_cast<int>(field.offset);
      else if (field.name == "y")
        y_offset = static_cast<int>(field.offset);
      else if (field.name == "z")
        z_offset = static_cast<int>(field.offset);
    }

    if (x_offset < 0 || y_offset < 0 || z_offset < 0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "PointCloud2 has no FLOAT32 x/y/z fields. Passing cloud through.");
      cloud_pub_->publish(*msg);
      return;
    }

    sensor_msgs::msg::PointCloud2 out = *msg;
    out.data.clear();
    out.data.reserve(msg->data.size());
    out.height = 1;
    out.is_dense = false;

    const std::size_t point_count =
      static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);

    std::uint32_t kept_count = 0;

    for (std::size_t i = 0; i < point_count; ++i)
    {
      const std::uint8_t *point_ptr = msg->data.data() + i * msg->point_step;

      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      std::memcpy(&x, point_ptr + x_offset, sizeof(float));
      std::memcpy(&y, point_ptr + y_offset, sizeof(float));
      std::memcpy(&z, point_ptr + z_offset, sizeof(float));

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        continue;

      const double range_xy = std::hypot(static_cast<double>(x), static_cast<double>(y));
      const bool in_dynamic_person_band =
        range_xy >= remove_min_range_ &&
        range_xy <= remove_max_range_ &&
        static_cast<double>(z) >= remove_min_z_ &&
        static_cast<double>(z) <= remove_max_z_;

      if (in_dynamic_person_band)
        continue;

      out.data.insert(out.data.end(), point_ptr, point_ptr + msg->point_step);
      ++kept_count;
    }

    out.width = kept_count;
    out.row_step = out.point_step * out.width;
    cloud_pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  bool enabled_ = true;
  double remove_min_range_ = 0.2;
  double remove_max_range_ = 1.5;
  double remove_min_z_ = 0.1;
  double remove_max_z_ = 1.8;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicObjectFilterNode>());
  rclcpp::shutdown();
  return 0;
}
