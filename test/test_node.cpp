#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

// rosbag2 include
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/info.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/bag_metadata.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node node("nturt_bag_recorder_test");

  rosbag2_storage::StorageOptions storage_options{};

  storage_options.uri = argv[1];
  storage_options.storage_id = "";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::readers::SequentialReader reader;
  reader.open(storage_options, converter_options);

  const auto topics = reader.get_all_topics_and_types();

  for (const auto& topic : topics)
    RCLCPP_INFO(node.get_logger(), topic.name.c_str());

  auto serializer = rclcpp::Serialization<can_msgs::msg::Frame>();
  auto msg = std::make_shared<can_msgs::msg::Frame>();

  while (reader.has_next()) {
    // serialized data
    auto serialized_message = reader.read_next();
    rclcpp::SerializedMessage extracted_serialized_msg(
        *serialized_message->serialized_data);
    auto topic = serialized_message->topic_name;
    if (topic.find("/from_can_bus") != std::string::npos) {
      serializer.deserialize_message(&extracted_serialized_msg, msg.get());
      printf(
          "stamp: sec: %d, nanosec: %d\nid: 0x%X, is_rtr: %s, is_extended: "
          "%s, is_error: %s, dlc: %d\ndata: ",
          msg->header.stamp.sec, msg->header.stamp.nanosec, msg->id,
          msg->is_rtr ? "true" : "false", msg->is_extended ? "true" : "false",
          msg->is_error ? "true" : "false", msg->dlc);

      for (int i = 0; i < msg->dlc; i++) {
        printf("0x%X ", msg->data[i]);
      }
      std::cout << "\n---" << std::endl;
    }
  }

  return 0;
}
