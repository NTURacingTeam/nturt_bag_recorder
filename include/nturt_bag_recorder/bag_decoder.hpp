/**
 * @file bag_decoder.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for decoding bag file to csv.
 */

#ifndef BAG_DECODER_HPP
#define BAG_DECODER_HPP

// glibc include
#include <stdint.h>

// std include
#include <fstream>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>
#include <variant>

// ros2 include
#include <rclcpp/serialization.hpp>

// ros2 bag include
#include <rosbag2_cpp/readers/sequential_reader.hpp>

// ros2 message include
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// nturt include
#include "nturt_bag_recorder/bag_decoder_arg_parser.hpp"
#include "nturt_bag_recorder/csv_writer.hpp"
#include "nturt_can_config.h"
#include "nturt_can_config/battery_utils.hpp"
#include "nturt_can_config_logger-binutil.h"
#include "nturt_ros_interface/msg/system_stats.hpp"

template <typename T>
class DataLogger {
 public:
  /**
   * @brief Constructor of DataLogger.
   *
   * @param output_file Name of the file to write to (without the csv file
   * extension).
   * @param header Header for each field of the data.
   * @param update_function Callback function to get the vector for the csv
   * file.
   * @param period Time between each time data is logged in [s].
   */
  DataLogger(const std::string &output_file,
             const std::vector<std::string> &header,
             std::function<std::vector<T>()> update_function, double period)
      : csv_writer_(output_file + ".csv"),
        update_function_(update_function),
        period_(period) {
    csv_writer_.get_stream() << "time,";
    csv_writer_.write_row(header);
  }

  /**
   * @brief Initialize time counter of the logger.
   *
   * @param time Current time;
   */
  void init_time(double time);

  /**
   * @brief Update the logger.
   *
   * @param time Current time.
   */
  void update(double time);

  /**
   * @brief Write raw data to the csv file.
   *
   * @param data Raw data to write.
   */
  void write_raw(std::string data);

 private:
  /// @brief Writer for csv file.
  CsvWriter csv_writer_;

  /// @brief Callback function to get the vector for the csv file.
  std::function<std::vector<T>()> update_function_;

  /// @brief Time between each data is logged in [s].
  double period_;

  /// @brief Last written time of the logger.
  double time_;
};

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for decoding bag file to csv.
 */
class BagDecoder {
 public:
  /// @brief Constructor of BagDecoder.
  BagDecoder(BagDecoderArg arg);

  /// @brief Start decoding bag file to csv.
  void run();

 private:
  /**
   * @brief Function to update local data buffer.
   *
   * @param msg Bag message.
   */
  void update_data(rosbag2_storage::SerializedBagMessageSharedPtr msg);

  /**
   * @brief Function to get status data to write to csv file.
   *
   * @return std::vector<uint32_t> Data to write to csv file.
   */
  std::vector<uint32_t> onUpdateStatus();

  /**
   * @brief Function to get sensor data to write status data to csv file.
   *
   * @return std::vector<double> Data to write to csv file.
   */
  std::vector<double> onUpdateSensor();

  /**
   * @brief Function to get imu data to write status data to csv file.
   *
   * @return std::vector<double> Data to write to csv file.
   */
  std::vector<double> onUpdateImu();

  /**
   * @brief Function to get gps data to write status data to csv file.
   *
   * @return std::vector<std::string> Data to write to csv file.
   */
  std::vector<std::string> onUpdateGps();

  /**
   * @brief Function to get battery data to write status data to csv file.
   *
   * @return std::vector<double> Data to write to csv file.
   */
  std::vector<double> onUpdateBattery();

  /**
   * @brief Function to get inverter data to write status data to csv file.
   *
   * @return std::vector<double> Data to write to csv file.
   */
  std::vector<double> onUpdateInverterData();

  /**
   * @brief Function to get system stats data to write status data to csv file.
   *
   * @return std::vector<double> Data to write to csv file.
   */
  std::vector<double> onUpdateSystemStats();

  /**
   * @brief Function to get unified data to write to a unified csv file
   * 
   * @author CHYang25 chris920325@gmail.com
   * 
   * @return std::vector<std::string> Data to write to unified csv file
   * 
   */
  std::vector<std::string> onUpdateUnifiedData();

  /* coder dbc callback function ---------------------------------------------*/
  uint32_t get_tick();

  /// @brief Map to convert log level to string.
  static const std::unordered_map<int, std::string> log_level_to_string_;

  /// @brief Map to convert gps fix code to string.
  static const std::unordered_map<int, std::string> gps_fix_to_string_;

  /// @brief Map to convert gps covarience type code to string.
  static const std::unordered_map<int, std::string>
      gps_covarience_type_to_string_;

  /// @brief Header for system status.
  static const std::vector<std::string> status_header_;

  /// @brief Header for sensor data.
  static const std::vector<std::string> sensor_header_;

  /// @brief Header for imu data.
  static const std::vector<std::string> imu_header_;

  /// @brief Header for gps data.
  static const std::vector<std::string> gps_header_;

  /// @brief Header for battery data.
  static const std::vector<std::string> battery_header_;

  /// @brief Header for inverter data.
  static const std::vector<std::string> inverter_data_header_;

  /// @brief Header for system stats.
  static const std::vector<std::string> system_stats_header_;

  /// @brief Header for unified data.
  /// @author CHYang25 chris920325@gmail.com
  static const std::vector<std::string> unified_data_header_;

  /// @brief Struct containing command line arguments.
  BagDecoderArg arg_;

  /// @brief Last updated time.
  double time_;

  /// @brief Struct for storing can frame data.
  nturt_can_config_logger_rx_t can_rx_;

  /// @brief Struct for storing battery data.
  BatteryData battery_data_;

  /// @brief Struct for storing "/fix" message data.
  sensor_msgs::msg::NavSatFix gps_fix_;

  /// @brief Struct for storing "/vel" message data.
  geometry_msgs::msg::TwistStamped gps_vel_;

  /// @brief Struct for storing "/system_stats" message data.
  nturt_ros_interface::msg::SystemStats system_stats_;

  /// @brief Serializer for deserializing "/rosout" message.
  rclcpp::Serialization<rcl_interfaces::msg::Log> log_serializer_;

  /// @brief Serializer for deserializing "/from_can_bus" message.
  rclcpp::Serialization<can_msgs::msg::Frame> frame_serializer_;

  /// @brief Serializer for deserializing "/fix" message.
  rclcpp::Serialization<sensor_msgs::msg::NavSatFix> nav_sat_fix_serializer_;

  /// @brief Serializer for deserializing "/vel" message.
  rclcpp::Serialization<geometry_msgs::msg::TwistStamped>
      twist_stamped_serializer_;

  /// @brief Serializer for deserializing "/system_stats" message.
  rclcpp::Serialization<nturt_ros_interface::msg::SystemStats>
      system_stats_serializer_;

  /// @brief CSV writter for roslog.
  CsvWriter roslog_writter_;

  /// @brief Data logger for logging system status.
  DataLogger<uint32_t> status_logger_;

  /// @brief Data logger for logging sensor data.
  DataLogger<double> sensor_logger_;

  /// @brief Data logger for logging imu data.
  DataLogger<double> imu_logger_;

  /// @brief Data logger for logging gps data.
  DataLogger<std::string> gps_logger_;

  /// @brief Data logger for logging battery data.
  DataLogger<double> battery_logger_;

  /// @brief Data logger for logging inverter data.
  DataLogger<double> inverter_data_logger_;

  /// @brief Data logger for logging system stats.
  DataLogger<double> system_stats_logger_;

  /// @brief Data logger for logging unified stats
  /// @author CHYang25 chris920325@gmail.com
  DataLogger<std::string> unified_logger_;
};

/**
 * @brief Create a directory "path", exit the program if the directory already
 * exist.
 *
 * @param path The name of the path.
 */
void create_directory(const std::string &path);

#endif  // BAG_DECODER_HPP
