#include "nturt_bag_recorder/bag_decoder.hpp"

// glibc include
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>

// std include
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

// ros2 include
#include <rcutils/time.h>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

// rosbag2 include
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

// nturt include
#include "nturt_bag_recorder/bag_decoder_arg_parser.hpp"
#include "nturt_can_config.h"
#include "nturt_can_config/battery_utils.hpp"
#include "nturt_can_config/can_callback_register.hpp"
#include "nturt_can_config/can_timeout_monitor.hpp"
#include "nturt_can_config_logger-binutil.h"

#define COLOR_REST "\033[0m"
#define HIGHLIGHT "\033[0;1m"
#define COLOR_RED "\033[1;31m"

/* Vector Concatenate Operator -----------------------------------------------*/
/// @brief An operator for vector concatenation
/// @author CHYang25 chris920325@gmail.com
template <typename T>
std::vector<T> operator+(const std::vector<T> &x, const std::vector<T> &y) {
    std::vector<T> vec;
    vec.reserve(x.size() + y.size());
    vec.insert(vec.end(), x.begin(), x.end());
    vec.insert(vec.end(), y.begin(), y.end());
    return vec;
}

/* Class Static vataible -----------------------------------------------------*/
const std::unordered_map<int, std::string> BagDecoder::log_level_to_string_ = {
    {rcl_interfaces::msg::Log::DEBUG, "DEBUG"},
    {rcl_interfaces::msg::Log::INFO, "INFO"},
    {rcl_interfaces::msg::Log::WARN, "WARN"},
    {rcl_interfaces::msg::Log::ERROR, "ERROR"},
    {rcl_interfaces::msg::Log::FATAL, "FATAL"},
};

const std::unordered_map<int, std::string> BagDecoder::gps_fix_to_string_ = {
    {-1, "NO_FIX"}, {0, "NO_FIX"}, {1, "FIX"}, {2, "SBAS_FIX"}, {3, "GBAS_FIX"},

};

const std::unordered_map<int, std::string>
    BagDecoder::gps_covarience_type_to_string_ = {
        {0, "UNKNOWN"},
        {1, "APPROXIMATED"},
        {2, "DIAGONAL_KNOWN"},
        {3, "KNOWN"},
};

const std::vector<std::string> BagDecoder::status_header_ = {
    "can_rx_timeout",      "can_rx_timeout_code", "vcu_status",
    "vcu_error_code",      "rear_sensor_status",  "rear_sensor_error_code",
    "bms_error_code",      "inverter_state",      "inverter_vsm_state",
    "inverter_post_fault", "inverter_run_fault",
};

const std::vector<std::string> BagDecoder::sensor_header_ = []() {
  std::vector<std::string> header;
  header.reserve(32);

  header.push_back("bse");
  header.push_back("apps1");
  header.push_back("apps2");
  header.push_back("brake_micro");
  header.push_back("accelerator_micro");

  header.push_back("steer_angle");
  header.push_back("front_break_pressure");
  header.push_back("rear_break_pressure");

  header.push_back("front_left_wheel_speed");
  header.push_back("front_right_wheel_speed");
  header.push_back("rear_left_wheel_speed");
  header.push_back("rear_right_wheel_speed");

  header.push_back("front_left_suspension_travel");
  header.push_back("front_right_suspension_travel");
  header.push_back("rear_left_suspension_travel");
  header.push_back("rear_right_suspension_travel");

  for (int i = 1; i <= 4; i++) {
    header.push_back("front_left_tire_temp_" + std::to_string(i));
  }

  for (int i = 1; i <= 4; i++) {
    header.push_back("front_right_tire_temp_" + std::to_string(i));
  }

  for (int i = 1; i <= 4; i++) {
    header.push_back("rear_left_tire_temp_" + std::to_string(i));
  }

  for (int i = 1; i <= 4; i++) {
    header.push_back("rear_right_tire_temp_" + std::to_string(i));
  }

  return header;
}();

const std::vector<std::string> BagDecoder::imu_header_ = {
    "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y",
    "gyro_z",  "quat_w",  "quat_x",  "quat_y", "quat_z",
};

const std::vector<std::string> BagDecoder::gps_header_ = {
    "status",        "serivce used",    "longitude",     "latitude",    "altitude",
    "covarience[0]", "covarience[1]",   "covarience[2]", "covarience[3]",
    "covarience[4]", "covarience[5]",   "covarience[6]", "covarience[7]",
    "covarience[8]", "covarience type", "velocity_x",    "velocity_y",
};

const std::vector<std::string> BagDecoder::battery_header_ = []() {
  std::vector<std::string> header;
  header.reserve(2 * NUM_BATTERY_SEGMENT * NUM_BATTERY_CELL_PER_SEGMENT + 1);
  header.push_back("SOC");

  for (int i = 1; i <= NUM_BATTERY_SEGMENT; i++) {
    for (int j = 1; j <= NUM_BATTERY_CELL_PER_SEGMENT; j++) {
      header.push_back("volt_" + std::to_string(i) + "_" + std::to_string(j));
    }
  }

  for (int i = 1; i <= NUM_BATTERY_SEGMENT; i++) {
    for (int j = 1; j <= NUM_BATTERY_CELL_PER_SEGMENT; j++) {
      header.push_back("temp_" + std::to_string(i) + "_" + std::to_string(j));
    }
  }
  return header;
}();

const std::vector<std::string> BagDecoder::inverter_data_header_ = {
    "torque_command",       "torque_feedback",   "motor_speed",
    "dc_bus_voltage",       "dc_bus_current",    "control_board_temperature",
    "hot_spot_temperature", "motor_temperature",
};

const std::vector<std::string> BagDecoder::system_stats_header_ = {
    "cpu_temp", "cpu_usage", "mem_usage", "swap_usage", "disk_usage",
};

const std::vector<std::string> BagDecoder::unified_data_header_ = 
    BagDecoder::sensor_header_ + BagDecoder::imu_header_ + BagDecoder::gps_header_ +
    BagDecoder::battery_header_ + BagDecoder::inverter_data_header_;

/* Class function ------------------------------------------------------------*/
template <typename T>
void DataLogger<T>::init_time(double time) {
  time_ = time;
}

template <typename T>
void DataLogger<T>::update(double time) {
  while (time > time_ + period_) {
    std::ofstream& fstream = csv_writer_.get_stream();
    std::ios_base::fmtflags original_stream_flag = fstream.flags();
    fstream << time_ << ",";
    fstream.flags(original_stream_flag);

    csv_writer_.write_row(update_function_());
    time_ += period_;
  }
}

template <typename T>
void DataLogger<T>::write_raw(std::string data) {
  csv_writer_.get_stream() << data + ",";
}

BagDecoder::BagDecoder(BagDecoderArg arg)
    : arg_(arg),
      roslog_writter_(arg.output_directory + "/roslog.csv"),
      status_logger_(arg.output_directory + "/status", status_header_,
                     std::bind(&BagDecoder::onUpdateStatus, this), 0.01),
      sensor_logger_(arg.output_directory + "/sensor", sensor_header_,
                     std::bind(&BagDecoder::onUpdateSensor, this), 0.01),
      imu_logger_(arg.output_directory + "/imu", imu_header_,
                  std::bind(&BagDecoder::onUpdateImu, this), 0.01),
      gps_logger_(arg.output_directory + "/gps", gps_header_,
                  std::bind(&BagDecoder::onUpdateGps, this), 0.01),
      battery_logger_(arg.output_directory + "/battery", battery_header_,
                      std::bind(&BagDecoder::onUpdateBattery, this), 0.01),
      inverter_data_logger_(
          arg.output_directory + "/inverter_data", inverter_data_header_,
          std::bind(&BagDecoder::onUpdateInverterData, this), 0.01),
      system_stats_logger_(
          arg.output_directory + "/system_stats", system_stats_header_,
          std::bind(&BagDecoder::onUpdateSystemStats, this), 0.01),
      unified_logger_(
          arg.output_directory + "/unified_stats", unified_data_header_,
          std::bind(&BagDecoder::onUpdateUnifiedData, this), 0.01) {
  // init can_rx_
  memset(&can_rx_, 0, sizeof(can_rx_));

  const std::vector<std::string> roslog_header = {
      "time", "level", "name", "msg", "function", "file", "line",
  };
  roslog_writter_.write_row(roslog_header);
}

void BagDecoder::run() {
  CanCallbackRegieter::register_callback(
      static_cast<get_tick_t>(std::bind(&BagDecoder::get_tick, this)));

  // open bag
  rosbag2_storage::StorageOptions storage_options{};

  storage_options.uri = arg_.bag_file;
  storage_options.storage_id = "";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::readers::SequentialReader reader;
  reader.open(storage_options, converter_options);

  // init time of every logger
  rosbag2_storage::SerializedBagMessageSharedPtr serialized_message =
      reader.read_next();

  time_ = static_cast<double>(serialized_message->time_stamp) / 1.0E9;

  status_logger_.init_time(time_);
  sensor_logger_.init_time(time_);
  imu_logger_.init_time(time_);
  gps_logger_.init_time(time_);
  battery_logger_.init_time(time_);
  inverter_data_logger_.init_time(time_);
  system_stats_logger_.init_time(time_);
  unified_logger_.init_time(time_);

  nturt_can_config_logger_Check_Receive_Timeout_Init(&can_rx_);

  while (reader.has_next()) {
    serialized_message = reader.read_next();
    time_ = static_cast<double>(serialized_message->time_stamp) / 1.0E9;

    // write local buffers to csv file before updating the data
    status_logger_.update(time_);
    sensor_logger_.update(time_);
    imu_logger_.update(time_);
    gps_logger_.update(time_);
    battery_logger_.update(time_);
    inverter_data_logger_.update(time_);
    system_stats_logger_.update(time_);
    unified_logger_.update(time_);

    // update local buffers from bag message
    update_data(serialized_message);

    // update can_rx_ timeout
    nturt_can_config_logger_Check_Receive_Timeout(&can_rx_);
  }
}

void BagDecoder::update_data(
    rosbag2_storage::SerializedBagMessageSharedPtr msg) {
  rclcpp::SerializedMessage extracted_serialized_msg(*msg->serialized_data);

  if (msg->topic_name == "/rosout") {
    rcl_interfaces::msg::Log log;
    log_serializer_.deserialize_message(&extracted_serialized_msg, &log);
    const std::vector<std::string> roslog = {
        std::to_string(msg->time_stamp / 10E9),
        log_level_to_string_.at(log.level),
        log.name,
        "\"" + log.msg + "\"",
        log.function,
        log.file,
        std::to_string(log.line),
    };
    roslog_writter_.write_row(roslog);
  } else if (msg->topic_name == "/from_can_bus") {
    can_msgs::msg::Frame frame;
    frame_serializer_.deserialize_message(&extracted_serialized_msg, &frame);
    uint32_t id = nturt_can_config_logger_Receive(&can_rx_, frame.data.data(),
                                                  frame.id, frame.dlc);

    if (id == BMS_Cell_Stats_CANID) {
      battery_data_.update(&can_rx_.BMS_Cell_Stats);
    }
  } else if (msg->topic_name == "/fix") {
    nav_sat_fix_serializer_.deserialize_message(&extracted_serialized_msg,
                                                &gps_fix_);
  } else if (msg->topic_name == "/vel") {
    twist_stamped_serializer_.deserialize_message(&extracted_serialized_msg,
                                                  &gps_vel_);
  } else if (msg->topic_name == "/system_stats") {
    system_stats_serializer_.deserialize_message(&extracted_serialized_msg,
                                                 &system_stats_);
  }
}

std::vector<uint32_t> BagDecoder::onUpdateStatus() {
  // can rx timeout
  std::string timeout_frame_name = "\"";
  if (can_timeout_monior::can_rx_error & FRAME_FRONT_MASK) {
    timeout_frame_name += "Front Box, ";
  }
  if (can_timeout_monior::can_rx_error & FRAME_REAR_MASK) {
    timeout_frame_name += "Rear Box, ";
  }
  if (can_timeout_monior::can_rx_error & FRAME_BMS_MASK) {
    timeout_frame_name += "BMS, ";
  }
  if (can_timeout_monior::can_rx_error & FRAME_INVERTER_MASK) {
    timeout_frame_name += "Inverter, ";
  }
  if (can_timeout_monior::can_rx_error & FRAME_IMU_MASK) {
    timeout_frame_name += "IMU, ";
  }

  timeout_frame_name += "\"";

  status_logger_.write_raw(timeout_frame_name);

  std::vector<uint32_t> data;
  data.reserve(10);

  data.push_back(can_timeout_monior::can_rx_error);

  data.push_back(can_rx_.VCU_Status.VCU_Status);
  data.push_back(can_rx_.VCU_Status.VCU_Error_Code);

  data.push_back(can_rx_.REAR_SENSOR_Status.REAR_SENSOR_Status);
  data.push_back(can_rx_.REAR_SENSOR_Status.REAR_SENSOR_Error_Code);

  data.push_back(can_rx_.BMS_Status.BMS_Error_Code);

  data.push_back(can_rx_.INV_Internal_States.INV_VSM_State);
  data.push_back(can_rx_.INV_Internal_States.INV_Inverter_State);
  data.push_back((can_rx_.INV_Fault_Codes.INV_Post_Fault_Hi << 16) +
                 can_rx_.INV_Fault_Codes.INV_Post_Fault_Lo);
  data.push_back((can_rx_.INV_Fault_Codes.INV_Run_Fault_Hi << 16) +
                 can_rx_.INV_Fault_Codes.INV_Run_Fault_Lo);

  return data;
}

std::vector<double> BagDecoder::onUpdateSensor() {
  std::vector<double> data;
  data.reserve(32);

  data.push_back(can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Brake_phys);
  data.push_back(can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Accelerator_1_phys);
  data.push_back(can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Accelerator_2_phys);
  data.push_back(can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Brake_Micro);
  data.push_back(can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Accelerator_Micro);

  data.push_back(can_rx_.FRONT_SENSOR_1.FRONT_SENSOR_Steer_Angle);
  data.push_back(can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Brake_Pressure_phys);
  data.push_back(can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Rear_Brake_Pressure_phys);

  data.push_back(
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Left_Wheel_Speed_phys);
  data.push_back(
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Right_Wheel_Speed_phys);
  data.push_back(can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Left_Wheel_Speed_phys);
  data.push_back(can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Right_Wheel_Speed_phys);

  data.push_back(
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Left_Suspension_phys);
  data.push_back(
      can_rx_.FRONT_SENSOR_2.FRONT_SENSOR_Front_Right_Suspension_phys);
  data.push_back(can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Left_Suspension_phys);
  data.push_back(can_rx_.REAR_SENSOR_1.REAR_SENSOR_Rear_Right_Suspension_phys);

  data.push_back(
      can_rx_.FRONT_SENSOR_3.FRONT_SENSOR_Front_Left_Tire_Temperature_1_phys);
  data.push_back(
      can_rx_.FRONT_SENSOR_3.FRONT_SENSOR_Front_Left_Tire_Temperature_2_phys);
  data.push_back(
      can_rx_.FRONT_SENSOR_3.FRONT_SENSOR_Front_Left_Tire_Temperature_3_phys);
  data.push_back(
      can_rx_.FRONT_SENSOR_3.FRONT_SENSOR_Front_Left_Tire_Temperature_4_phys);

  data.push_back(
      can_rx_.FRONT_SENSOR_3.FRONT_SENSOR_Front_Right_Tire_Temperature_1_phys);
  data.push_back(
      can_rx_.FRONT_SENSOR_3.FRONT_SENSOR_Front_Right_Tire_Temperature_2_phys);
  data.push_back(
      can_rx_.FRONT_SENSOR_3.FRONT_SENSOR_Front_Right_Tire_Temperature_3_phys);
  data.push_back(
      can_rx_.FRONT_SENSOR_3.FRONT_SENSOR_Front_Right_Tire_Temperature_4_phys);

  data.push_back(
      can_rx_.REAR_SENSOR_2.REAR_SENSOR_Rear_Left_Tire_Temperature_1_phys);
  data.push_back(
      can_rx_.REAR_SENSOR_2.REAR_SENSOR_Rear_Left_Tire_Temperature_2_phys);
  data.push_back(
      can_rx_.REAR_SENSOR_2.REAR_SENSOR_Rear_Left_Tire_Temperature_3_phys);
  data.push_back(
      can_rx_.REAR_SENSOR_2.REAR_SENSOR_Rear_Left_Tire_Temperature_4_phys);

  data.push_back(
      can_rx_.REAR_SENSOR_2.REAR_SENSOR_Rear_Right_Tire_Temperature_1_phys);
  data.push_back(
      can_rx_.REAR_SENSOR_2.REAR_SENSOR_Rear_Right_Tire_Temperature_2_phys);
  data.push_back(
      can_rx_.REAR_SENSOR_2.REAR_SENSOR_Rear_Right_Tire_Temperature_3_phys);
  data.push_back(
      can_rx_.REAR_SENSOR_2.REAR_SENSOR_Rear_Right_Tire_Temperature_4_phys);

  return data;
}

std::vector<double> BagDecoder::onUpdateImu() {
  std::vector<double> data;
  data.reserve(9);

  IMU_Acceleration_t* accel = &can_rx_.IMU_Acceleration;
  data.push_back(accel->IMU_Acceleration_X_phys);
  data.push_back(accel->IMU_Acceleration_Y_phys);
  data.push_back(accel->IMU_Acceleration_Z_phys);

  IMU_Angular_Velocity_t* gyro = &can_rx_.IMU_Angular_Velocity;
  data.push_back(gyro->IMU_Angular_Velocity_X_phys);
  data.push_back(gyro->IMU_Angular_Velocity_Y_phys);
  data.push_back(gyro->IMU_Angular_Velocity_Z_phys);

  IMU_Quaternion_t* quat = &can_rx_.IMU_Quaternion;
  data.push_back(quat->IMU_Quaternion_W_phys);
  data.push_back(quat->IMU_Quaternion_X_phys);
  data.push_back(quat->IMU_Quaternion_Y_phys);
  data.push_back(quat->IMU_Quaternion_Z_phys);

  return data;
}

std::vector<std::string> BagDecoder::onUpdateGps() {
  std::vector<std::string> data;
  data.reserve(17);

  data.push_back(gps_fix_to_string_.at(gps_fix_.status.status));
  data.push_back(std::to_string(gps_fix_.status.service));

  data.push_back(std::to_string(gps_fix_.longitude));
  data.push_back(std::to_string(gps_fix_.latitude));
  data.push_back(std::to_string(gps_fix_.altitude));

  for (int i = 0; i < 9; i++) {
    data.push_back(std::to_string(gps_fix_.position_covariance[i]));
  }

  data.push_back(
      gps_covarience_type_to_string_.at(gps_fix_.position_covariance_type));

  data.push_back(std::to_string(gps_vel_.twist.linear.x));
  data.push_back(std::to_string(gps_vel_.twist.linear.y));

  return data;
}

std::vector<double> BagDecoder::onUpdateBattery() {
  std::vector<double> data;
  data.reserve(2 * NUM_BATTERY_SEGMENT * NUM_BATTERY_CELL_PER_SEGMENT + 1);

  double voltage = battery_data_.average_voltage();
  double current = can_rx_.INV_Current_Info.INV_DC_Bus_Current_phys;
  data.push_back(state_of_charge(voltage, current));

  for (int i = 1; i <= NUM_BATTERY_SEGMENT; i++) {
    for (int j = 1; j <= NUM_BATTERY_CELL_PER_SEGMENT; j++) {
      data.push_back(battery_data_.voltage[i][j]);
    }
  }

  for (int i = 1; i <= NUM_BATTERY_SEGMENT; i++) {
    for (int j = 1; j <= NUM_BATTERY_CELL_PER_SEGMENT; j++) {
      data.push_back(battery_data_.temperature[i][j]);
    }
  }

  return data;
}

std::vector<double> BagDecoder::onUpdateInverterData() {
  std::vector<double> data;
  data.reserve(8);

  data.push_back(can_rx_.INV_Fast_Info.INV_Fast_Torque_Command_phys);
  data.push_back(can_rx_.INV_Fast_Info.INV_Fast_Torque_Feedback_phys);
  data.push_back(can_rx_.INV_Fast_Info.INV_Fast_Motor_Speed);
  data.push_back(can_rx_.INV_Fast_Info.INV_Fast_DC_Bus_Voltage_phys);
  data.push_back(can_rx_.INV_Current_Info.INV_DC_Bus_Current_phys);
  data.push_back(can_rx_.INV_Temperature_Set_2.INV_Control_Board_Temp_phys);
  data.push_back(can_rx_.INV_Temperature_Set_3.INV_Hot_Spot_Temp_phys);
  data.push_back(can_rx_.INV_Temperature_Set_3.INV_Motor_Temp_phys);

  return data;
}

std::vector<double> BagDecoder::onUpdateSystemStats() {
  std::vector<double> data;
  data.reserve(5);

  data.push_back(system_stats_.cpu_temperature);
  data.push_back(system_stats_.cpu_usage);
  data.push_back(system_stats_.memory_usage);
  data.push_back(system_stats_.swap_usage);
  data.push_back(system_stats_.disk_usage);

  return data;
}

/// @author CHYang25 chris920325@gmail.com
// It should match with the order of headers
std::vector<std::string> BagDecoder::onUpdateUnifiedData() {
  std::vector<double> sensor_data = this->onUpdateSensor(), 
                      imu_data = this->onUpdateImu(),
                      battery_data = this->onUpdateBattery(),
                      inverter_data = this->onUpdateInverterData();
  std::vector<std::string> gps_data = this->onUpdateGps();

  const int col_size = sensor_data.size() + imu_data.size() + 
                    gps_data.size() + battery_data.size() + inverter_data.size();
      
  std::vector<std::string> data(col_size);
  std::vector<std::string>::iterator data_iter = data.begin();

  auto cast_double_lambda_function = [] (double x){
    return std::to_string(x);
  };

  std::transform(sensor_data.begin(), sensor_data.end(), data_iter, cast_double_lambda_function);
  data_iter += sensor_data.size();
  std::transform(imu_data.begin(), imu_data.end(), data_iter, cast_double_lambda_function);
  data_iter += imu_data.size();
  std::copy(gps_data.begin(), gps_data.end(), data_iter);
  data_iter += gps_data.size();
  std::transform(battery_data.begin(), battery_data.end(), data_iter, cast_double_lambda_function);
  data_iter += battery_data.size();
  std::transform(inverter_data.begin(), inverter_data.end(), data_iter, cast_double_lambda_function);

  return data;
}

uint32_t BagDecoder::get_tick() { return static_cast<uint32_t>(time_ * 1000); }

/* Exported function ---------------------------------------------------------*/
void create_directory(const std::string& path) {
  struct stat info;

  // directory doesn't exist
  if (stat(path.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
    std::filesystem::create_directory(path);
  } else {
    std::cerr << COLOR_RED "Error:" << HIGHLIGHT " Directory \"" << path
              << "\" already exist" << COLOR_REST << std::endl;
    exit(1);
  }
}
