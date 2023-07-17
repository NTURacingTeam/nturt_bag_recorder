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
#include "nturt_can_config_logger-binutil.h"

#define COLOR_REST "\033[0m"
#define HIGHLIGHT "\033[0;1m"
#define COLOR_RED "\033[1;31m"

/* Class Static vataible -----------------------------------------------------*/
const std::vector<std::string> BagDecoder::status_header_ = {
    "vcu_status",
    "vcu_error_code",
    "rear_sensor_status",
    "rear_sensor_error_code",
    "bms_error_code",
    "inverter_state",
    "inverter_vsm_state"
    "inverter_post_fault",
    "inverter_run_fault",
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

  header.push_back("rear_right_suspension_travel");
  header.push_back("rear_right_suspension_travel");
  header.push_back("rear_right_suspension_travel");
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
    "longitude", "latitude", "altitude", "velocity_x", "velocity_y",
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

BagDecoder::BagDecoder(BagDecoderArg arg)
    : arg_(arg),
      status_logger_(arg.output_directory + "/status", status_header_,
                     std::bind(&BagDecoder::onUpdateStatus, this), 0.1),
      sensor_logger_(arg.output_directory + "/sensor", sensor_header_,
                     std::bind(&BagDecoder::onUpdateSensor, this), 0.1),
      imu_logger_(arg.output_directory + "/imu", imu_header_,
                  std::bind(&BagDecoder::onUpdateImu, this), 0.01),
      gps_logger_(arg.output_directory + "/gps", gps_header_,
                  std::bind(&BagDecoder::onUpdateGps, this), 1),
      battery_logger_(arg.output_directory + "/battery", battery_header_,
                      std::bind(&BagDecoder::onUpdateBattery, this), 1),
      inverter_data_logger_(
          arg.output_directory + "/inverter_data", inverter_data_header_,
          std::bind(&BagDecoder::onUpdateInverterData, this), 0.01),
      system_stats_logger_(
          arg.output_directory + "/system_stats", system_stats_header_,
          std::bind(&BagDecoder::onUpdateSystemStats, this), 0.1) {
  memset(&can_rx_, 0, sizeof(can_rx_));
}

void BagDecoder::run() {
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

  double time = static_cast<double>(serialized_message->time_stamp) / 1.0E9;

  status_logger_.init_time(time);
  sensor_logger_.init_time(time);
  imu_logger_.init_time(time);
  gps_logger_.init_time(time);
  battery_logger_.init_time(time);
  inverter_data_logger_.init_time(time);
  system_stats_logger_.init_time(time);

  while (reader.has_next()) {
    serialized_message = reader.read_next();
    time = static_cast<double>(serialized_message->time_stamp) / 1.0E9;

    // write local buffers to csv file before updating the data
    status_logger_.update(time);
    sensor_logger_.update(time);
    imu_logger_.update(time);
    gps_logger_.update(time);
    battery_logger_.update(time);
    inverter_data_logger_.update(time);
    system_stats_logger_.update(time);

    // update local buffers from bag message
    update_data(serialized_message);
  }
}

void BagDecoder::update_data(
    rosbag2_storage::SerializedBagMessageSharedPtr msg) {
  rclcpp::SerializedMessage extracted_serialized_msg(*msg->serialized_data);

  if (msg->topic_name == "/rosout") {
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
  std::vector<uint32_t> data;
  data.reserve(9);

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

std::vector<double> BagDecoder::onUpdateGps() {
  std::vector<double> data;
  data.reserve(5);

  data.push_back(gps_fix_.longitude);
  data.push_back(gps_fix_.latitude);
  data.push_back(gps_fix_.altitude);

  data.push_back(gps_vel_.twist.linear.x);
  data.push_back(gps_vel_.twist.linear.y);

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
