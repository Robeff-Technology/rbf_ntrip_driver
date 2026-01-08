#include "rbf_ntrip_driver/rbf_ntrip_driver.h"

namespace rbf_ntrip_driver
{

NtripDriver::NtripDriver(const rclcpp::NodeOptions & options) : Node("rbf_ntrip_driver", options)
{
  load_parameters();

  // Configure and start the NTRIP client
  ntrip_client_ptr_ = std::make_shared<libntrip::NtripClient>(
    config_.ntrip.host, config_.ntrip.port, config_.ntrip.username, config_.ntrip.password,
    config_.ntrip.mountpoint);
  ntrip_client_ptr_->set_report_interval(static_cast<int>(config_.ntrip.gpgga_interval_sec));
  diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
  diagnostic_updater_->setHardwareID("NTRIP DRIVER");
  diagnostic_updater_->add("NTRIP Client Status", this, &NtripDriver::diagnostic_callback);

  ntrip_client_ptr_->OnReceived(std::bind(
    &NtripDriver::ntrip_client_callback, this, std::placeholders::_1, std::placeholders::_2));

  // Configure and open the serial port publisher if enabled
  if (config_.serial_port.publish_port_rtcm) {
    try {
      serial_port_ptr_ = std::make_shared<SerialPort>(config_.serial_port.port.c_str());
      serial_port_ptr_->open();
      serial_port_ptr_->configure(config_.serial_port.baudrate, 8, 'N', 1);
    } catch (const SerialPortException & e) {
      RCLCPP_ERROR(get_logger(), e.what());
      rclcpp::shutdown();
    }
  }

  if (config_.ntrip.use_gpgga_for_ntrip) {
    RCLCPP_INFO(this->get_logger(), "Subscribing to GPGGA sentences for NTRIP client...");
    sub_gpgga_ = this->create_subscription<nmea_msgs::msg::Sentence>(
      config_.ntrip.gpgga_topic_name, rclcpp::SensorDataQoS(),
      [this](const nmea_msgs::msg::Sentence::SharedPtr msg) {
        if (!is_valid_gpgga_sentence(msg->sentence)) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000, "Invalid GPGGA received = %s, skipping",
            msg->sentence.c_str());
          return;
        }
        ntrip_client_ptr_->set_gga_buffer(msg->sentence);
      });
  }

  // Create the RTCM publisher if enabled
  if (config_.rtcm_publisher.publish_rtcm) {
    pub_rtcm_ = this->create_publisher<mavros_msgs::msg::RTCM>(
      config_.rtcm_publisher.topic_name, rclcpp::SensorDataQoS());
  }
  // Subscribe to NAV-SAT-FIX if it's used initially, Otherwise, try to establish the NTRIP
  // connection
  if (config_.ntrip.use_nav_sat_fix_init) {
    RCLCPP_INFO(this->get_logger(), "Waiting for NavSatFix message to initialize NTRIP client...");
    sub_nav_sat_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      config_.ntrip.nav_sat_fix_topic_name, rclcpp::SensorDataQoS(),
      std::bind(&NtripDriver::nav_sat_fix_callback, this, std::placeholders::_1));
  } else {
    try_to_ntrip_connect();
  }
}

static uint8_t hex_to_nibble(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;
}

static bool verify_nmea_checksum(const std::string & sentence)
{
  const auto asterisk_pos = sentence.find('*');
  if (
    sentence.empty() || sentence[0] != '$' || asterisk_pos == std::string::npos ||
    asterisk_pos + 2 >= sentence.size()) {
    return false;
  }

  uint8_t checksum = 0;
  for (size_t i = 1; i < asterisk_pos; ++i) {
    checksum ^= static_cast<uint8_t>(sentence[i]);
  }

  uint8_t expected =
    (hex_to_nibble(sentence[asterisk_pos + 1]) << 4) | hex_to_nibble(sentence[asterisk_pos + 2]);

  return checksum == expected;
}

bool NtripDriver::is_valid_gpgga_sentence(const std::string & sentence)
{
  if (!(sentence.rfind("$GPGGA", 0) == 0 || sentence.rfind("$GNGGA", 0) == 0)) {
    return false;
  }

  if (!verify_nmea_checksum(sentence)) {
    return false;
  }

  std::string clean = sentence;
  if (clean.size() >= 2 && clean.substr(clean.size() - 2) == "\r\n") {
    clean.resize(clean.size() - 2);
  }

  const auto asterisk_pos = clean.find('*');
  if (asterisk_pos == std::string::npos) {
    return false;
  }

  clean = clean.substr(0, asterisk_pos);

  std::vector<std::string> fields;
  fields.reserve(15);

  size_t start = 0;
  size_t end;
  while ((end = clean.find(',', start)) != std::string::npos) {
    fields.emplace_back(clean.substr(start, end - start));
    start = end + 1;
  }
  fields.emplace_back(clean.substr(start));

  if (fields.size() < 10) {
    return false;
  }

  int fix_quality = std::stoi(fields[6]);
  if (fix_quality <= 0) {
    return false;
  }

  // 7️⃣ Lat / Lon boş olmamalı
  if (fields[2].empty() || fields[4].empty()) {
    return false;
  }

  // 8️⃣ Sayısal parse + mantık kontrolü (DDMM.MMMM)
  double lat = std::stod(fields[2]);
  double lon = std::stod(fields[4]);

  if (lat <= 0.0 || lat > 9000.0) return false;
  if (lon <= 0.0 || lon > 18000.0) return false;

  return true;
}

void NtripDriver::load_parameters()
{
  config_.ntrip.host = declare_parameter("ntrip.host", "ntrip.example.com");
  config_.ntrip.port = declare_parameter("ntrip.port", 2101);
  config_.ntrip.mountpoint = declare_parameter("ntrip.mount_point", "VRSRTCM31");
  config_.ntrip.username = declare_parameter("ntrip.user_name", "username");
  config_.ntrip.password = declare_parameter("ntrip.password", "password");
  config_.ntrip.use_nav_sat_fix_init = declare_parameter("ntrip.use_nav_sat_fix_init", false);
  config_.ntrip.nav_sat_fix_topic_name = declare_parameter("ntrip.nav_sat_fix_topic_name", "/fix");
  config_.ntrip.init_lat_position = declare_parameter("ntrip.init_ntrip_location_lat", 0.0);
  config_.ntrip.init_lon_position = declare_parameter("ntrip.init_ntrip_location_lon", 0.0);
  config_.ntrip.use_gpgga_for_ntrip = declare_parameter("ntrip.use_gpgga_for_ntrip", false);
  config_.ntrip.gpgga_topic_name = declare_parameter("ntrip.gpgga_topic_name", "/gpgga");
  config_.ntrip.gpgga_interval_sec = declare_parameter("ntrip.gpgga_interval_sec", 5.0);
  config_.ntrip.shutdown_if_not_connected = declare_parameter("shutdown_if_not_connected", false);
  config_.ntrip.shutdown_length_sec = declare_parameter("shutdown_length_sec", 10.0);
    

  config_.serial_port.port = declare_parameter("serial_port.name", "/dev/ttyUSB0");
  config_.serial_port.baudrate = declare_parameter("serial_port.baud_rate", 9600);
  config_.serial_port.publish_port_rtcm = declare_parameter("serial_port.publish_port_rtcm", false);

  config_.rtcm_publisher.topic_name = declare_parameter("rtcm_publisher.rtcm_topic", "rtcm");
  config_.rtcm_publisher.publish_rtcm = declare_parameter("rtcm_publisher.publish_rtcm", true);
  config_.rtcm_publisher.frame_id = declare_parameter("rtcm_publisher.frame_id", "rtcm");

  RCLCPP_INFO(this->get_logger(), "---------NTRIP CONFIGURATION--------");
  RCLCPP_INFO(this->get_logger(), "host: %s", config_.ntrip.host.c_str());
  RCLCPP_INFO(this->get_logger(), "port: %d", config_.ntrip.port);
  RCLCPP_INFO(this->get_logger(), "mountpoint: %s", config_.ntrip.mountpoint.c_str());
  RCLCPP_INFO(this->get_logger(), "username: %s", config_.ntrip.username.c_str());
  RCLCPP_INFO(this->get_logger(), "password: %s", config_.ntrip.password.c_str());
  RCLCPP_INFO(this->get_logger(), "use_nav_sat_fix_init: %d", config_.ntrip.use_nav_sat_fix_init);
  RCLCPP_INFO(
    this->get_logger(), "nav_sat_fix_topic_name: %s", config_.ntrip.nav_sat_fix_topic_name.c_str());
  RCLCPP_INFO(this->get_logger(), "init_lat_position: %f", config_.ntrip.init_lat_position);
  RCLCPP_INFO(this->get_logger(), "init_lon_position: %f", config_.ntrip.init_lon_position);
  RCLCPP_INFO(this->get_logger(), "use_gpgga_for_ntrip: %d", config_.ntrip.use_gpgga_for_ntrip);
  RCLCPP_INFO(this->get_logger(), "gpgga_topic_name: %s", config_.ntrip.gpgga_topic_name.c_str());
  RCLCPP_INFO(this->get_logger(), "gpgga_interval_sec: %f", config_.ntrip.gpgga_interval_sec);
  RCLCPP_INFO(this->get_logger(), "shutdown_if_not_connected: %d", config_.ntrip.shutdown_if_not_connected);
  RCLCPP_INFO(this->get_logger(), "shutdown_length_sec: %d", config_.ntrip.shutdown_length_sec);

  RCLCPP_INFO(this->get_logger(), "---------SERIAL PORT CONFIGURATION--------");
  RCLCPP_INFO(this->get_logger(), "port: %s", config_.serial_port.port.c_str());
  RCLCPP_INFO(this->get_logger(), "baudrate: %d", config_.serial_port.baudrate);
  RCLCPP_INFO(this->get_logger(), "publish_port_rtcm: %d", config_.serial_port.publish_port_rtcm);
  RCLCPP_INFO(this->get_logger(), "---------RTCM PUBLISHER CONFIGURATION--------");
  RCLCPP_INFO(this->get_logger(), "topic_name: %s", config_.rtcm_publisher.topic_name.c_str());
  RCLCPP_INFO(this->get_logger(), "publish_rtcm: %d", config_.rtcm_publisher.publish_rtcm);
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", config_.rtcm_publisher.frame_id.c_str());

  RCLCPP_INFO(this->get_logger(), "----------------------------------");
}

// Start the NTRIP client
bool NtripDriver::run_ntrip()
{
  if (!config_.ntrip.use_nav_sat_fix_init) {
    ntrip_client_ptr_->set_location(
      config_.ntrip.init_lat_position, config_.ntrip.init_lon_position);
  }
  return ntrip_client_ptr_->Run();
}

// Try to establish the NTRIP connection
void NtripDriver::try_to_ntrip_connect()
{
  int try_count = 0;

  // if shutdown_if_not_connected argument is given false, the while
  // loop will last forever. if argument is determined as true,
  // shutdown_arg will be specified based on whether try_count is less
  // then shutdown_length_sec or not.

  bool shutdown_arg = false;
  if (!config_.ntrip.shutdown_if_not_connected)
  {
    shutdown_arg = try_count < config_.ntrip.shutdown_length_sec;
    // is it possible to change parameters in while loop? if yes,
    // create const int max_attempts to compare with try_count
  }
  // Try for a certain number of attempts or until successful
  while (!shutdown_arg && rclcpp::ok()) {
    if (run_ntrip()) {
      ntrip_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Connected to the NTRIP Client");
      return;
    }
    // Retry after a delay if failed
    RCLCPP_INFO(this->get_logger(), "Not Connected to the NTRIP Client");
    try_count++;
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  // Shutdown if maximum attempts reached or failed to initialize NTRIP client
  if(config_.ntrip.shutdown_if_not_connected)
  {
    rclcpp::shutdown();
  }
}

// Timer callback function
void NtripDriver::diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!ntrip_client_ptr_->service_is_running()) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Try to connect to NTRIP server failed");
    ntrip_client_ptr_->Stop();
    if (config_.ntrip.use_nav_sat_fix_init) {
      if (nav_sat_fix_received_ == false) {
        stat.summary(
          diagnostic_msgs::msg::DiagnosticStatus::WARN, "NavSatFix message not received");
      }
      if (sub_nav_sat_fix_ == nullptr) {
        nav_sat_fix_received_ = false;
        sub_nav_sat_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
          config_.ntrip.nav_sat_fix_topic_name, 10,
          std::bind(&NtripDriver::nav_sat_fix_callback, this, std::placeholders::_1));
      }
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Trying to connect to NTRIP");
      try_to_ntrip_connect();
    }
  } else {
    if ((this->now() - ntrip_time_).seconds() > 3) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No RTCM data received");
      stat.add("Last RTCM data received(second)", (this->now() - ntrip_time_).seconds());
      ntrip_client_ptr_->Stop();
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "NTRIP Client is running");
    }
  }
}

// Callback for data received from the NTRIP client
void NtripDriver::ntrip_client_callback(char const * _buffer, int _size)
{
  ntrip_time_ = this->now();
  auto rtcm_msg = mavros_msgs::msg::RTCM();
  rtcm_msg.header.stamp = this->now();
  rtcm_msg.header.frame_id = config_.rtcm_publisher.frame_id;
  rtcm_msg.data = std::vector<uint8_t>(_buffer, _buffer + _size);
  if (config_.rtcm_publisher.publish_rtcm) {
    pub_rtcm_->publish(rtcm_msg);
  }
  if (config_.serial_port.publish_port_rtcm) {
    serial_port_ptr_->write(_buffer, _size);
  }
}

// Callback for NAV-SAT-FIX data
void NtripDriver::nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  ntrip_client_ptr_->set_location(msg->latitude, msg->longitude);
  nav_sat_fix_received_ = true;
  try_to_ntrip_connect();
  sub_nav_sat_fix_.reset();
}

};  // namespace rbf_ntrip_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rbf_ntrip_driver::NtripDriver)
