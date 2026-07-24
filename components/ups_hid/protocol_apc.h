#pragma once

#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

// APC HID Protocol implementation for modern APC UPS devices
class ApcHidProtocol : public UpsProtocolBase {
public:
  // Public struct for use by report parsers
  struct HidReport {
    uint8_t report_id;
    std::vector<uint8_t> data;
    
    HidReport() : report_id(0) {}
  };
  
  explicit ApcHidProtocol(UpsHidComponent *parent);
  
  bool detect() override;
  bool initialize() override;
  bool read_data(UpsData &data) override;
  DeviceInfo::DetectedProtocol get_protocol_type() const override { return DeviceInfo::PROTOCOL_APC_HID; }
  std::string get_protocol_name() const override { return "APC HID Protocol"; }
  
  // Beeper control methods
  bool beeper_enable() override;
  bool beeper_disable() override;
  bool beeper_mute() override;
  bool beeper_test() override;
  
  // UPS and battery test methods
  bool start_battery_test_quick() override;
  bool start_battery_test_deep() override;
  bool stop_battery_test() override;
  bool start_ups_test() override;
  bool stop_ups_test() override;
  
  // Timer polling for real-time countdown
  bool read_timer_data(UpsData &data) override;
  
  // Delay configuration methods
  bool set_shutdown_delay(int seconds) override;
  bool set_start_delay(int seconds) override;
  bool set_reboot_delay(int seconds) override;

private:
  struct SlowMetricsCache {
    float battery_voltage_nominal{NAN};
    float battery_voltage{NAN};
    float battery_runtime_low{NAN};
    float battery_charge_low{NAN};
    float battery_charge_warning{NAN};
    std::string battery_type;
    std::string battery_mfr_date;
    float input_voltage_nominal{NAN};
    float input_transfer_low{NAN};
    float input_transfer_high{NAN};
    int16_t delay_shutdown{0};
    int16_t delay_start{0};
    int16_t delay_reboot{0};
    int timer_shutdown{-1};
    int timer_start{-1};
    int timer_reboot{-1};
    std::string test_result;
  };

  // Cache static USB descriptor information to avoid expensive repeated descriptor reads.
  bool static_info_cached_{false};
  uint32_t last_static_info_read_ms_{0};
  std::string cached_manufacturer_;
  std::string cached_model_;
  std::string cached_serial_number_;
  std::string cached_firmware_version_;
  std::string cached_firmware_aux_;
  static constexpr uint32_t STATIC_INFO_REFRESH_MS = 300000;  // 5 minutes

  // Slow metrics are expensive to poll over HID; refresh less often and reuse snapshot in-between.
  bool slow_metrics_cached_{false};
  uint8_t slow_poll_cycle_{0};
  static constexpr uint8_t SLOW_POLL_INTERVAL = 6;
  SlowMetricsCache slow_metrics_cache_;

  bool init_hid_communication();
  bool read_hid_report(uint8_t report_id, HidReport &report);
  bool write_hid_report(const HidReport &report);
  void cache_slow_metrics_(const UpsData &data);
  void apply_cached_slow_metrics_(UpsData &data) const;
  
  void parse_status_report(const HidReport &report, UpsData &data);
  void parse_battery_report(const HidReport &report, UpsData &data);
  void parse_voltage_report(const HidReport &report, UpsData &data);
  void parse_power_report(const HidReport &report, UpsData &data);
  
  // NUT-compatible parsers
  void parse_power_summary_report(const HidReport &report, UpsData &data);
  void parse_present_status_report(const HidReport &report, UpsData &data);
  void parse_apc_status_report(const HidReport &report, UpsData &data);
  void parse_input_voltage_report(const HidReport &report, UpsData &data);
  void parse_load_report(const HidReport &report, UpsData &data);
  void read_device_info();
  void parse_device_info_report(const HidReport &report);
  void log_raw_data(const uint8_t* buffer, size_t buffer_len);
  
  // Device information parsing
  void read_device_information(UpsData &data);
  void parse_serial_number_report(const HidReport &report, UpsData &data);
  void parse_firmware_version_report(const HidReport &report, UpsData &data);
  void parse_beeper_status_report(const HidReport &report, UpsData &data);
  void parse_input_sensitivity_report(const HidReport &report, UpsData &data);
  
  // Missing dynamic values from NUT analysis
  void read_missing_dynamic_values(UpsData &data);
  void parse_battery_voltage_nominal_report(const HidReport &report, UpsData &data);
  void parse_battery_voltage_actual_report(const HidReport &report, UpsData &data);
  void parse_input_voltage_nominal_report(const HidReport &report, UpsData &data);
  void parse_input_transfer_limits_report(const HidReport &report, UpsData &data);
  void parse_battery_runtime_low_report(const HidReport &report, UpsData &data);
  void parse_manufacture_date_report(const HidReport &report, UpsData &data, bool is_battery);
  void parse_ups_delay_shutdown_report(const HidReport &report, UpsData &data);
  void parse_ups_delay_reboot_report(const HidReport &report, UpsData &data);
  void parse_battery_charge_threshold_report(const HidReport &report, UpsData &data, bool is_low_threshold);
  void parse_battery_chemistry_report(const HidReport &report, UpsData &data);
  void parse_test_result_report(const HidReport &report, UpsData &data);
  std::string convert_apc_date(uint16_t date_value);
  
  // Device model detection and configuration
  void detect_nominal_power_rating(const std::string& model_name, UpsData &data);
  
  // Frequency reading methods
  void read_frequency_data(UpsData &data);
  float parse_frequency_from_report(const HidReport &report);
};

} // namespace ups_hid
} // namespace esphome