#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace dfrobot_ltr390 {

class DFRobotLTR390Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_ambient_light_sensor(sensor::Sensor *ambient_light_sensor) {
    ambient_light_sensor_ = ambient_light_sensor;
  }
  void set_uv_index_sensor(sensor::Sensor *uv_index_sensor) {
    uv_index_sensor_ = uv_index_sensor;
  }
  void set_gain(uint8_t gain) { gain_ = gain; }
  void set_resolution(uint8_t resolution) { resolution_ = resolution; }
  void set_measurement_rate(uint8_t measurement_rate) { measurement_rate_ = measurement_rate; }

 protected:
  sensor::Sensor *ambient_light_sensor_{nullptr};
  sensor::Sensor *uv_index_sensor_{nullptr};
  
  uint8_t gain_{0x01};  // Default: 3x gain
  uint8_t resolution_{0x02};  // Default: 18-bit
  uint8_t measurement_rate_{0x02};  // Default: 100ms

  enum class ReadingState {
    IDLE,
    READING_ALS,
    READING_UV
  };
  ReadingState reading_state_{ReadingState::IDLE};

  bool initialize_sensor_();
  void start_als_reading_();
  void read_als_data_();
  void start_uv_reading_();
  void read_uv_data_();
  bool write_register_(uint8_t reg, uint8_t value);
  uint8_t read_register_(uint8_t reg);
  float calculate_lux_(uint32_t als_data);
  float calculate_uv_index_(uint32_t uvs_data);
  uint8_t get_gain_factor_();
  uint32_t get_measurement_rate_ms_();
  
  // Legacy methods for compatibility - not used in DFRobot version
  void set_mode_(bool als_mode);
  optional<uint32_t> read_register_24_(uint8_t reg);
  bool is_data_ready_();
  float get_integration_time_();
};

}  // namespace dfrobot_ltr390
}  // namespace esphome
