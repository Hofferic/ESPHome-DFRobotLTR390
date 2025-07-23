#include "dfrobot_ltr390.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dfrobot_ltr390 {

static const char *const TAG = "dfrobot_ltr390";

void DFRobotLTR390Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DFRobot LTR390...");
  
  if (!this->initialize_sensor_()) {
    ESP_LOGE(TAG, "Failed to initialize sensor");
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "DFRobot LTR390 setup complete");
}

void DFRobotLTR390Component::update() {
  ESP_LOGD(TAG, "Starting sensor update");
  
  if (!this->read_sensor_data_()) {
    ESP_LOGW(TAG, "Failed to read sensor data");
    this->status_set_warning();
    return;
  }
  
  this->status_clear_warning();
}

void DFRobotLTR390Component::dump_config() {
  ESP_LOGCONFIG(TAG, "DFRobot LTR390:");
  LOG_I2C_DEVICE(this);
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with DFRobot LTR390 failed!");
    return;
  }
  
  ESP_LOGCONFIG(TAG, "  Gain: %dx", this->get_gain_factor_());
  ESP_LOGCONFIG(TAG, "  Resolution: %d-bit", 13 + this->resolution_);
  ESP_LOGCONFIG(TAG, "  Integration time: %.1fms", this->get_integration_time_() * 1000);
  
  LOG_SENSOR("  ", "Ambient Light", this->ambient_light_sensor_);
  LOG_SENSOR("  ", "UV Index", this->uv_index_sensor_);
}

bool DFRobotLTR390Component::initialize_sensor_() {
  // Reset the sensor
  if (!this->write_register_(LTR390_MAIN_CTRL, LTR390_CTRL_RESET)) {
    ESP_LOGE(TAG, "Failed to reset sensor");
    return false;
  }
  
  delay(100);  // Wait for reset to complete
  
  // Check part ID
  uint8_t part_id = this->read_register_(LTR390_PART_ID);
  if (part_id != 0xB2) {
    ESP_LOGE(TAG, "Invalid part ID: 0x%02X (expected 0xB2)", part_id);
    return false;
  }
  
  // Configure measurement rate and resolution
  uint8_t meas_rate = (this->resolution_ << 4) | this->measurement_rate_;
  if (!this->write_register_(LTR390_MEAS_RATE, meas_rate)) {
    ESP_LOGE(TAG, "Failed to set measurement rate");
    return false;
  }
  
  // Configure gain
  if (!this->write_register_(LTR390_GAIN, this->gain_)) {
    ESP_LOGE(TAG, "Failed to set gain");
    return false;
  }
  
  ESP_LOGI(TAG, "Sensor initialized successfully");
  return true;
}

bool DFRobotLTR390Component::read_sensor_data_() {
  // Read ALS data if sensor is configured
  if (this->ambient_light_sensor_ != nullptr) {
    ESP_LOGD(TAG, "Reading ambient light sensor");
    
    this->set_mode_(true);  // ALS mode
    delay(150);  // Wait for mode switch and measurement
    
    // Wait for data to be ready (with timeout)
    uint32_t start_time = millis();
    while (!this->is_data_ready_() && (millis() - start_time) < 500) {
      delay(10);
    }
    
    if (this->is_data_ready_()) {
      auto als_data = this->read_register_24_(LTR390_ALSDATA);
      if (als_data.has_value()) {
        float lux = this->calculate_lux_(als_data.value());
        this->ambient_light_sensor_->publish_state(lux);
        ESP_LOGD(TAG, "ALS: %d, Lux: %.2f", als_data.value(), lux);
      } else {
        ESP_LOGW(TAG, "Failed to read ALS data register");
        return false;
      }
    } else {
      ESP_LOGW(TAG, "ALS data not ready after timeout");
      return false;
    }
  }
  
  // Read UV data if sensor is configured
  if (this->uv_index_sensor_ != nullptr) {
    ESP_LOGD(TAG, "Reading UV index sensor");
    
    this->set_mode_(false);  // UV mode
    delay(150);  // Wait for mode switch and measurement
    
    // Wait for data to be ready (with timeout)
    uint32_t start_time = millis();
    while (!this->is_data_ready_() && (millis() - start_time) < 500) {
      delay(10);
    }
    
    if (this->is_data_ready_()) {
      auto uvs_data = this->read_register_24_(LTR390_UVSDATA);
      if (uvs_data.has_value()) {
        float uv_index = this->calculate_uv_index_(uvs_data.value());
        this->uv_index_sensor_->publish_state(uv_index);
        ESP_LOGD(TAG, "UVS: %d, UV Index: %.2f", uvs_data.value(), uv_index);
      } else {
        ESP_LOGW(TAG, "Failed to read UVS data register");
        return false;
      }
    } else {
      ESP_LOGW(TAG, "UVS data not ready after timeout");
      return false;
    }
  }
  
  return true;
}

void DFRobotLTR390Component::set_mode_(bool als_mode) {
  uint8_t ctrl_value = als_mode ? LTR390_CTRL_MODE_ALS : LTR390_CTRL_MODE_UVS;
  this->write_register_(LTR390_MAIN_CTRL, ctrl_value);
  this->is_als_mode_ = als_mode;
  ESP_LOGD(TAG, "Set mode to %s", als_mode ? "ALS" : "UVS");
}

bool DFRobotLTR390Component::write_register_(uint8_t reg, uint8_t value) {
  bool success = this->write_byte(reg, value);
  if (!success) {
    ESP_LOGW(TAG, "Failed to write register 0x%02X with value 0x%02X", reg, value);
  }
  return success;
}

uint8_t DFRobotLTR390Component::read_register_(uint8_t reg) {
  uint8_t value = 0;
  if (!this->read_byte(reg, &value)) {
    ESP_LOGW(TAG, "Failed to read register 0x%02X", reg);
  }
  return value;
}

optional<uint32_t> DFRobotLTR390Component::read_register_24_(uint8_t reg) {
  uint8_t buffer[3];
  if (!this->read_bytes(reg, buffer, 3)) {
    ESP_LOGW(TAG, "Failed to read 24-bit register 0x%02X", reg);
    return {};
  }
  uint32_t value = (uint32_t(buffer[2]) << 16) | (uint32_t(buffer[1]) << 8) | buffer[0];
  ESP_LOGD(TAG, "Read 24-bit register 0x%02X: 0x%06X", reg, value);
  return value;
}

bool DFRobotLTR390Component::is_data_ready_() {
  uint8_t status = this->read_register_(LTR390_MAIN_STATUS);
  bool ready = (status & LTR390_STATUS_DATA_READY) != 0;
  ESP_LOGVV(TAG, "Status register: 0x%02X, data ready: %s", status, ready ? "yes" : "no");
  return ready;
}

float DFRobotLTR390Component::calculate_lux_(uint32_t als_data) {
  float gain_factor = this->get_gain_factor_();
  float integration_time = this->get_integration_time_();
  
  // Calculate lux using the formula from the datasheet
  float lux = (0.6 * als_data) / (gain_factor * integration_time);
  return lux;
}

float DFRobotLTR390Component::calculate_uv_index_(uint32_t uvs_data) {
  float gain_factor = this->get_gain_factor_();
  float integration_time = this->get_integration_time_();
  
  // Calculate UV index using the formula from the datasheet
  float uv_index = (uvs_data * 0.25) / (gain_factor * integration_time * 262144);
  return uv_index;
}

uint8_t DFRobotLTR390Component::get_gain_factor_() {
  switch (this->gain_) {
    case 0x00: return 1;
    case 0x01: return 3;
    case 0x02: return 6;
    case 0x03: return 9;
    case 0x04: return 18;
    default: return 3;
  }
}

float DFRobotLTR390Component::get_integration_time_() {
  switch (this->measurement_rate_) {
    case 0x00: return 0.025;   // 25ms
    case 0x01: return 0.050;   // 50ms
    case 0x02: return 0.100;   // 100ms
    case 0x03: return 0.200;   // 200ms
    case 0x04: return 0.500;   // 500ms
    case 0x05: return 1.000;   // 1000ms
    case 0x06: return 2.000;   // 2000ms
    default: return 0.100;     // 100ms
  }
}

}  // namespace dfrobot_ltr390
}  // namespace esphome
