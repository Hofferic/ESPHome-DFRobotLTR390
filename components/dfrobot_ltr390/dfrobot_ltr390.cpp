#include "dfrobot_ltr390.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dfrobot_ltr390 {

static const char *const TAG = "dfrobot_ltr390";

// DFRobot specific register addresses (different from standard LTR390)
static const uint8_t DFROBOT_MODE_REGISTER = 0x00;
static const uint8_t DFROBOT_MEAS_RATE_REGISTER = 0x01;
static const uint8_t DFROBOT_GAIN_REGISTER = 0x02;
static const uint8_t DFROBOT_ALS_DATA_REGISTER = 0x03;  // 4 bytes
static const uint8_t DFROBOT_UVS_DATA_REGISTER = 0x07;  // 4 bytes

// DFRobot specific mode values
static const uint8_t DFROBOT_MODE_ALS = 0x01;
static const uint8_t DFROBOT_MODE_UVS = 0x02;

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
  
  this->reading_state_ = ReadingState::IDLE;
  this->start_als_reading_();
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
  ESP_LOGCONFIG(TAG, "  Measurement rate: %dms", this->get_measurement_rate_ms_());
  
  LOG_SENSOR("  ", "Ambient Light", this->ambient_light_sensor_);
  LOG_SENSOR("  ", "UV Index", this->uv_index_sensor_);
}

bool DFRobotLTR390Component::initialize_sensor_() {
  // Test I2C communication first
  uint8_t test_data = 0;
  if (!this->read_byte(DFROBOT_MODE_REGISTER, &test_data)) {
    ESP_LOGE(TAG, "Failed to communicate with sensor");
    return false;
  }
  
  ESP_LOGD(TAG, "Initial mode register value: 0x%02X", test_data);
  
  // Configure measurement rate and resolution
  // Format: [resolution_bits << 4] | measurement_rate_bits
  uint8_t meas_rate_config = (this->resolution_ << 4) | this->measurement_rate_;
  if (!this->write_register_(DFROBOT_MEAS_RATE_REGISTER, meas_rate_config)) {
    ESP_LOGE(TAG, "Failed to set measurement rate and resolution");
    return false;
  }
  
  // Configure gain
  if (!this->write_register_(DFROBOT_GAIN_REGISTER, this->gain_)) {
    ESP_LOGE(TAG, "Failed to set gain");
    return false;
  }
  
  ESP_LOGI(TAG, "Sensor initialized successfully");
  return true;
}

void DFRobotLTR390Component::start_als_reading_() {
  if (this->ambient_light_sensor_ == nullptr) {
    // Skip ALS, go to UV
    this->start_uv_reading_();
    return;
  }
  
  ESP_LOGD(TAG, "Starting ALS reading");
  this->reading_state_ = ReadingState::READING_ALS;
  
  // Set ALS mode
  if (!this->write_register_(DFROBOT_MODE_REGISTER, DFROBOT_MODE_ALS)) {
    ESP_LOGW(TAG, "Failed to set ALS mode");
    this->start_uv_reading_();
    return;
  }
  
  // Wait for measurement to complete
  uint32_t wait_time = this->get_measurement_rate_ms_() + 100;  // Add buffer
  this->set_timeout("als_read", wait_time, [this]() {
    this->read_als_data_();
  });
}

void DFRobotLTR390Component::read_als_data_() {
  // Read 4 bytes of ALS data
  uint8_t buffer[4];
  if (!this->read_bytes(DFROBOT_ALS_DATA_REGISTER, buffer, 4)) {
    ESP_LOGW(TAG, "Failed to read ALS data");
  } else {
    // Combine bytes into 32-bit value (little-endian)
    uint32_t als_data = (uint32_t(buffer[3]) << 24) | 
                       (uint32_t(buffer[2]) << 16) | 
                       (uint32_t(buffer[1]) << 8) | 
                       buffer[0];
    
    float lux = this->calculate_lux_(als_data);
    this->ambient_light_sensor_->publish_state(lux);
    ESP_LOGD(TAG, "ALS raw: %u, Lux: %.2f", als_data, lux);
  }
  
  // Wait before switching to UV mode
  this->set_timeout("switch_to_uv", 50, [this]() {
    this->start_uv_reading_();
  });
}

void DFRobotLTR390Component::start_uv_reading_() {
  if (this->uv_index_sensor_ == nullptr) {
    // No UV sensor, we're done
    this->reading_state_ = ReadingState::IDLE;
    return;
  }
  
  ESP_LOGD(TAG, "Starting UV reading");
  this->reading_state_ = ReadingState::READING_UV;
  
  // Set UV mode
  if (!this->write_register_(DFROBOT_MODE_REGISTER, DFROBOT_MODE_UVS)) {
    ESP_LOGW(TAG, "Failed to set UV mode");
    this->reading_state_ = ReadingState::IDLE;
    return;
  }
  
  // Wait for measurement to complete
  uint32_t wait_time = this->get_measurement_rate_ms_() + 100;  // Add buffer
  this->set_timeout("uv_read", wait_time, [this]() {
    this->read_uv_data_();
  });
}

void DFRobotLTR390Component::read_uv_data_() {
  // Read 4 bytes of UV data
  uint8_t buffer[4];
  if (!this->read_bytes(DFROBOT_UVS_DATA_REGISTER, buffer, 4)) {
    ESP_LOGW(TAG, "Failed to read UV data");
  } else {
    // Combine bytes into 32-bit value (little-endian)
    uint32_t uvs_data = (uint32_t(buffer[3]) << 24) | 
                       (uint32_t(buffer[2]) << 16) | 
                       (uint32_t(buffer[1]) << 8) | 
                       buffer[0];
    
    float uv_index = this->calculate_uv_index_(uvs_data);
    this->uv_index_sensor_->publish_state(uv_index);
    ESP_LOGD(TAG, "UV raw: %u, UV Index: %.2f", uvs_data, uv_index);
  }
  
  // Reading complete
  this->reading_state_ = ReadingState::IDLE;
}

bool DFRobotLTR390Component::write_register_(uint8_t reg, uint8_t value) {
  bool success = this->write_byte(reg, value);
  if (!success) {
    ESP_LOGW(TAG, "Failed to write register 0x%02X with value 0x%02X", reg, value);
  } else {
    ESP_LOGD(TAG, "Wrote register 0x%02X = 0x%02X", reg, value);
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

float DFRobotLTR390Component::calculate_lux_(uint32_t als_data) {
  if (als_data == 0) return 0.0;
  
  // DFRobot specific calculation - the built-in MCU likely does some processing
  // This may need adjustment based on testing with known light sources
  float gain_factor = this->get_gain_factor_();
  
  // Simplified calculation - may need calibration
  float lux = als_data / (gain_factor * 100.0);
  return lux;
}

float DFRobotLTR390Component::calculate_uv_index_(uint32_t uvs_data) {
  if (uvs_data == 0) return 0.0;
  
  // DFRobot specific calculation
  float gain_factor = this->get_gain_factor_();
  
  // Simplified calculation - may need calibration based on datasheet
  float uv_index = uvs_data / (gain_factor * 10000.0);
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

uint32_t DFRobotLTR390Component::get_measurement_rate_ms_() {
  switch (this->measurement_rate_) {
    case 0x00: return 25;
    case 0x01: return 50;
    case 0x02: return 100;
    case 0x03: return 200;
    case 0x04: return 500;
    case 0x05: return 1000;
    case 0x06: return 2000;
    default: return 100;
  }
}

// Remove the old methods that don't apply to DFRobot version
void DFRobotLTR390Component::set_mode_(bool als_mode) {
  // Not used in DFRobot version - mode is set directly per measurement
}

optional<uint32_t> DFRobotLTR390Component::read_register_24_(uint8_t reg) {
  // Not used in DFRobot version - uses 32-bit reads
  return {};
}

bool DFRobotLTR390Component::is_data_ready_() {
  // Not used in DFRobot version - relies on timing
  return true;
}

float DFRobotLTR390Component::get_integration_time_() {
  // Convert measurement rate to seconds
  return this->get_measurement_rate_ms_() / 1000.0;
}

}  // namespace dfrobot_ltr390
}  // namespace esphome
