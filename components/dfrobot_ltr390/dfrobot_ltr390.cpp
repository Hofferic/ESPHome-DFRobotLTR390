#include "dfrobot_ltr390.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dfrobot_ltr390 {

static const char *const TAG = "dfrobot_ltr390";

// DFRobot register addresses (from Python library)
// Input Registers (read-only)
static const uint8_t LTR390UV_INPUTREG_PID = 0x00;
static const uint8_t LTR390UV_INPUTREG_VID = 0x01;
static const uint8_t LTR390UV_INPUTREG_ADDR = 0x02;
static const uint8_t LTR390UV_INPUTREG_ALS_DATA_LOW = 0x07;
static const uint8_t LTR390UV_INPUTREG_UVS_DATA_LOW = 0x09;

// Holding Registers (write with +5 offset for I2C)
static const uint8_t LTR390UV_HOLDINGREG_ALS_UVS_GAIN = 0x06;
static const uint8_t LTR390UV_HOLDINGREG_ALS_UVS_MEAS_RATE = 0x0D;
static const uint8_t LTR390UV_HOLDINGREG_MAIN_CTRL = 0x0E;

// Mode values
static const uint8_t ALS_MODE = 0x02;
static const uint8_t UVS_MODE = 0x0A;

// Expected device address
static const uint8_t DEV_ADDRESS = 0x1C;

// Gain and integration time lookup tables (from Python library)
static const float GAIN_VALUES[] = {1.0, 3.0, 6.0, 9.0, 18.0};
static const float INT_TIME_VALUES[] = {4.0, 2.0, 1.0, 0.5, 0.25, 0.25}; // seconds

// todo: constructor mit logging, da componetn_state_ setzen?
DFRobotLTR390Component::DFRobotLTR390Component() {
  ESP_LOGD(TAG, "Constructor called - component created");
  ESP_LOGD(TAG, "Component state: 0x%02X", this->get_component_state());
}

void DFRobotLTR390Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DFRobot LTR390...");
  // Small startup delay to let I²C settle
  delay(50); 
  
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
  
  ESP_LOGCONFIG(TAG, "  Gain: %.0fx", this->get_gain_factor_());
  ESP_LOGCONFIG(TAG, "  Resolution: %d-bit", this->get_resolution_bits_());
  ESP_LOGCONFIG(TAG, "  Measurement rate: %dms", this->get_measurement_rate_ms_());
  
  LOG_SENSOR("  ", "Ambient Light", this->ambient_light_sensor_);
  LOG_SENSOR("  ", "UV Index", this->uv_index_sensor_);
}

bool DFRobotLTR390Component::initialize_sensor_() {
  // Check device address first
  uint8_t addr_buffer[2];
  if (!this->read_bytes(LTR390UV_INPUTREG_ADDR, addr_buffer, 2)) {
    ESP_LOGE(TAG, "Failed to read device address");
    return false;
  }
  
  uint16_t device_addr = addr_buffer[0] | (addr_buffer[1] << 8);
  ESP_LOGD(TAG, "Device address: 0x%04X", device_addr);
  
  if (device_addr != DEV_ADDRESS) {
    ESP_LOGE(TAG, "Invalid device address: 0x%04X (expected 0x%04X)", device_addr, DEV_ADDRESS);
    return false;
  }
  
  // Configure gain (using holding register with +5 offset)
  uint8_t gain_data[] = {this->gain_, 0x00};
  if (!this->write_bytes(LTR390UV_HOLDINGREG_ALS_UVS_GAIN + 5, gain_data, 2)) {
    ESP_LOGE(TAG, "Failed to set gain");
    return false;
  }
  
  // Configure measurement rate and resolution (using holding register with +5 offset)
  uint8_t meas_rate_config = (this->resolution_ << 4) | this->measurement_rate_;
  uint8_t meas_data[] = {meas_rate_config, 0x00};
  if (!this->write_bytes(LTR390UV_HOLDINGREG_ALS_UVS_MEAS_RATE + 5, meas_data, 2)) {
    ESP_LOGE(TAG, "Failed to set measurement rate and resolution");
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
  
  // Set ALS mode (using holding register with +5 offset)
  uint8_t mode_data[] = {ALS_MODE, 0x00};
  if (!this->write_bytes(LTR390UV_HOLDINGREG_MAIN_CTRL + 5, mode_data, 2)) {
    ESP_LOGW(TAG, "Failed to set ALS mode");
    this->start_uv_reading_();
    return;
  }
  
  // Wait for measurement to complete
  uint32_t wait_time = this->get_measurement_rate_ms_() + 200;  // Add extra buffer
  this->set_timeout("als_read", wait_time, [this]() {
    this->read_als_data_();
  });
}

void DFRobotLTR390Component::read_als_data_() {
  // Read 4 bytes of ALS data (from Python: buffer[2]<<16|buffer[3]<<24|buffer[0]|buffer[1]<<8)
  uint8_t buffer[4];
  if (!this->read_bytes(LTR390UV_INPUTREG_ALS_DATA_LOW, buffer, 4)) {
    ESP_LOGW(TAG, "Failed to read ALS data");
  } else {
    // Combine bytes using DFRobot's byte ordering
    uint32_t als_data = (uint32_t(buffer[2]) << 16) | 
                       (uint32_t(buffer[3]) << 24) | 
                       buffer[0] | 
                       (uint32_t(buffer[1]) << 8);
    
    float lux = this->calculate_lux_(als_data);
    this->ambient_light_sensor_->publish_state(lux);
    ESP_LOGD(TAG, "ALS raw: %u (0x%02X %02X %02X %02X), Lux: %.2f", 
             als_data, buffer[0], buffer[1], buffer[2], buffer[3], lux);
  }
  
  // Wait before switching to UV mode
  this->set_timeout("switch_to_uv", 100, [this]() {
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
  
  // Set UV mode (using holding register with +5 offset)
  uint8_t mode_data[] = {UVS_MODE, 0x00};
  if (!this->write_bytes(LTR390UV_HOLDINGREG_MAIN_CTRL + 5, mode_data, 2)) {
    ESP_LOGW(TAG, "Failed to set UV mode");
    this->reading_state_ = ReadingState::IDLE;
    return;
  }
  
  // Wait for measurement to complete
  uint32_t wait_time = this->get_measurement_rate_ms_() + 200;  // Add extra buffer
  this->set_timeout("uv_read", wait_time, [this]() {
    this->read_uv_data_();
  });
}

void DFRobotLTR390Component::read_uv_data_() {
  // Read 4 bytes of UV data (using same byte ordering as ALS)
  uint8_t buffer[4];
  if (!this->read_bytes(LTR390UV_INPUTREG_UVS_DATA_LOW, buffer, 4)) {
    ESP_LOGW(TAG, "Failed to read UV data");
  } else {
    // Combine bytes using DFRobot's byte ordering
    uint32_t uvs_data = (uint32_t(buffer[2]) << 16) | 
                       (uint32_t(buffer[3]) << 24) | 
                       buffer[0] | 
                       (uint32_t(buffer[1]) << 8);
    
    float uv_index = this->calculate_uv_index_(uvs_data);
    this->uv_index_sensor_->publish_state(uv_index);
    ESP_LOGD(TAG, "UV raw: %u (0x%02X %02X %02X %02X), UV Index: %.2f", 
             uvs_data, buffer[0], buffer[1], buffer[2], buffer[3], uv_index);
  }
  
  // Reading complete
  this->reading_state_ = ReadingState::IDLE;
}

bool DFRobotLTR390Component::write_register_(uint8_t reg, uint8_t value) {
  uint8_t data[] = {value, 0x00};
  bool success = this->write_bytes(reg + 5, data, 2);  // +5 offset for holding registers
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
  
  // Use DFRobot's exact formula from Python library:
  // data = (0.6*data)/(a_gain[self.gain]*a_int[self.resolution])
  float gain_factor = this->get_gain_factor_();
  float integration_time = this->get_integration_time_();
  
  float lux = (0.6 * als_data) / (gain_factor * integration_time);
  return lux;
}

float DFRobotLTR390Component::calculate_uv_index_(uint32_t uvs_data) {
  if (uvs_data == 0) return 0.0;
  
  // For UV, the Python library doesn't show a transformed calculation,
  // so we'll use a basic scaling. This may need adjustment based on testing.
  float gain_factor = this->get_gain_factor_();
  float integration_time = this->get_integration_time_();
  
  // Estimated UV Index calculation (may need calibration)
  float uv_index = (uvs_data * 0.25) / (gain_factor * integration_time * 262144.0);
  return uv_index;
}

float DFRobotLTR390Component::get_gain_factor_() {
  if (this->gain_ < 5) {
    return GAIN_VALUES[this->gain_];
  }
  return 3.0; // Default
}

float DFRobotLTR390Component::get_integration_time_() {
  if (this->resolution_ < 6) {
    return INT_TIME_VALUES[this->resolution_];
  }
  return 1.0; // Default
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

uint8_t DFRobotLTR390Component::get_resolution_bits_() {
  switch (this->resolution_) {
    case 0: return 20;  // e20bit = 0
    case 1: return 19;  // e19bit = 16 >> 4 = 1
    case 2: return 18;  // e18bit = 32 >> 4 = 2
    case 3: return 17;  // e17bit = 48 >> 4 = 3
    case 4: return 16;  // e16bit = 64 >> 4 = 4
    case 5: return 13;  // e13bit = 80 >> 4 = 5
    default: return 18;
  }
}

}  // namespace dfrobot_ltr390
}  // namespace esphome
