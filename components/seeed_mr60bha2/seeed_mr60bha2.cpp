#include "seeed_mr60bha2.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <utility>

namespace esphome {
namespace seeed_mr60bha2 {

static const char *const TAG = "seeed_mr60bha2";

// Prints the component's configuration data. dump_config() prints all of the component's configuration
// items in an easy-to-read format, including the configuration key-value pairs.
void MR60BHA2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MR60BHA2:");
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Has Target", this->has_target_binary_sensor_);
#endif
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Breath Rate", this->breath_rate_sensor_);
  LOG_SENSOR("  ", "Heart Rate", this->heart_rate_sensor_);
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
  LOG_SENSOR("  ", "Number of Targets", this->num_targets_sensor_);
  LOG_SENSOR("  ", "Total Phase", this->total_phase_sensor_);
  LOG_SENSOR("  ", "Breath Phase", this->breath_phase_sensor_);
  LOG_SENSOR("  ", "Heart Phase", this->heart_phase_sensor_);
  LOG_SENSOR("  ", "Target 1 X", this->target_1_x_sensor_);
  LOG_SENSOR("  ", "Target 1 Y", this->target_1_y_sensor_);
  LOG_SENSOR("  ", "Target 2 X", this->target_2_x_sensor_);
  LOG_SENSOR("  ", "Target 2 Y", this->target_2_y_sensor_);
  LOG_SENSOR("  ", "Target 3 X", this->target_3_x_sensor_);
  LOG_SENSOR("  ", "Target 3 Y", this->target_3_y_sensor_);
#endif
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Firmware Version", this->firmware_version_text_sensor_);
#endif
}

struct FirmwareVersion {
  uint8_t project_name;
  uint8_t major_version;
  uint8_t sub_version;
  uint8_t modified_version;
};

// main loop
void MR60BHA2Component::loop() {
  uint8_t byte;

  // Is there data on the serial port
  while (this->available()) {
    this->read_byte(&byte);
    this->rx_message_.push_back(byte);
    if (!this->validate_message_()) {
      this->rx_message_.clear();
    }
  }
}

/**
 * @brief Calculate the checksum for a byte array.
 *
 * This function calculates the checksum for the provided byte array using an
 * XOR-based checksum algorithm.
 *
 * @param data The byte array to calculate the checksum for.
 * @param len The length of the byte array.
 * @return The calculated checksum.
 */
static uint8_t calculate_checksum(const uint8_t *data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  checksum = ~checksum;
  return checksum;
}

/**
 * @brief Validate the checksum of a byte array.
 *
 * This function validates the checksum of the provided byte array by comparing
 * it to the expected checksum.
 *
 * @param data The byte array to validate.
 * @param len The length of the byte array.
 * @param expected_checksum The expected checksum.
 * @return True if the checksum is valid, false otherwise.
 */
static bool validate_checksum(const uint8_t *data, size_t len, uint8_t expected_checksum) {
  return calculate_checksum(data, len) == expected_checksum;
}

bool MR60BHA2Component::validate_message_() {
  size_t at = this->rx_message_.size() - 1;
  auto *data = &this->rx_message_[0];

  if (at == 0) {
    return data[at] == FRAME_HEADER_BUFFER;
  }

  if (at <= 6) {
    return true;
  }

  uint16_t frame_type = encode_uint16(data[5], data[6]);

  if (frame_type != BREATH_RATE_TYPE_BUFFER && frame_type != HEART_RATE_TYPE_BUFFER &&
      frame_type != DISTANCE_TYPE_BUFFER && frame_type != PEOPLE_EXIST_TYPE_BUFFER &&
      frame_type != POINT_CLOUD_TARGET_INFO_BUFFER && frame_type != HEART_BREATH_PHASE_BUFFER &&
      frame_type != FIRMWARE_VERSION_BUFFER) {
    return false;
  }

  uint8_t header_checksum = data[at];

  if (at == 7) {
    if (!validate_checksum(data, 7, header_checksum)) {
      ESP_LOGE(TAG, "HEAD_CKSUM_FRAME ERROR: 0x%02x", header_checksum);
      ESP_LOGV(TAG, "GET FRAME: %s", format_hex_pretty(data, 8).c_str());
      return false;
    }
    return true;
  }

  // Wait until all data is read
  uint16_t length = encode_uint16(data[3], data[4]);
  if (at - 8 < length) {
    return true;
  }

  uint8_t data_checksum = data[at];
  if (at == 8 + length) {
    if (!validate_checksum(data + 8, length, data_checksum)) {
      ESP_LOGE(TAG, "DATA_CKSUM_FRAME ERROR: 0x%02x", data_checksum);
      ESP_LOGV(TAG, "GET FRAME: %s", format_hex_pretty(data, 8 + length).c_str());
      return false;
    }
  }

  uint16_t frame_id = encode_uint16(data[1], data[2]);
  const uint8_t *frame_data = data + 8;
  ESP_LOGV(TAG, "Received Frame: ID: 0x%04x, Type: 0x%04x, Data: [%s] Raw Data: [%s]", frame_id, frame_type,
           format_hex_pretty(frame_data, length).c_str(), format_hex_pretty(this->rx_message_).c_str());
  this->process_frame_(frame_id, frame_type, data + 8, length);

  // Return false to reset rx buffer
  return false;
}

void MR60BHA2Component::process_frame_(uint16_t frame_id, uint16_t frame_type, const uint8_t *data, size_t length) {
  this->target_x_sensors_[0] = this->target_1_x_sensor_;
  this->target_y_sensors_[0] = this->target_1_y_sensor_;
  this->target_x_sensors_[1] = this->target_2_x_sensor_;
  this->target_y_sensors_[1] = this->target_2_y_sensor_;
  this->target_x_sensors_[2] = this->target_3_x_sensor_;
  this->target_y_sensors_[2] = this->target_3_y_sensor_;
  
  switch (frame_type) {
    case HEART_BREATH_PHASE_BUFFER: {
      if (length >= 12) {
        if (this->total_phase_sensor_ != nullptr)
          this->total_phase_sensor_->publish_state(*reinterpret_cast<const float *>(&data[0]));
        if (this->breath_phase_sensor_ != nullptr)
          this->breath_phase_sensor_->publish_state(*reinterpret_cast<const float *>(&data[4]));
        if (this->heart_phase_sensor_ != nullptr)
          this->heart_phase_sensor_->publish_state(*reinterpret_cast<const float *>(&data[8]));
      }
      break;
    }

    case BREATH_RATE_TYPE_BUFFER: {
      if (this->breath_rate_sensor_ != nullptr && length >= 4) {
        this->breath_rate_sensor_->publish_state(*reinterpret_cast<const float *>(data));
      }
      break;
    }

    case HEART_RATE_TYPE_BUFFER: {
      if (this->heart_rate_sensor_ != nullptr && length >= 4) {
        this->heart_rate_sensor_->publish_state(*reinterpret_cast<const float *>(data));
      }
      break;
    }

    case DISTANCE_TYPE_BUFFER: {
      if (this->distance_sensor_ != nullptr && length >= 8 && data[0] != 0) {
        this->distance_sensor_->publish_state(*reinterpret_cast<const float *>(&data[4]));
      }
      break;
    }

    case PEOPLE_EXIST_TYPE_BUFFER: {
      if (this->has_target_binary_sensor_ != nullptr && length >= 2) {
        bool detected = encode_uint16(data[1], data[0]);
        this->has_target_binary_sensor_->publish_state(detected);
        if (!detected) { // If no one is detected, reset other sensors
          if (this->breath_rate_sensor_ != nullptr) this->breath_rate_sensor_->publish_state(0);
          if (this->heart_rate_sensor_ != nullptr) this->heart_rate_sensor_->publish_state(0);
          if (this->distance_sensor_ != nullptr) this->distance_sensor_->publish_state(0);
          if (this->num_targets_sensor_ != nullptr) this->num_targets_sensor_->publish_state(0);
        }
      }
      break;
    }

        case POINT_CLOUD_TARGET_INFO_BUFFER: {
      if (length >= 4) {
        uint32_t num_targets = encode_uint32(data[3], data[2], data[1], data[0]);
        if (this->num_targets_sensor_ != nullptr) {
          this->num_targets_sensor_->publish_state(num_targets);
        }

        const uint8_t *ptr = data + 4;
        for (int i = 0; i < MAX_TARGETS; i++) {
          if (i < num_targets) {
            // This target exists, publish its data
            float x = *reinterpret_cast<const float *>(&ptr[0]);
            float y = *reinterpret_cast<const float *>(&ptr[4]);
            
            if (this->target_x_sensors_[i] != nullptr) this->target_x_sensors_[i]->publish_state(x);
            if (this->target_y_sensors_[i] != nullptr) this->target_y_sensors_[i]->publish_state(y);

            ptr += 16; // Move pointer to the next target's data block
          } else {
            // This target does not exist, publish 0 to clear old values
            if (this->target_x_sensors_[i] != nullptr) this->target_x_sensors_[i]->publish_state(0.0f);
            if (this->target_y_sensors_[i] != nullptr) this->target_y_sensors_[i]->publish_state(0.0f);
          }
        }
      }
      break;
    }

    case FIRMWARE_VERSION_BUFFER: {
      if (this->firmware_version_text_sensor_ != nullptr && length >= 4) {
        // Cast the data buffer to the FirmwareVersion struct
        const auto *version_info = reinterpret_cast<const FirmwareVersion *>(data);
        
        // Construct the firmware string from the struct members
        std::string fw = to_string(version_info->major_version) + "." +
                         to_string(version_info->sub_version) + "." +
                         to_string(version_info->modified_version);
                         
        this->firmware_version_text_sensor_->publish_state(fw);
      }
      break;
    }

    default:
      ESP_LOGV(TAG, "Unhandled frame type: 0x%04X", frame_type);
      break;
  }
}

}  // namespace seeed_mr60bha2
}  // namespace esphome
