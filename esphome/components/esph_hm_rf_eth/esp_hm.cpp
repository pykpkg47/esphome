#include "esp_hm.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#ifdef USE_ESP32
#include "radiomoduledetector.h"

static const char *const TAG = "esp-hm";

namespace esphome {
namespace esph_hm {

void esph_hm::setup() {
  ESP_LOGD(TAG, "setup started");

  this->radioModuleConnector_ =
      new RadioModuleConnector(this->reset_, this->uart_->get_uart_event_queue(), this->uart_->get_hw_serial_number());
  radioModuleConnector_->addLed(this->red_, this->green_, this->blue_);

  ESP_LOGD(TAG, "RadioModuleConnector started");
  this->radioModuleConnector_->start();

  ESP_LOGD(TAG, "HM Modul detecting started");
  RadioModuleDetector radioModuleDetector;
  radioModuleDetector.detectRadioModule(radioModuleConnector_);

  auto radioModuleType = radioModuleDetector.getRadioModuleType();

  std::string type;
  if (radioModuleType != RADIO_MODULE_NONE) {
    switch (radioModuleType) {
      case RADIO_MODULE_HM_MOD_RPI_PCB:
        ESP_LOGD(TAG, "Detected HM-MOD-RPI-PCB");
        type = "HM-MOD-RPI-PCB";
        break;
      case RADIO_MODULE_RPI_RF_MOD:
        ESP_LOGD(TAG, "Detected RPI-RF-MOD");
        type = "RPI-RF-MOD";
        break;
      default:
        ESP_LOGD(TAG, "Detected unknown radio module");
        type = "unknown radio module";
        break;
    }

    auto firmwareVersion = radioModuleDetector.getFirmwareVersion();
    ESP_LOGD(TAG, "Firmware Version: %i.%i.%i", firmwareVersion[0], firmwareVersion[1], firmwareVersion[2]);
    if (this->firmware_sensor_) {
      this->firmware_sensor_->publish_state(
          esphome::str_sprintf("%i.%i.%i", firmwareVersion[0], firmwareVersion[1], firmwareVersion[2]));
    }
    auto serial = radioModuleDetector.getSerial();
    ESP_LOGD(TAG, "Serial %s", serial);
    if (this->serial_sensor_) {
      this->serial_sensor_->publish_state(serial);
    }
    auto sgtin = radioModuleDetector.getSGTIN();
    ESP_LOGD(TAG, " SGTIN: %s", sgtin);
    if (this->SGTIN_sensor_) {
      this->SGTIN_sensor_->publish_state(sgtin);
    }

    ESP_LOGD(TAG, "Starting Raw Uart Udp Listener");
    this->rawUartUdpListener_ = new RawUartUdpListener(this->radioModuleConnector_);
    this->rawUartUdpListener_->start();

  } else {
    ESP_LOGE(TAG, "Radio module could not be detected.");
    type = ("No Radio Module");
    this->mark_failed();
  }

  if (this->radio_module_sensor_) {
    this->radio_module_sensor_->publish_state(type);
  }
}

void esphome::esph_hm::esph_hm::update() {
  if (this->connected_) {
    bool newState = this->rawUartUdpListener_->isConnected();
    if (newState != this->connected_->state) {
      this->connected_->publish_state(newState);
    }
  }
}

// void esph_hm::loop() {
//   auto uart_num = this->uart_->get_hw_serial_number();
//   auto uart_queue = *this->uart_->get_uart_event_queue();
//   uart_event_t event;

//   if (xQueueReceive(uart_queue, (void *) &event, (TickType_t) (10 / portTICK_PERIOD_MS))) {
//     switch (event.type) {
//       case UART_DATA:
//         ESP_LOGD(TAG, "received uart data event size %i", event.size);
//         ESP_LOGD(TAG, "on  %i", uart_num);
//         uart_read_bytes(uart_num, this->buffer_, event.size, portMAX_DELAY);
//         log_frame("Received HM frame:", this->buffer_, event.size);
//         //_streamParser->append(buffer, event.size);
//         break;
//       case UART_FIFO_OVF:
//       case UART_BUFFER_FULL:
//         uart_flush_input(uart_num);
//         xQueueReset(uart_queue);
//         //_streamParser->flush();
//         break;
//       case UART_BREAK:
//       case UART_PARITY_ERR:
//       case UART_FRAME_ERR:
//         //_streamParser->flush();
//         break;
//       default:
//         break;
//     }
//   } else {
//     // ESP_LOGD(TAG, "nothing received in loop iteration");
//   }
// }

void esph_hm::dump_config() {
  ESP_LOGCONFIG(TAG, "esp-hm Component Configuration:");

  ESP_LOGCONFIG(TAG, "uart number %i", this->uart_->get_hw_serial_number());

  if (this->red_) {
    ESP_LOGCONFIG(TAG, "  Red LED: Configured");
  }
  if (this->green_) {
    ESP_LOGCONFIG(TAG, "  Green LED: Configured");
  }
  if (this->blue_) {
    ESP_LOGCONFIG(TAG, "  Blue LED: Configured");
  }
}
void esph_hm::on_shutdown() {}

}  // namespace esph_hm
}  // namespace esphome

#endif
