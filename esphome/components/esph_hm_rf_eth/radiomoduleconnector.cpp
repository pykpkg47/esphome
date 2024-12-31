/*
 *  radiomoduleconnector.h is part of the HB-RF-ETH firmware - https://github.com/alexreinert/HB-RF-ETH
 *
 *  Copyright 2022 Alexander Reinert
 *
 *  The HB-RF-ETH firmware is licensed under a
 *  Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 *  You should have received a copy of the license along with this
 *  work.  If not, see <http://creativecommons.org/licenses/by-nc-sa/4.0/>.
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "radiomoduleconnector.h"
#include "hmframe.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "radiomoduledetector_utils.h"

using esphome::esp_log_printf_;

static const char *TAG = "RadioModuleConnector";

void serialQueueHandlerTask(void *parameter) { ((RadioModuleConnector *) parameter)->_serialQueueHandler(); }

RadioModuleConnector::RadioModuleConnector(BinaryOutput *reset, QueueHandle_t *uart_queue, uart_port_t uart_num)
    : _reset(reset), _uart_queue(*uart_queue), _uart_num(uart_num) {
  using namespace std::placeholders;
  _streamParser = new StreamParser(false, std::bind(&RadioModuleConnector::_handleFrame, this, _1, _2));
}

void RadioModuleConnector::start() {
  xTaskCreate(serialQueueHandlerTask, "RadioModuleConnector_UART_QueueHandler", 4096, this, 15, &_tHandle);
  resetModule();
}

void RadioModuleConnector::stop() {
  if (_tHandle) {
    vTaskDelete(_tHandle);
    _tHandle = nullptr;
    resetModule();
  }
}

void RadioModuleConnector::setFrameHandler(FrameHandler *frameHandler, bool decodeEscaped) {
  atomic_store(&_frameHandler, frameHandler);
  _streamParser->setDecodeEscaped(decodeEscaped);
}

void RadioModuleConnector::resetModule() {
  _reset->turn_on();
  vTaskDelay(50 / portTICK_PERIOD_MS);
  _reset->turn_off();
  vTaskDelay(50 / portTICK_PERIOD_MS);
}

void RadioModuleConnector::sendFrame(unsigned char *buffer, uint16_t len) {
  uart_write_bytes(_uart_num, (const char *) buffer, len);
}

void RadioModuleConnector::_serialQueueHandler() {
  uart_event_t event;
  uint8_t *buffer = (uint8_t *) malloc(UART_FIFO_LEN);

  uart_flush_input(_uart_num);

  for (;;) {
    if (xQueueReceive(_uart_queue, (void *) &event, (TickType_t) portMAX_DELAY)) {
      switch (event.type) {
        case UART_DATA:
          uart_read_bytes(_uart_num, buffer, event.size, portMAX_DELAY);
          _streamParser->append(buffer, event.size);
          break;
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
          uart_flush_input(_uart_num);
          xQueueReset(_uart_queue);
          _streamParser->flush();
          break;
        case UART_BREAK:
        case UART_PARITY_ERR:
        case UART_FRAME_ERR:
          _streamParser->flush();
          break;
        default:
          break;
      }
    }
  }

  free(buffer);
  buffer = NULL;
  vTaskDelete(NULL);
}

void RadioModuleConnector::setLED(bool red, bool green, bool blue) {
  ESP_LOGD(TAG, "red : %s green: %s blue:%s", red ? "on" : "off", green ? "on" : "off", blue ? "on" : "off");
  if (_redLED)
    _redLED->set_state(red);
  if (_greenLED)
    _greenLED->set_state(green);
  if (_blueLED)
    _blueLED->set_state(blue);
}

void RadioModuleConnector::_handleFrame(unsigned char *buffer, uint16_t len) {
  log_frame("uart frame received:", buffer, len);
  FrameHandler *frameHandler = (FrameHandler *) atomic_load(&_frameHandler);

  if (frameHandler) {
    frameHandler->handleFrame(buffer, len);
  }
}
