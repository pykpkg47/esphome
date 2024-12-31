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

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "streamparser.h"
#include <atomic>
#define _Atomic(X) std::atomic<X>
#include "esphome/components/output/binary_output.h"

class FrameHandler {
 public:
  virtual void handleFrame(unsigned char *buffer, uint16_t len) = 0;
};

using BinaryOutput = esphome::output::BinaryOutput;
using LED = BinaryOutput;

class RadioModuleConnector {
 private:
  LED *_redLED{nullptr};
  LED *_greenLED{nullptr};
  LED *_blueLED{nullptr};
  BinaryOutput *_reset;
  StreamParser *_streamParser;
  std::atomic<FrameHandler *> _frameHandler = ATOMIC_VAR_INIT(0);
  QueueHandle_t _uart_queue;
  uart_port_t _uart_num;
  TaskHandle_t _tHandle{nullptr};
  uint8_t *_buffer{nullptr};

  void _handleFrame(unsigned char *buffer, uint16_t len);

 public:
  RadioModuleConnector(BinaryOutput *reset, QueueHandle_t *uart_queue, uart_port_t uart_num);

  void start();
  void stop();

  void setFrameHandler(FrameHandler *handler, bool decodeEscaped);

  void resetModule();

  void sendFrame(unsigned char *buffer, uint16_t len);

  void _serialQueueHandler();

  void setLED(bool red, bool green, bool blue);

  void addLed(LED *redLED, LED *greenLED, LED *blueLED) {
    _redLED = redLED;
    _greenLED = greenLED;
    _blueLED = blueLED;
  }
};
