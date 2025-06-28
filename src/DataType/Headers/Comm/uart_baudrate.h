/*
 *  This file is generated with Embedded Proto, PLEASE DO NOT EDIT!
 *  source: Comm/uart_baudrate.proto
 */

// This file is generated. Please do not edit!
#ifndef COMM_UART_BAUDRATE_H
#define COMM_UART_BAUDRATE_H

#include <cstdint>
// Include external proto definitions

namespace iFOC {
namespace DataType {
namespace Comm {

enum class UARTBaudrate : uint32_t
{
  BAUD_115200 = 0,
  BAUD_9600 = 1,
  BAUD_230400 = 2,
  BAUD_460800 = 3,
  BAUD_921600 = 4,
  BAUD_1843200 = 5
};

} // End of namespace Comm
} // End of namespace DataType
} // End of namespace iFOC
#endif // COMM_UART_BAUDRATE_H