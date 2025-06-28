/*
 *  This file is generated with Embedded Proto, PLEASE DO NOT EDIT!
 *  source: Comm/uart_protocol.proto
 */

// This file is generated. Please do not edit!
#ifndef COMM_UART_PROTOCOL_H
#define COMM_UART_PROTOCOL_H

#include <cstdint>
// Include external proto definitions

namespace iFOC {
namespace DataType {
namespace Comm {

enum class UARTProtocol : uint32_t
{
  ASCII = 0,
  VOFA = 1,
  RAW_PROTO = 2,
  MATLAB = 3
};

} // End of namespace Comm
} // End of namespace DataType
} // End of namespace iFOC
#endif // COMM_UART_PROTOCOL_H