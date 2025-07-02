/*
 *  This file is generated with Embedded Proto, PLEASE DO NOT EDIT!
 *  source: Comm/can_baudrate.proto
 */

// This file is generated. Please do not edit!
#ifndef COMM_CAN_BAUDRATE_H
#define COMM_CAN_BAUDRATE_H

#include <cstdint>
// Include external proto definitions

namespace iFOC {
namespace DataType {
namespace Comm {

enum class CANBaudrate : uint32_t
{
  BAUD_AUTO_DETECT = 0,
  BAUD_10_KBPS = 1,
  BAUD_20_KBPS = 2,
  BAUD_50_KBPS = 3,
  BAUD_100_KBPS = 4,
  BAUD_125_KBPS = 5,
  BAUD_250_KBPS = 6,
  BAUD_500_KBPS = 7,
  BAUD_800_KBPS = 8,
  BAUD_1_MBPS = 9,
  BAUD_2_MBPS = 10,
  BAUD_5_MBPS = 11
};

} // End of namespace Comm
} // End of namespace DataType
} // End of namespace iFOC
#endif // COMM_CAN_BAUDRATE_H