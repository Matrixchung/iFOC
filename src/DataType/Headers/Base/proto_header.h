/*
 *  This file is generated with Embedded Proto, PLEASE DO NOT EDIT!
 *  source: Base/proto_header.proto
 */

// This file is generated. Please do not edit!
#ifndef BASE_PROTO_HEADER_H
#define BASE_PROTO_HEADER_H

#include <cstdint>
// Include external proto definitions

namespace iFOC {
namespace DataType {
namespace Base {

enum class ProtoHeader : uint32_t
{
  NOT_USED = 0,
  BOARD_CONFIG = 1,
  FOC_MOTOR_CONFIG = 2
};

} // End of namespace Base
} // End of namespace DataType
} // End of namespace iFOC
#endif // BASE_PROTO_HEADER_H