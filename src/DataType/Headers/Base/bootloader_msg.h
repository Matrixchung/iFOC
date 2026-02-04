#pragma once

#include <cstdint>

namespace iFOC::DataType::Base
{
struct BootloaderMsg
{
    uint8_t server_node_id = 0;
    uint8_t update_image_path[7]{};
};
}