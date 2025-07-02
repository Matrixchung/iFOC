static void init_object_dictionary(iFOC::MiniCANOpen& can)
{
    using namespace iFOC;
    // 0x1000, Device Type
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1000;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_VAR;
        entry.subIndexes.resize(1);

        // 0x1000sub0, Device Type
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        can.AddODEntry(entry);
    }

    // 0x1001, Error Register
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1001;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_VAR;
        entry.subIndexes.resize(1);

        // 0x1001sub0, Error Register
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        can.AddODEntry(entry);
    }

    // 0x1014, Emergency COB ID
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1014;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_VAR;
        entry.subIndexes.resize(1);

        // 0x1014sub0, Emergency COB ID
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = can.GetNodeID()+0x80;
            memcpy(sub.pObject, &val, 4);
        }
        can.AddODEntry(entry);
    }

    // 0x1017, Producer Heartbeat Time
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1017;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_VAR;
        entry.subIndexes.resize(1);

        // 0x1017sub0, Producer Heartbeat Time
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 1000;
            memcpy(sub.pObject, &val, 2);
        }
        can.AddODEntry(entry);
    }

    // 0x1018, Identity
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1018;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_RECORD;
        entry.subIndexes.resize(5);

        // 0x1018sub0, Number of Entries
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 4;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1018sub1, Vendor ID
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1018sub2, Product Code
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1018sub3, Revision Number
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1018sub4, Serial Number
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[4];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        can.AddODEntry(entry);
    }

    // 0x1200, Server SDO Parameter
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1200;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_RECORD;
        entry.subIndexes.resize(3);

        // 0x1200sub0, Number of Entries
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 2;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1200sub1, COB ID Client to Server (Receive SDO)
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = can.GetNodeID()+0x600;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1200sub2, COB ID Server to Client (Transmit SDO)
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = can.GetNodeID()+0x580;
            memcpy(sub.pObject, &val, 4);
        }
        can.AddODEntry(entry);
    }

    // 0x1400, RXPDO 1 Receive PDO 1 Parameter
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1400;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_RECORD;
        entry.subIndexes.resize(6);

        // 0x1400sub0, Highest SubIndex Supported
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 6;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1400sub1, COB ID used by PDO
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = can.GetNodeID()+0x200;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1400sub2, Transmission Type
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1400sub3, Inhibit Time
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1400sub5, Event Timer
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1400sub6, SYNC start value
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        can.AddODEntry(entry);
    }

    // 0x1401, RXPDO 2 Receive PDO 2 Parameter
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1401;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_RECORD;
        entry.subIndexes.resize(6);

        // 0x1401sub0, Highest SubIndex Supported
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 6;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1401sub1, COB ID used by PDO
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = can.GetNodeID()+0x300;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1401sub2, Transmission Type
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1401sub3, Inhibit Time
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1401sub5, Event Timer
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1401sub6, SYNC start value
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        can.AddODEntry(entry);
    }

    // 0x1402, RXPDO 3 Receive PDO 3 Parameter
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1402;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_RECORD;
        entry.subIndexes.resize(6);

        // 0x1402sub0, Highest SubIndex Supported
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 6;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1402sub1, COB ID used by PDO
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = can.GetNodeID()+0x400;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1402sub2, Transmission Type
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1402sub3, Inhibit Time
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1402sub5, Event Timer
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1402sub6, SYNC start value
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        can.AddODEntry(entry);
    }

    // 0x1403, RXPDO 4 Receive PDO 4 Parameter
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1403;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_RECORD;
        entry.subIndexes.resize(6);

        // 0x1403sub0, Highest SubIndex Supported
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 6;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1403sub1, COB ID used by PDO
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = can.GetNodeID()+0x500;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1403sub2, Transmission Type
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1403sub3, Inhibit Time
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1403sub5, Event Timer
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1403sub6, SYNC start value
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        can.AddODEntry(entry);
    }

    // 0x1600, RXPDO 1 Receive PDO 1 Mapping
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1600;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_ARRAY;
        entry.subIndexes.resize(9);

        // 0x1600sub0, Number of Entries
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 8;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1600sub1, PDO 1 Mapping for an application object 1
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1600sub2, PDO 1 Mapping for an application object 2
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1600sub3, PDO 1 Mapping for an application object 3
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1600sub4, PDO 1 Mapping for an application object 4
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[4];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1600sub5, PDO 1 Mapping for an application object 5
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1600sub6, PDO 1 Mapping for an application object 6
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1600sub7, PDO 1 Mapping for an application object 7
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[7];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1600sub8, PDO 1 Mapping for an application object 8
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[8];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        can.AddODEntry(entry);
    }

    // 0x1601, RXPDO 2 Receive PDO 2 Mapping
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1601;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_ARRAY;
        entry.subIndexes.resize(9);

        // 0x1601sub0, Number of Entries
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 8;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1601sub1, PDO 2 Mapping for an application object 1
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1601sub2, PDO 2 Mapping for an application object 2
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1601sub3, PDO 2 Mapping for an application object 3
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1601sub4, PDO 2 Mapping for an application object 4
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[4];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1601sub5, PDO 2 Mapping for an application object 5
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1601sub6, PDO 2 Mapping for an application object 6
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1601sub7, PDO 2 Mapping for an application object 7
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[7];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1601sub8, PDO 2 Mapping for an application object 8
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[8];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        can.AddODEntry(entry);
    }

    // 0x1602, RXPDO 3 Receive PDO 3 Mapping
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1602;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_ARRAY;
        entry.subIndexes.resize(9);

        // 0x1602sub0, Number of Entries
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 8;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1602sub1, PDO 3 Mapping for an application object 1
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1602sub2, PDO 3 Mapping for an application object 2
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1602sub3, PDO 3 Mapping for an application object 3
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1602sub4, PDO 3 Mapping for an application object 4
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[4];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1602sub5, PDO 3 Mapping for an application object 5
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1602sub6, PDO 3 Mapping for an application object 6
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1602sub7, PDO 3 Mapping for an application object 7
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[7];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1602sub8, PDO 3 Mapping for an application object 8
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[8];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        can.AddODEntry(entry);
    }

    // 0x1603, RXPDO 4 Receive PDO 4 Mapping
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1603;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_ARRAY;
        entry.subIndexes.resize(9);

        // 0x1603sub0, Number of Entries
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 8;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1603sub1, PDO 4 Mapping for an application object 1
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1603sub2, PDO 4 Mapping for an application object 2
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1603sub3, PDO 4 Mapping for an application object 3
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1603sub4, PDO 4 Mapping for an application object 4
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[4];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1603sub5, PDO 4 Mapping for an application object 5
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1603sub6, PDO 4 Mapping for an application object 6
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1603sub7, PDO 4 Mapping for an application object 7
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[7];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1603sub8, PDO 4 Mapping for an application object 8
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[8];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        can.AddODEntry(entry);
    }

    // 0x1800, TXPDO 1 Transmit PDO 1 Parameter
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1800;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_RECORD;
        entry.subIndexes.resize(6);

        // 0x1800sub0, Highest SubIndex Supported
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 6;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1800sub1, COB ID used by PDO
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = can.GetNodeID()+0x180;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1800sub2, Transmission Type
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1800sub3, Inhibit Time
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1800sub5, Event Timer
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1800sub6, SYNC start value
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        can.AddODEntry(entry);
    }

    // 0x1801, TXPDO 2 Transmit PDO 2 Parameter
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1801;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_RECORD;
        entry.subIndexes.resize(6);

        // 0x1801sub0, Highest SubIndex Supported
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 6;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1801sub1, COB ID used by PDO
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = can.GetNodeID()+0x280;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1801sub2, Transmission Type
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1801sub3, Inhibit Time
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1801sub5, Event Timer
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1801sub6, SYNC start value
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        can.AddODEntry(entry);
    }

    // 0x1802, TXPDO 3 Transmit PDO 3 Parameter
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1802;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_RECORD;
        entry.subIndexes.resize(6);

        // 0x1802sub0, Highest SubIndex Supported
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 6;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1802sub1, COB ID used by PDO
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = can.GetNodeID()+0x380;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1802sub2, Transmission Type
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1802sub3, Inhibit Time
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1802sub5, Event Timer
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1802sub6, SYNC start value
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        can.AddODEntry(entry);
    }

    // 0x1803, TXPDO 4 Transmit PDO 4 Parameter
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1803;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_RECORD;
        entry.subIndexes.resize(6);

        // 0x1803sub0, Highest SubIndex Supported
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RO;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 6;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1803sub1, COB ID used by PDO
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = can.GetNodeID()+0x480;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1803sub2, Transmission Type
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1803sub3, Inhibit Time
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1803sub5, Event Timer
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        // 0x1803sub6, SYNC start value
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 0;
            memcpy(sub.pObject, &val, 1);
        }
        can.AddODEntry(entry);
    }

    // 0x1A00, TXPDO 1 Transmit PDO 1 Mapping
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1A00;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_ARRAY;
        entry.subIndexes.resize(9);

        // 0x1A00sub0, Number of Entries
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 8;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1A00sub1, PDO 1 Mapping for a process data variable 1
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A00sub2, PDO 1 Mapping for a process data variable 2
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A00sub3, PDO 1 Mapping for a process data variable 3
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A00sub4, PDO 1 Mapping for a process data variable 4
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[4];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A00sub5, PDO 1 Mapping for a process data variable 5
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A00sub6, PDO 1 Mapping for a process data variable 6
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A00sub7, PDO 1 Mapping for a process data variable 7
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[7];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A00sub8, PDO 1 Mapping for a process data variable 8
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[8];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        can.AddODEntry(entry);
    }

    // 0x1A01, TXPDO 2 Transmit PDO 2 Mapping
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1A01;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_ARRAY;
        entry.subIndexes.resize(9);

        // 0x1A01sub0, Number of Entries
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 8;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1A01sub1, PDO 2 Mapping for a process data variable 1
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A01sub2, PDO 2 Mapping for a process data variable 2
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A01sub3, PDO 2 Mapping for a process data variable 3
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A01sub4, PDO 2 Mapping for a process data variable 4
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[4];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A01sub5, PDO 2 Mapping for a process data variable 5
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A01sub6, PDO 2 Mapping for a process data variable 6
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A01sub7, PDO 2 Mapping for a process data variable 7
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[7];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A01sub8, PDO 2 Mapping for a process data variable 8
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[8];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        can.AddODEntry(entry);
    }

    // 0x1A02, TXPDO 3 Transmit PDO 3 Mapping
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1A02;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_ARRAY;
        entry.subIndexes.resize(9);

        // 0x1A02sub0, Number of Entries
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 8;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1A02sub1, PDO 3 Mapping for a process data variable 1
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A02sub2, PDO 3 Mapping for a process data variable 2
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A02sub3, PDO 3 Mapping for a process data variable 3
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A02sub4, PDO 3 Mapping for a process data variable 4
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[4];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A02sub5, PDO 3 Mapping for a process data variable 5
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A02sub6, PDO 3 Mapping for a process data variable 6
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A02sub7, PDO 3 Mapping for a process data variable 7
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[7];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A02sub8, PDO 3 Mapping for a process data variable 8
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[8];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        can.AddODEntry(entry);
    }

    // 0x1A03, TXPDO 4 Transmit PDO 4 Mapping
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x1A03;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_ARRAY;
        entry.subIndexes.resize(9);

        // 0x1A03sub0, Number of Entries
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT8;
            sub.size = 1;
            sub.pObject = can.allocator.allocate(1);
            uint8_t val = 8;
            memcpy(sub.pObject, &val, 1);
        }
        // 0x1A03sub1, PDO 4 Mapping for a process data variable 1
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[1];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A03sub2, PDO 4 Mapping for a process data variable 2
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[2];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A03sub3, PDO 4 Mapping for a process data variable 3
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[3];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A03sub4, PDO 4 Mapping for a process data variable 4
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[4];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A03sub5, PDO 4 Mapping for a process data variable 5
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[5];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A03sub6, PDO 4 Mapping for a process data variable 6
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[6];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A03sub7, PDO 4 Mapping for a process data variable 7
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[7];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        // 0x1A03sub8, PDO 4 Mapping for a process data variable 8
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[8];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT32;
            sub.size = 4;
            sub.pObject = can.allocator.allocate(4);
            uint32_t val = 0;
            memcpy(sub.pObject, &val, 4);
        }
        can.AddODEntry(entry);
    }

    // 0x6040, Controlword
    {
        MiniCANOpen::ODEntry entry;
        entry.index = 0x6040;
        entry.objectType = MiniCANOpen::ODObjectType::OBJ_VAR;
        entry.subIndexes.resize(1);

        // 0x6040sub0, Controlword
        {
            MiniCANOpen::ODSubIndex& sub = entry.subIndexes[0];
            sub.accessType = MiniCANOpen::ODAccessType::RW;
            sub.dataType = MiniCANOpen::ODDataType::UINT16;
            sub.size = 2;
            sub.pObject = can.allocator.allocate(2);
            uint16_t val = 0;
            memcpy(sub.pObject, &val, 2);
        }
        can.AddODEntry(entry);
    }

}

