/**
 * @file cell_net_l2_protocol.h
 * @brief OSI Layer-2 High-Speed Wire Protocol for the Battery Sorting Cell.
 *
 * Provides fixed-size packed binary framing over raw Ethernet (EtherType 0x88B5)
 * for sub-50-microsecond deterministic peer-to-peer communication between:
 *   - zDdsNode_vision     (Ubuntu Linux x86_64)
 *   - zDdsNode_conveyor   (WT32-ETH01)
 *   - zDdsNode_gantry     (WT32-ETH01)
 *   - zDdsNode_supervisor (Raspberry Pi ARM64)
 */

#ifndef CELL_NET_L2_PROTOCOL_H
#define CELL_NET_L2_PROTOCOL_H

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// IEEE 802.3 Local / Experimental EtherType
constexpr uint16_t CELL_NET_L2_ETHERTYPE = 0x88B5;

// Protocol Version
constexpr uint8_t CELL_NET_L2_VERSION = 0x01;

// Subsystem Node IDs
enum class CellNodeId : uint8_t {
  UNKNOWN = 0x00,
  VISION = 0x01,       // Ubuntu IPC Camera Classifier
  CONVEYOR = 0x02,     // WT32 Belt Tracker (500 PPR)
  GANTRY = 0x03,       // WT32 Multi-Axis Motion Controller
  SUPERVISOR = 0x04,   // Raspberry Pi Cell Coordinator / HMI
  BROADCAST = 0xFF,
};

// Message Type Identifiers
enum class CellMsgType : uint8_t {
  HEARTBEAT = 0x00,
  VISION_DETECT = 0x01,    // Vision -> Gantry / Supervisor
  CONVEYOR_SPEED = 0x02,   // Conveyor -> Gantry / Supervisor
  GANTRY_STATUS = 0x03,    // Gantry -> Supervisor
  CELL_COMMAND = 0x04,     // Supervisor -> Gantry / Conveyor
};

#pragma pack(push, 1)

/**
 * @brief Standard 14-byte IEEE 802.3 Ethernet Header.
 */
struct L2EthernetHeader {
  uint8_t dest_mac[6];
  uint8_t src_mac[6];
  uint16_t ethertype;  // Big-Endian 0x88B5
};

/**
 * @brief Lean 8-byte Cell Protocol Common Header.
 */
struct L2CellHeader {
  uint8_t version;            // CELL_NET_L2_VERSION (0x01)
  uint8_t msg_type;           // CellMsgType
  uint8_t sender_id;          // CellNodeId
  uint8_t sequence;           // Rolling sequence counter
  uint32_t timestamp_us_low;  // Microsecond timestamp for time-sync
};

/**
 * @brief Payload for Message Type 0x01: Vision Detection.
 */
struct L2VisionDetectPayload {
  uint32_t item_id;
  float x_across_mm;       // Battery centroid across belt (linear X)
  float y_bat_mm;          // Battery centroid along belt (at capture instant)
  float theta_deg;         // Battery orientation angle (-180..+180 deg)
  uint8_t battery_class;   // 1=18650, 2=21700, 3=Prismatic, 4=Pouch
  uint8_t reserved[3];
};

/**
 * @brief Payload for Message Type 0x02: Conveyor Telemetry.
 */
struct L2ConveyorSpeedPayload {
  float speed_mm_s;         // Real-time belt linear speed
  float displacement_m;     // Cumulative belt travel
  int32_t raw_encoder_cnt;  // 500-PPR hardware PCNT value
};

/**
 * @brief Payload for Message Type 0x03: Gantry Status.
 */
struct L2GantryStatusPayload {
  uint8_t motion_state;      // 0=IDLE, 1=HOMING, 2=INTERCEPTING, 3=GRIPPING, 4=FAULT
  uint8_t active_slot;       // 0=ota_0, 1=ota_1
  uint16_t fault_flags;      // Bitmask of active drive warnings
  float x_pos_mm;            // Current X position
  float z_pos_mm;            // Current Z position
  float theta_deg;           // Current Theta angle
  float last_cycle_time_ms;  // Last completed pick cycle time
};

/**
 * @brief Payload for Message Type 0x04: Cell Supervisor Command.
 */
struct L2CellCommandPayload {
  uint8_t command_id;  // 1=START, 2=STOP, 3=PAUSE, 4=CLEAR_FAULT, 5=PARK
  uint8_t param_u8;
  uint16_t param_u16;
  float param_float;
};

/**
 * @brief Payload for Message Type 0x00: Heartbeat.
 */
struct L2HeartbeatPayload {
  uint32_t uptime_ms;
  uint16_t status_flags;
  uint8_t reserved[2];
};

/**
 * @brief Full Wire Frames.
 */
struct L2HeartbeatFrame {
  L2EthernetHeader eth;
  L2CellHeader cell;
  L2HeartbeatPayload payload;
};

struct L2VisionDetectFrame {
  L2EthernetHeader eth;
  L2CellHeader cell;
  L2VisionDetectPayload payload;
};

struct L2ConveyorSpeedFrame {
  L2EthernetHeader eth;
  L2CellHeader cell;
  L2ConveyorSpeedPayload payload;
};

struct L2GantryStatusFrame {
  L2EthernetHeader eth;
  L2CellHeader cell;
  L2GantryStatusPayload payload;
};

struct L2CellCommandFrame {
  L2EthernetHeader eth;
  L2CellHeader cell;
  L2CellCommandPayload payload;
};

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif  // CELL_NET_L2_PROTOCOL_H
