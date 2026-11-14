/* Shared application constants for the standalone LD19 bench F407 firmware. */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#ifndef APP_VERSION
#define APP_VERSION "LD19_BENCH_F407VE_0.3.5_FRONT150"
#endif
#ifndef PROTOCOL_VERSION
#define PROTOCOL_VERSION 2
#endif

#define BOARD_PROFILE          "f407ve_ld19_bench"
#define MCU_NAME               "STM32F407VET6"
#define HSE_HZ                 25000000U
#define SYS_HZ                 168000000U
#define APB1_HZ                42000000U
#define APB2_HZ                84000000U
#define HOST_BAUD              460800U
#define LIDAR_BAUD             230400U
#define INSTALLED_FOV_START_CDEG 19500U
#define INSTALLED_FOV_END_CDEG   34500U

#define LD19_PACKET_SIZE       47U
#define LD19_POINTS_PER_PACKET 12U
#define LD19_HEADER            0x54U
#define LD19_VER_LEN           0x2CU
#define LD19_DEVICE_MS_WRAP    30000U

#define UART2_RX_RING_BYTES    4096U   /* ordinary SRAM, never CCM (DMA) */
#define UART1_TX_RING_BYTES    4096U
#define UART1_RX_RING_BYTES    512U
#define RAW_CHUNK_MAX          240U
#define RAW_CHUNK_FLUSH_US     10000U
#define SCAN_MAX_POINTS        1200U
#define FEATURE_MAX            16U
#define PB_PAYLOAD_MAX         250U
#define PB_FRAME_MAX           (6U + PB_PAYLOAD_MAX) /* AA 55 CMD SEQ LEN + payload + XOR */

/* Capability bits (must match shared/bench_protocol_v1.json). */
#define CAP_RAW_STATUS         0x01U
#define CAP_FEATURES           0x02U
#define CAP_STORAGE_INFO       0x04U
#define CAP_PWM                0x08U
#define CAP_ONBOARD_LOG        0x10U
#define CAP_DESKEW             0x20U
#define CAP_TRACKS             0x40U
#define CAP_SHADOW             0x80U
#define APP_CAPS               (CAP_RAW_STATUS | CAP_FEATURES | CAP_STORAGE_INFO | CAP_DESKEW | CAP_TRACKS | CAP_SHADOW)

/* STATUS flags. */
#define ST_FLAG_NO_PACKET_500MS  0x0001U
#define ST_FLAG_DISCONTINUOUS    0x0002U
#define ST_FLAG_PARTIAL_SCAN     0x0004U
#define ST_FLAG_EXTRINSICS_VALID 0x0008U
#define ST_FLAG_POSE_VALID       0x0010U
#define ST_FLAG_DESKEW           0x0020U

static inline uint32_t app_us_diff(uint32_t a, uint32_t b)
{
    return (uint32_t)(a - b); /* unsigned wrap-safe difference */
}

#endif
