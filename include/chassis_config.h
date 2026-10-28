#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* 车头为正常前进方向；地址由用户逐轮点动确认。 */
#define CHASSIS_FRONT_LEFT_ID  4U
#define CHASSIS_FRONT_RIGHT_ID 1U
#define CHASSIS_REAR_LEFT_ID   3U
#define CHASSIS_REAR_RIGHT_ID  2U

/* 四轮 jog 0 时，上沿均朝车尾；故当前四轮前进方向位均为 1。
 * 这是当前安装与驱动器设置的实测配置，不是所有底盘通用的方向。 */
#define CHASSIS_FORWARD_DIRECTION 1U

/* 用户报告“叉连接”，按从正上方看滚轮轴线呈 X 配置。
 * 新增横移和转向组合尚待架空、落地实测，不代表角度/距离已标定。 */
#define CHASSIS_LAYOUT_NAME "X"

_Static_assert(CHASSIS_FRONT_LEFT_ID >= 1U && CHASSIS_FRONT_LEFT_ID <= 4U &&
               CHASSIS_FRONT_RIGHT_ID >= 1U && CHASSIS_FRONT_RIGHT_ID <= 4U &&
               CHASSIS_REAR_LEFT_ID >= 1U && CHASSIS_REAR_LEFT_ID <= 4U &&
               CHASSIS_REAR_RIGHT_ID >= 1U && CHASSIS_REAR_RIGHT_ID <= 4U,
               "Wheel addresses must be within 1..4");
_Static_assert(((1U << CHASSIS_FRONT_LEFT_ID) | (1U << CHASSIS_FRONT_RIGHT_ID) |
                (1U << CHASSIS_REAR_LEFT_ID) | (1U << CHASSIS_REAR_RIGHT_ID)) == 0x1EU,
               "Each wheel must have a different address");
_Static_assert(CHASSIS_FORWARD_DIRECTION <= 1U, "Direction must be 0 or 1");

#endif
