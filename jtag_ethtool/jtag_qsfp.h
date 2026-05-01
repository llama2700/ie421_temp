
#ifndef JTAG_QSFP_H
#define JTAG_QSFP_H

#include "internal.h"

/* 寄存器地址映射 (对应 reg_qsfp_i2c_logic.v) */
#define REG_CONTROL   0x00  /* 控制：Bit31=读写方向, Bit30=完成标志, Bit7-0=设备ID */
#define REG_ADDR      0x04  /* I2C 地址 */
#define REG_WDATA     0x08  /* 写入数据 */
#define REG_RDATA     0x0C  /* 读取数据 */
#define REG_RESETB    0x10  /* 复位控制 */

/* 核心读取函数 */
int jtag_qsfp_read_byte(int offset);
int jtag_qsfp_write_byte(int offset, __u8 data);

#endif