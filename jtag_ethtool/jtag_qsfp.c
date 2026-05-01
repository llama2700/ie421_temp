#include <stdio.h>
#include "internal.h"
#include "jtag_qsfp.h"

/* 
* 底层 JTAG 读写函数 (目前仅为占位，后续对接具体 JTAG API)
*/
static void jtag_write_reg(__u32 addr, __u32 val) {
    // TODO: 这里将调用 Vivado Tcl 脚本或 libftdi 库
    printf("[JTAG-WR] Addr: 0x%02x, Val: 0x%02x\n", addr, val);
}

static __u32 jtag_read_reg(__u32 addr) {
    // TODO: 这里将调用 Vivado Tcl 脚本或 libftdi 库
    printf("[JTAG-RD] Addr: 0x%02x\n", addr);
    return 0x40000000; // 返回模拟的完成状态 (Bit30 set)
}

/**
* 对应 qsfp.tcl 的 i2c_wr
*/
int jtag_qsfp_write_byte(int offset, __u8 data) {
    __u8 dev_id = 0xA0; /* QSFP I2C 默认地址 */

    jtag_write_reg(REG_ADDR, offset);
    jtag_write_reg(REG_WDATA, data);
    jtag_write_reg(REG_CONTROL, dev_id); /* RW=0 (Write) */
        
    /* 等待操作完成 */
    __u32 status;
    do {
        status = jtag_read_reg(REG_CONTROL);
    } while (!(status & 0x40000000)); /* 检查 Bit30 (Done) */

    return 0;
}

/**
 * 对应 qsfp.tcl 的 i2c_rd
 */
int jtag_qsfp_read_byte(int offset) {
    __u8 dev_id = 0xA0;

    jtag_write_reg(REG_ADDR, offset);
    jtag_write_reg(REG_CONTROL, dev_id | 0x80000000); /* RW=1 (Read), 置位 Bit31 */
        
    /* 等待操作完成 */
    __u32 status;
    do {
        status = jtag_read_reg(REG_CONTROL);
    } while (!(status & 0x40000000)); /* 检查 Bit30 (Done) */

    /* 读取返回数据 */
    return jtag_read_reg(REG_RDATA) & 0xFF;
}