/*
Copyright (c) 2023, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
*/

//
// Renesas I2C Peripherals -- single jitter cleaner on UL3422
//

RC38612A002GN2 #(
    .DEVICE_ID ( 'hB0 )
) RC38612A002GN2_0 (
    .enable  ( 1'b1         ),
    .sda_io  ( clkgen_sda_r ),
    .scl_io  ( clkgen_scl_r )
);

