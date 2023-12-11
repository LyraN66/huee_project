// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none

module user_proj_esc #(
    parameter BITS = 16
)(
`ifdef USE_POWER_PINS
    inout vdd,	// User area 1 1.8V supply
    inout vss,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,

    // IOs
    input  [7:0] io_in,
    output [7:0] io_out,
    output [7:0] io_oeb

);
    wire sda_enable;
    //wire sda_wire;
    assign io_oeb[7:0] = {1'b1,sda_enable,4'b1,2'b0};
    //assign sda_wire = sda_enable ? io_in[6] : io_out[6];
    assign io_out[5:2] = io_in[5:2];
    assign io_out[7] = io_in[7];

    esc_1 ESC_1(
    . clk(wb_clk_i),
    . rst(wb_rst_i | io_in[7]),
    // I2C ports 
    . sda_enable(sda_enable),
    . sda(io_out[6]),
    . scl(io_in[5]),
    // esc ports
    . pwm_en(io_in[4]),
    . encoder_a(io_in[3]),
    . encoder_b(io_in[2]),
    . motor_positive(io_out[1]),
    . motor_negative(io_out[0])
);
endmodule
`default_nettype wire
