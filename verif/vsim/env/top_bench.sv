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

`timescale 1 ns / 1 ps

/*
`define UNIT_DELAY         #1
`define USE_POWER_PINS
`define SIM_TIME           100_000

`include "libs.ref/sky130_fd_sc_hd/verilog/primitives.v"
`include "libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v"

`include "libs.ref/sky130_fd_sc_hvl/verilog/primitives.v"
`include "libs.ref/sky130_fd_sc_hvl/verilog/sky130_fd_sc_hvl.v"

`include "libs.ref/sky130_fd_sc_hd/verilog/primitives.v"
`include "libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v"

`include "libs.ref/sky130_fd_sc_hvl/verilog/primitives.v"
`include "libs.ref/sky130_fd_sc_hvl/verilog/sky130_fd_sc_hvl.v"

`include "libs.ref/sky130_fd_io/verilog/sky130_fd_io.v"
`include "libs.ref/sky130_fd_io/verilog/sky130_ef_io.v"
`include "libs.ref/sky130_fd_io/verilog/sky130_ef_io__gpiov2_pad_wrapped.v"

*/

// `include "defines.v
// `include "__uprj_netlists.v"
// `include "caravel_netlists.v"
// `include "spiflash.v"

// ------------------------------------------------------------------------------

`include "project_define.svh"

module top_bench;

  `include "bench_ini.svh"

  wire        gpio;
  wire [37:0] mprj_io;
  wire        flash_csb;
  wire        flash_clk;
  wire        flash_io0;
  wire        flash_io1;
  wire        SDO;


  wire [15:0] checkbits;
  assign      checkbits = mprj_io[31:16];

  wire  [7:0] spivalue;
  assign      spivalue  = mprj_io[15: 8];

  reg         power1;  // 3.3V
  reg         power2;  // 1.8V

  // External clock is used by default.  Make this artificially fast for the
  // simulation.  Normally this would be a slow clock and the digital PLL
  // would be the fast clock.
  //
  reg clock;
  reg io_clk;   // generated from FPGA in real system

  localparam pSOC_FREQ = 10.0;                 // 10 MHz
  localparam pIOS_FREQ = (pSOC_FREQ * 4.0);    // 40 MHz

  // Timing Order
  // POWER ==> CLOCK ==> RESET

  initial begin
    clock  = 0;
    wait(power2);
    #100;
    forever begin
      #(500.0 / pSOC_FREQ);
      clock = ~clock;
    end
  end

  initial begin
    io_clk  = 0;
    wait(power2);
    #100;
    forever begin
      #(500.0 / pIOS_FREQ);
      io_clk = ~io_clk;
    end
  end

  wire [11:0] rx_dat;
  wire        rx_clk;
  // TBD
  assign #1 rx_clk = io_clk;
  // TBD
  assign #2 rx_dat = 12'h000;

  assign mprj_io[   37] = io_clk;
  assign mprj_io[19: 8] = rx_dat;
  assign mprj_io[   20] = rx_clk;
 

  initial begin
    $timeformat (-9, 3, " ns", 13);
  //$dumpfile("top_bench.vcd");
  //$dumpvars(0, top_bench);

    repeat (250) begin
      repeat (1000) @(posedge clock);
    //$display("+1000 cycles");
    end

    $display("%c[1;31m",27);
    $display ("Monitor: Timeout, Test Failed");
    $display("%c[0m",27);
    $finish;
  end

  // ------------------------------------------------------------
  /*
 
  // User clock monitoring
  integer ucount;
  always @(posedge mprj_io[15]) begin
      ucount = ucount + 1;
  end


  // Core clock monitoring
  integer ccount;
  always @(posedge mprj_io[14]) begin
      ccount = ccount + 1;
  end


  // Monitor
  initial begin
      wait(checkbits == 16'hA040);
      $display("Monitor: Test 1 PLL (RTL) Started");
      ucount = 0;
      ccount = 0;
      wait(checkbits == 16'hA041);
      $display("Monitor: ucount = %d ccount = %d", ucount, ccount);
            if (ucount !== 129 || ccount != 129) begin
                $display("Monitor: Test PLL Failed");
                $finish;
            end
    
      wait(checkbits == 16'hA042);
      $display("Monitor: Test 2 PLL (RTL) Started");
      ucount = 0;
      ccount = 0;
      wait(checkbits == 16'hA043);
      $display("Monitor: ucount = %d ccount = %d", ucount, ccount);
            if (ucount !== 193 || ccount != 193) begin
                $display("Monitor: Test PLL Failed");
                $finish;
            end

      wait(checkbits == 16'hA044);
      $display("Monitor: Test 3 PLL (RTL) Started");
      ucount = 0;
      ccount = 0;
      wait(checkbits == 16'hA045);
      $display("Monitor: ucount = %d ccount = %d", ucount, ccount);
            if (ucount !== 385 || ccount != 129) begin
                $display("Monitor: Test PLL Failed");
                $finish;
            end

      wait(checkbits == 16'hA046);
      $display("Monitor: Test 4 PLL (RTL) Started");
      ucount = 0;
      ccount = 0;
      wait(checkbits == 16'hA047);
      $display("Monitor: ucount = %d ccount = %d", ucount, ccount);
            if (ucount !== 385 || ccount != 129) begin
                $display("Monitor: Test PLL Failed");
                $finish;
            end

      wait(checkbits == 16'hA048);
      $display("Monitor: Test 5 PLL (RTL) Started");
      ucount = 0;
      ccount = 0;
      wait(checkbits == 16'hA049);
      $display("Monitor: ucount = %d ccount = %d", ucount, ccount);
            if (ucount !== 513 || ccount != 129) begin
                $display("Monitor: Test PLL Failed");
                $finish;
            end

      wait(checkbits == 16'hA090);

      $display("Monitor: Test PLL (RTL) Passed");
      $finish;
  end
  */
  reg RSTB;

  initial begin
    RSTB <= 1'b0;
    wait(power2);
    #400;
    $display("%t MSG %m, Chip Reset# is released ", $time);
    RSTB <= 1'b1;      // Release reset
    #2000;
  end

  initial begin
    power1 <= 1'b0;
    power2 <= 1'b0;
    #200;
    power1 <= 1'b1;
    #200;
    power2 <= 1'b1;
  end

  always @(checkbits) begin
    #1 $display("GPIO state = %b ", checkbits);
  end

  wire VDD3V3;
  wire VDD1V8;
  wire VSS;
  
  assign VDD3V3 = power1;
  assign VDD1V8 = power2;
  assign VSS    = 1'b0;

  // These are the mappings of mprj_io GPIO pads that are set to
  // specific functions on startup:
  //
  // JTAG      = mgmt_gpio_io[0]              (inout)
  // SDO       = mgmt_gpio_io[1]              (output)
  // SDI       = mgmt_gpio_io[2]              (input)
  // CSB       = mgmt_gpio_io[3]              (input)
  // SCK       = mgmt_gpio_io[4]              (input)
  // ser_rx    = mgmt_gpio_io[5]              (input)
  // ser_tx    = mgmt_gpio_io[6]              (output)
  // irq       = mgmt_gpio_io[7]              (input)


  // move to bench_vec.svh
  // assign mprj_io[3] = 1'b1;  // Force CSB high.

  caravel uut (.vddio     (VDD3V3),
               .vddio_2   (VDD3V3),
               .vssio     (VSS),
               .vssio_2   (VSS),
               .vdda      (VDD3V3),
               .vssa      (VSS),
               .vccd      (VDD1V8),
               .vssd      (VSS),
               .vdda1     (VDD3V3),
               .vdda1_2   (VDD3V3),
               .vdda2     (VDD3V3),
               .vssa1     (VSS),
               .vssa1_2   (VSS),
               .vssa2     (VSS),
               .vccd1     (VDD1V8),
               .vccd2     (VDD1V8),
               .vssd1     (VSS),
               .vssd2     (VSS),
               .clock     (clock),
               .gpio      (gpio),
               .mprj_io   (mprj_io),
               .flash_csb (flash_csb),
               .flash_clk (flash_clk),
               .flash_io0 (flash_io0),
               .flash_io1 (flash_io1),
               .resetb    (RSTB) );


  spiflash #(.FILENAME("riscv.hex")) spiflash ( .csb(flash_csb),
                                              .clk(flash_clk),
                                              .io0(flash_io0),
                                              .io1(flash_io1),
                                              .io2(),          // not used
                                              .io3() );        // not used

  wire   uart_tx;
  assign uart_tx = mprj_io[6];

  tbuart tbuart (.ser_rx(uart_tx));  // I

  // -------------------------------------------------------
  `include "bench_vec.svh"

  // -------------------------------------------------------
  `ifdef    CPU_TRACE
  `include "cpu_trace.v"
  `include "dasm.v"
  `endif // CPU_TRACE

endmodule // top_bench

`default_nettype wire
