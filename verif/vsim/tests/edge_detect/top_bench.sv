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

//`define USE_EDGEDETECT_IP 1
// --------------------------------------------------------
// define only one of below items
// --------------------------------------------------------
//`define SYSTEM_test111 1
//`define SYSTEM_test112 1
//`define SYSTEM_test103 1
//`define SYSTEM_test104 1
//`define SYSTEM_test113 1
//`define SYSTEM_test114 1


// `include "defines.v
// `include "__uprj_netlists.v"
// `include "caravel_netlists.v"
// `include "spiflash.v"

// ------------------------------------------------------------------------------

`include "project_define.svh"

module top_bench #( parameter BITS=32,
        parameter pUSER_PROJECT_SIDEBAND_WIDTH   = 5,
        `ifdef USER_PROJECT_SIDEBAND_SUPPORT
            parameter pSERIALIO_WIDTH   = 13,
        `else
            parameter pSERIALIO_WIDTH   = 12,
        `endif
		parameter pADDR_WIDTH   = 15,
		parameter pDATA_WIDTH   = 32,
		parameter IOCLK_Period	= 10,
		// parameter DLYCLK_Period	= 1,
		parameter SHIFT_DEPTH = 5,
		parameter pRxFIFO_DEPTH = 5,
		parameter pCLK_RATIO = 4
	)
(
);
  `include "bench_ini.svh"
  `ifdef USE_EDGEDETECT_IP
    localparam TST_TOTAL_FRAME_NUM = 2;
    localparam TST_FRAME_WIDTH     = 640;
    localparam TST_FRAME_HEIGHT    = 360;
    localparam TST_TOTAL_PIXEL_NUM = TST_FRAME_WIDTH * TST_FRAME_HEIGHT;
    localparam TST_SW              = 1;
  `endif //USE_EDGEDETECT_IP
	localparam CoreClkPhaseLoop	= 1;
	localparam UP_BASE=32'h3000_0000;
	localparam AA_BASE=32'h3000_2000;
	localparam IS_BASE=32'h3000_3000;

	localparam SOC_to_FPGA_MailBox_Base=28'h000_2000;
	localparam FPGA_to_SOC_UP_BASE=28'h000_0000;
	localparam FPGA_to_SOC_AA_BASE=28'h000_2000;
	localparam FPGA_to_SOC_IS_BASE=28'h000_3000;

	localparam AA_MailBox_Reg_Offset=12'h000;
	localparam AA_Internal_Reg_Offset=12'h100;

	localparam TUSER_AXIS = 2'b00;
	localparam TUSER_AXILITE_WRITE = 2'b01;
	localparam TUSER_AXILITE_READ_REQ = 2'b10;
	localparam TUSER_AXILITE_READ_CPL = 2'b11;

	localparam TID_DN_UP = 2'b00;
	localparam TID_DN_AA = 2'b01;
	localparam TID_UP_UP = 2'b00;
	localparam TID_UP_AA = 2'b01;
	localparam TID_UP_LA = 2'b10;

  `ifdef USE_EDGEDETECT_IP
    localparam fpga_axis_test_length = TST_TOTAL_PIXEL_NUM / 4; //each pixel is 8 bits //each transaction
  `else //USE_EDGEDETECT_IP
    localparam fpga_axis_test_length = 16;
  `endif //USE_EDGEDETECT_IP

  localparam BASE_OFFSET = 8;
  localparam RXD_OFFSET = BASE_OFFSET;
  localparam RXCLK_OFFSET = RXD_OFFSET + pSERIALIO_WIDTH;
  localparam TXD_OFFSET = RXCLK_OFFSET + 1;
  localparam TXCLK_OFFSET = TXD_OFFSET + pSERIALIO_WIDTH;
  localparam IOCLK_OFFSET = TXCLK_OFFSET + 1;
  localparam TXRX_WIDTH = IOCLK_OFFSET - BASE_OFFSET + 1;

  wire        gpio;
  wire [37:0] mprj_io;
  wire        flash_csb;
  wire        flash_clk;
  wire        flash_io0;
  wire        flash_io1;
  wire        SDO;

	reg fpga_rst;
	//reg soc_resetb;		//POR reset
	reg fpga_resetb;	//POR reset
	wire fpga_coreclk;

	//write addr channel
	reg fpga_axi_awvalid;
	reg [pADDR_WIDTH-1:0] fpga_axi_awaddr;
	wire fpga_axi_awready;

	//write data channel
	reg 	fpga_axi_wvalid;
	reg 	[pDATA_WIDTH-1:0] fpga_axi_wdata;
	reg 	[3:0] fpga_axi_wstrb;
	wire	fpga_axi_wready;

	//read addr channel
	reg 	fpga_axi_arvalid;
	reg 	[pADDR_WIDTH-1:0] fpga_axi_araddr;
	wire 	fpga_axi_arready;

	//read data channel
	wire 	fpga_axi_rvalid;
	wire 	[pDATA_WIDTH-1:0] fpga_axi_rdata;
	reg 	fpga_axi_rready;

	reg 	fpga_cc_is_enable;		//axi_lite enable

	wire [pSERIALIO_WIDTH-1:0] soc_serial_txd;
	wire soc_txclk;
	wire fpga_txclk;

	reg [pDATA_WIDTH-1:0] fpga_as_is_tdata;
  `ifdef USER_PROJECT_SIDEBAND_SUPPORT
    reg [pUSER_PROJECT_SIDEBAND_WIDTH-1:0] fpga_as_is_tupsb;
  `endif
	reg [3:0] fpga_as_is_tstrb;
	reg [3:0] fpga_as_is_tkeep;
	reg fpga_as_is_tlast;
	reg [1:0] fpga_as_is_tid;
	reg fpga_as_is_tvalid;
	reg [1:0] fpga_as_is_tuser;
	reg fpga_as_is_tready;		//when local side axis switch Rxfifo size <= threshold then as_is_tready=0; this flow control mechanism is for notify remote side do not provide data with is_as_tvalid=1

	wire [pSERIALIO_WIDTH-1:0] fpga_serial_txd;
//	wire [7:0] fpga_Serial_Data_Out_tdata;
//	wire fpga_Serial_Data_Out_tstrb;
//	wire fpga_Serial_Data_Out_tkeep;
//	wire fpga_Serial_Data_Out_tid_tuser;	// tid and tuser
//	wire fpga_Serial_Data_Out_tlast_tvalid_tready;		//flowcontrol

	wire [pDATA_WIDTH-1:0] fpga_is_as_tdata;
  `ifdef USER_PROJECT_SIDEBAND_SUPPORT
    wire [pUSER_PROJECT_SIDEBAND_WIDTH-1:0] fpga_is_as_tupsb;
  `endif
	wire [3:0] fpga_is_as_tstrb;
	wire [3:0] fpga_is_as_tkeep;
	wire fpga_is_as_tlast;
	wire [1:0] fpga_is_as_tid;
	wire fpga_is_as_tvalid;
	wire [1:0] fpga_is_as_tuser;
	wire fpga_is_as_tready;		//when remote side axis switch Rxfifo size <= threshold then is_as_tready=0, this flow control mechanism is for notify local side do not provide data with as_is_tvalid=1

  reg [27:0] fpga_axilite_write_addr;

	reg[27:0] soc_to_fpga_mailbox_write_addr_expect_value;
	reg[3:0] soc_to_fpga_mailbox_write_addr_BE_expect_value;
	reg[31:0] soc_to_fpga_mailbox_write_data_expect_value;
	reg [31:0] soc_to_fpga_mailbox_write_addr_captured;
	reg [31:0] soc_to_fpga_mailbox_write_data_captured;
	event soc_to_fpga_mailbox_write_event;
  reg stream_data_addr_or_data; //0: address, 1: data, use to identify the write transaction from AA.

	reg [31:0] soc_to_fpga_axilite_read_cpl_expect_value;
	reg [31:0] soc_to_fpga_axilite_read_cpl_captured;
	event soc_to_fpga_axilite_read_cpl_event;

  `ifdef USE_EDGEDETECT_IP

    reg [31:0] soc_to_fpga_axis_expect_count;
    `ifdef USER_PROJECT_SIDEBAND_SUPPORT
      reg [(pUSER_PROJECT_SIDEBAND_WIDTH+4+4+1+32-1):0] soc_to_fpga_axis_expect_value[fpga_axis_test_length-1:0];
    `else //USER_PROJECT_SIDEBAND_SUPPORT
      reg [(4+4+1+32-1):0] soc_to_fpga_axis_expect_value[fpga_axis_test_length-1:0];
    `endif //USER_PROJECT_SIDEBAND_SUPPORT

    reg [31:0] soc_to_fpga_axis_captured_count;
    `ifdef USER_PROJECT_SIDEBAND_SUPPORT
      reg [(pUSER_PROJECT_SIDEBAND_WIDTH+4+4+1+32-1):0] soc_to_fpga_axis_captured[fpga_axis_test_length-1:0];
    `else //USER_PROJECT_SIDEBAND_SUPPORT
      reg [(4+4+1+32-1):0] soc_to_fpga_axis_captured[fpga_axis_test_length-1:0];
    `endif  //USER_PROJECT_SIDEBAND_SUPPORT

  `else //USE_EDGEDETECT_IP

    reg [6:0] soc_to_fpga_axis_expect_count;
    `ifdef USER_PROJECT_SIDEBAND_SUPPORT
      reg [(pUSER_PROJECT_SIDEBAND_WIDTH+4+4+1+32-1):0] soc_to_fpga_axis_expect_value[127:0];
    `else //USER_PROJECT_SIDEBAND_SUPPORT
      reg [(4+4+1+32-1):0] soc_to_fpga_axis_expect_value[127:0];
    `endif  //USER_PROJECT_SIDEBAND_SUPPORT

    reg [6:0] soc_to_fpga_axis_captured_count;
    `ifdef USER_PROJECT_SIDEBAND_SUPPORT
      reg [(pUSER_PROJECT_SIDEBAND_WIDTH+4+4+1+32-1):0] soc_to_fpga_axis_captured[127:0];
    `else //USER_PROJECT_SIDEBAND_SUPPORT
      reg [(4+4+1+32-1):0] soc_to_fpga_axis_captured[127:0];
    `endif //USER_PROJECT_SIDEBAND_SUPPORT

  `endif //USE_EDGEDETECT_IP

  event soc_to_fpga_axis_event;
	
  reg [31:0] error_cnt;
	reg [31:0] check_cnt;
  reg finish_flag;

  wire [11:0] checkbits;
  assign      checkbits = uut.mprj_io_out[32:21];

  //wire  [7:0] spivalue;
  //assign      spivalue  = mprj_io[15: 8];

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

// MPRJ_IO PIN PLANNING when pSERIALIO_WIDTH=13
// --------------------------------
// [20: 8]  I   RXD
// [   21]  I   RXCLK

// --------------------------------
// [34:22]  O   TXD
// [   35]  O   TXCLK

// --------------------------------
// [   36]  I   IO_CLK

// MPRJ_IO PIN PLANNING when pSERIALIO_WIDTH=12
// --------------------------------
// [19: 8]  I   RXD
// [   20]  I   RXCLK

// --------------------------------
// [32:21]  O   TXD
// [   33]  O   TXCLK

// --------------------------------
// [   34]  I   IO_CLK

    assign mprj_io[IOCLK_OFFSET] = io_clk;
    assign mprj_io[RXCLK_OFFSET] = fpga_txclk;
    assign mprj_io[RXD_OFFSET +: pSERIALIO_WIDTH] = fpga_serial_txd;

    assign soc_txclk = mprj_io[TXCLK_OFFSET];
    assign soc_serial_txd = mprj_io[TXD_OFFSET +: pSERIALIO_WIDTH];


   initial begin

    `ifdef USE_EDGEDETECT_IP
      run_edge_detect();
    `endif //USE_EDGEDETECT_IP

    `ifdef SYSTEM_test111
    test111();
    `endif //SYSTEM_test111

    `ifdef SYSTEM_test112
    test112();
    `endif //SYSTEM_test112

    `ifdef SYSTEM_test103
    test103();
    `endif //SYSTEM_test103

    `ifdef SYSTEM_test104
    test104();
    `endif //SYSTEM_test104


    `ifdef SYSTEM_test113
    test113();
    `endif //SYSTEM_test113

    `ifdef SYSTEM_test114
    test114();
    `endif //SYSTEM_test114

    end

  wire ioclk;
  assign ioclk = io_clk;

  reg [31:0] repeat_cnt;

  initial begin
    $timeformat (-9, 3, " ns", 13);
  //$dumpfile("top_bench.vcd");
  //$dumpvars(0, top_bench);
    error_cnt = 0;
    check_cnt = 0;
    finish_flag = 0;
    repeat_cnt = 0;


    do begin
        repeat_cnt = repeat_cnt + 1;
        repeat (1000) @(posedge clock);
        $display("%t MSG %m, +1000 cycles, finish_flag=%b,  repeat_cnt=%04d", $time, finish_flag, repeat_cnt);
    end
    while(finish_flag == 0 && repeat_cnt <= 200 );


		$display("=============================================================================================");
		$display("=============================================================================================");
		$display("=============================================================================================");
		if (error_cnt != 0 ) begin
      $display("%c[1;31m",27);
			$display($time, "=> Final result [FAILED], check_cnt = %d, error_cnt = %d, please search [ERROR] in the log", check_cnt, error_cnt);
      $display("%c[0m",27);
    end
		else begin
			$display($time, "=> Final result [PASS], check_cnt = %d, error_cnt = %04d", check_cnt, error_cnt);
    end
		$display("=============================================================================================");
		$display("=============================================================================================");
		$display("=============================================================================================");

    //$display("%c[1;31m",27);
    //$display ("Monitor: Timeout, Test Failed");
    //$display("%c[0m",27);
    $finish;
  end

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

/*
  always @(checkbits) begin
    //#1 $display("GPIO state = %b ", checkbits);
    #1 $display("%t IOCLK = %b, TX_CLK=%b, TXD=%b, RX_CLK=%b, RXD=%b,  ", $time, uut.mprj_io_in[37], uut.mprj_io_out[33], uut.mprj_io_out[32:21], uut.mprj_io_in[20], uut.mprj_io_in[19:8]);
  end
*/


	initial begin		//when soc cfg write to AA, then AA in soc generate soc_to_fpga_mailbox_write,
    stream_data_addr_or_data = 0;
		while (1) begin
			@(posedge fpga_coreclk);
			//New AA version, all stream data with last = 1.
      if (fpga_is_as_tvalid == 1 && fpga_is_as_tid == TID_UP_AA && fpga_is_as_tuser == TUSER_AXILITE_WRITE && fpga_is_as_tlast == 1) begin
        if(stream_data_addr_or_data == 1'b0) begin
            //Address
            $display($time, "=> get soc_to_fpga_mailbox_write_addr_captured be : soc_to_fpga_mailbox_write_addr_captured =%x, fpga_is_as_tdata=%x", soc_to_fpga_mailbox_write_addr_captured, fpga_is_as_tdata);
            soc_to_fpga_mailbox_write_addr_captured = fpga_is_as_tdata ;		//use block assignment
            $display($time, "=> get soc_to_fpga_mailbox_write_addr_captured af : soc_to_fpga_mailbox_write_addr_captured =%x, fpga_is_as_tdata=%x", soc_to_fpga_mailbox_write_addr_captured, fpga_is_as_tdata);
            //Next should be data
            stream_data_addr_or_data = 1;
        end else begin
            //Data
            $display($time, "=> get soc_to_fpga_mailbox_write_data_captured be : soc_to_fpga_mailbox_write_data_captured =%x, fpga_is_as_tdata=%x", soc_to_fpga_mailbox_write_data_captured, fpga_is_as_tdata);
            soc_to_fpga_mailbox_write_data_captured = fpga_is_as_tdata ;		//use block assignment
            $display($time, "=> get soc_to_fpga_mailbox_write_data_captured af : soc_to_fpga_mailbox_write_data_captured =%x, fpga_is_as_tdata=%x", soc_to_fpga_mailbox_write_data_captured, fpga_is_as_tdata);
            #0 -> soc_to_fpga_mailbox_write_event;
            $display($time, "=> soc_to_fpga_mailbox_write_data_captured : send soc_to_fpga_mailbox_write_event");
            //Next should be address
            stream_data_addr_or_data = 0;
        end

			end
		end
	end



  `ifdef SYSTEM_test111
  reg [31:0] idx5;
  initial begin

    // test111 - for soc CFG write to mailbox
    // 1. [FW] SOC CFG write to mailbox
    // 1.A [testbech] check soc_to_fpga_mailbox_write

    for (idx5 = 0 ; idx5 < 32'h10 ; idx5=idx5+4) begin
      soc_to_fpga_mailbox_write_addr_expect_value = 28'h000_2000 + idx5;
      soc_to_fpga_mailbox_write_addr_BE_expect_value = 4'b1111;
      soc_to_fpga_mailbox_write_data_expect_value = 32'h5a5a_5a5a;

      wait_and_check_soc_to_fpga_mailbox_write_event();
      soc_to_fpga_mailbox_write_data_expect_value = 32'ha5a5_a5a5;
      wait_and_check_soc_to_fpga_mailbox_write_event();
    end
    finish_flag = 1;

  end
  `endif //SYSTEM_test111

  `ifdef SYSTEM_test112
    // test112 - for soc CFG read and write to mailbox then send to fpga
    // 1. [FW] SOC CFG read and write to mailbox
    // 1.A [testbech] check soc_to_fpga_mailbox_write

    //for SOC internal register read/write test and update the result to mailbox, testbench check the data when received the mail box write.
    initial begin		//when soc cfg write to AA, then AA in soc generate soc_to_fpga_mailbox_write,

    soc_to_fpga_mailbox_write_addr_expect_value = 28'h000_2000;
    soc_to_fpga_mailbox_write_addr_BE_expect_value = 4'b1111;
    soc_to_fpga_mailbox_write_data_expect_value = 32'h0;
    wait_and_check_soc_to_fpga_mailbox_write_event();
    soc_to_fpga_mailbox_write_data_expect_value = 32'h1;
    wait_and_check_soc_to_fpga_mailbox_write_event();
    soc_to_fpga_mailbox_write_data_expect_value = 32'h3;
    wait_and_check_soc_to_fpga_mailbox_write_event();

    finish_flag = 1;
    end
  `endif //SYSTEM_test112


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
	fsic_clock_div fpga_clock_div (
	.resetb(fpga_resetb),
	.in(ioclk),
	.out(fpga_coreclk)
	);

	fpga  #(
		.pUSER_PROJECT_SIDEBAND_WIDTH(pUSER_PROJECT_SIDEBAND_WIDTH),
		.pSERIALIO_WIDTH(pSERIALIO_WIDTH),
		.pADDR_WIDTH(pADDR_WIDTH),
		.pDATA_WIDTH(pDATA_WIDTH),
		.pRxFIFO_DEPTH(pRxFIFO_DEPTH),
		.pCLK_RATIO(pCLK_RATIO)
	)
	fpga_fsic(
		.axis_rst_n(~fpga_rst),
		.axi_reset_n(~fpga_rst),
		.serial_tclk(fpga_txclk),
		.serial_rclk(soc_txclk),
		.ioclk(ioclk),
		.axis_clk(fpga_coreclk),
		.axi_clk(fpga_coreclk),

		//write addr channel
		.axi_awvalid_s_awvalid(fpga_axi_awvalid),
		.axi_awaddr_s_awaddr(fpga_axi_awaddr),
		.axi_awready_axi_awready3(fpga_axi_awready),

		//write data channel
		.axi_wvalid_s_wvalid(fpga_axi_wvalid),
		.axi_wdata_s_wdata(fpga_axi_wdata),
		.axi_wstrb_s_wstrb(fpga_axi_wstrb),
		.axi_wready_axi_wready3(fpga_axi_wready),

		//read addr channel
		.axi_arvalid_s_arvalid(fpga_axi_arvalid),
		.axi_araddr_s_araddr(fpga_axi_araddr),
		.axi_arready_axi_arready3(fpga_axi_arready),

		//read data channel
		.axi_rvalid_axi_rvalid3(fpga_axi_rvalid),
		.axi_rdata_axi_rdata3(fpga_axi_rdata),
		.axi_rready_s_rready(fpga_axi_rready),

		.cc_is_enable(fpga_cc_is_enable),


		.as_is_tdata(fpga_as_is_tdata),
    `ifdef USER_PROJECT_SIDEBAND_SUPPORT
      .as_is_tupsb  (fpga_as_is_tupsb),
    `endif
		.as_is_tstrb(fpga_as_is_tstrb),
		.as_is_tkeep(fpga_as_is_tkeep),
		.as_is_tlast(fpga_as_is_tlast),
		.as_is_tid(fpga_as_is_tid),
		.as_is_tvalid(fpga_as_is_tvalid),
		.as_is_tuser(fpga_as_is_tuser),
		.as_is_tready(fpga_as_is_tready),
		.serial_txd(fpga_serial_txd),
		.serial_rxd(soc_serial_txd),
		.is_as_tdata(fpga_is_as_tdata),
    `ifdef USER_PROJECT_SIDEBAND_SUPPORT
      .is_as_tupsb  (fpga_is_as_tupsb),
    `endif
		.is_as_tstrb(fpga_is_as_tstrb),
		.is_as_tkeep(fpga_is_as_tkeep),
		.is_as_tlast(fpga_is_as_tlast),
		.is_as_tid(fpga_is_as_tid),
		.is_as_tvalid(fpga_is_as_tvalid),
		.is_as_tuser(fpga_is_as_tuser),
		.is_as_tready(fpga_is_as_tready)
	);

  reg [31:0] i;

	task fpga_apply_reset;
		input real delta1;		// for POR De-Assert
		input real delta2;		// for reset De-Assert
		begin
			#(40);
			$display($time, "=> fpga POR Assert");
			fpga_resetb = 0;
			$display($time, "=> fpga reset Assert");
			fpga_rst = 1;
			#(delta1);

			$display($time, "=> fpga POR De-Assert");
			fpga_resetb = 1;

			#(delta2);
			$display($time, "=> fpga reset De-Assert");
			fpga_rst = 0;
		end
	endtask

  task init_fpga_as;
    begin
				#40;

				fpga_as_to_is_init();

				//soc_cc_is_enable=1;
				fpga_cc_is_enable=1;

				#400;
				$display($time, "=> wait uut.mprj.u_fsic.U_IO_SERDES0.rxen");
        wait(uut.mprj.u_fsic.U_IO_SERDES0.rxen);
				$display($time, "=> detect uut.mprj.u_fsic.U_IO_SERDES0.rxen=1");

				fpga_cfg_write(0,1,1,0);
				$display($time, "=> fpga rxen_ctl=1");

				repeat(4) @(posedge fpga_coreclk);
				fork
					//soc_is_cfg_write(0, 4'b0001, 3);				//ioserdes txen
					fpga_cfg_write(0,3,1,0);
				join
				//$display($time, "=> soc txen_ctl=1");
				$display($time, "=> fpga txen_ctl=1");

				#200;
				fpga_as_is_tdata = 32'h5a5a5a5a;
        `ifdef USER_PROJECT_SIDEBAND_SUPPORT
          fpga_as_is_tupsb = 5'h00;
        `endif
				#40;
				#200;
    end
	endtask





	
	task run_edge_detect;		//

		begin
      run_edge_detect_test001();
      run_edge_detect_test002();      
      finish_flag=1;
		end
	endtask


	task run_edge_detect_test001;   
    // 1. [FW] soc CFG write to internal register - target to REG_IS_BASE for rxen and txen
    // 2. [testbech] init_fpga_as() - wait for soc rxen then set FPGA rxen and txen
    // 3. [testbech] init edge detect register in user project with cfg read/write test
		begin
			for (i=0;i<CoreClkPhaseLoop;i=i+1) begin
				$display("run_edge_detect_test001: loop %02d", i);
				fork
					//soc_apply_reset(40+i*10, 40);			//change coreclk phase in soc
					fpga_apply_reset(40,40);		//fix coreclk phase in fpga
				join
        init_fpga_as();
				run_edge_detect_fpga_to_soc_CFG_access();		//target to FPGA_to_SOC_UP_BASE
				#200;
			end
		end

	endtask


	task run_edge_detect_fpga_to_soc_CFG_access;
		//FPGA to SOC Axilite test
		begin
			@ (posedge fpga_coreclk);
			fpga_as_is_tready <= 1;

			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access start");
      //step 1. set rst = 1
			//fpga issue cfg write request to soc
      fpga_axilite_write_addr = FPGA_to_SOC_UP_BASE + 'h0; 
      soc_to_fpga_axilite_read_cpl_expect_value = 32'h1;
			fpga_axilite_write(fpga_axilite_write_addr, 4'b0001, soc_to_fpga_axilite_read_cpl_expect_value);

			//fpga issue cfg read request to soc
			fpga_axilite_read_req(fpga_axilite_write_addr);
			//fpga wait for read completion from soc
			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access :wait for soc_to_fpga_axilite_read_cpl_event");
			@(soc_to_fpga_axilite_read_cpl_event);		//wait for fpga get the read cpl.
			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access : got soc_to_fpga_axilite_read_cpl_event");

			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access : soc_to_fpga_axilite_read_cpl_captured=%x", soc_to_fpga_axilite_read_cpl_captured);

			//Data part
      check_cnt = check_cnt + 1;
			if ( soc_to_fpga_axilite_read_cpl_expect_value !== soc_to_fpga_axilite_read_cpl_captured) begin
				$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access [ERROR] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
				error_cnt = error_cnt + 1;
			end
			else
				$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access [PASS] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);

      //step 2. set rst = 0
			//fpga issue cfg write request to soc
      fpga_axilite_write_addr = FPGA_to_SOC_UP_BASE + 'h0; 
      soc_to_fpga_axilite_read_cpl_expect_value = 32'h0;
			fpga_axilite_write(fpga_axilite_write_addr, 4'b0001, soc_to_fpga_axilite_read_cpl_expect_value);

			//fpga issue cfg read request to soc
			fpga_axilite_read_req(fpga_axilite_write_addr);
			//fpga wait for read completion from soc
			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access :wait for soc_to_fpga_axilite_read_cpl_event");
			@(soc_to_fpga_axilite_read_cpl_event);		//wait for fpga get the read cpl.
			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access : got soc_to_fpga_axilite_read_cpl_event");

			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access : soc_to_fpga_axilite_read_cpl_captured=%x", soc_to_fpga_axilite_read_cpl_captured);

			//Data part
      check_cnt = check_cnt + 1;
			if ( soc_to_fpga_axilite_read_cpl_expect_value !== soc_to_fpga_axilite_read_cpl_captured) begin
				$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access [ERROR] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
				error_cnt = error_cnt + 1;
			end
			else
				$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access [PASS] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);

      //step 3. set widthIn
			//fpga issue cfg write request to soc
      fpga_axilite_write_addr = FPGA_to_SOC_UP_BASE + 'h4; 
      soc_to_fpga_axilite_read_cpl_expect_value = TST_FRAME_WIDTH;
			fpga_axilite_write(fpga_axilite_write_addr, 4'b0111, soc_to_fpga_axilite_read_cpl_expect_value);

			//fpga issue cfg read request to soc
			fpga_axilite_read_req(fpga_axilite_write_addr);
			//fpga wait for read completion from soc
			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access :wait for soc_to_fpga_axilite_read_cpl_event");
			@(soc_to_fpga_axilite_read_cpl_event);		//wait for fpga get the read cpl.
			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access : got soc_to_fpga_axilite_read_cpl_event");

			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access : soc_to_fpga_axilite_read_cpl_captured=%x", soc_to_fpga_axilite_read_cpl_captured);

			//Data part
      check_cnt = check_cnt + 1;
			if ( soc_to_fpga_axilite_read_cpl_expect_value !== soc_to_fpga_axilite_read_cpl_captured) begin
				$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access [ERROR] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
				error_cnt = error_cnt + 1;
			end
			else
				$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access [PASS] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);

      //step 4. set heightIn
			//fpga issue cfg write request to soc
      fpga_axilite_write_addr = FPGA_to_SOC_UP_BASE + 'h8; 
      soc_to_fpga_axilite_read_cpl_expect_value = TST_FRAME_HEIGHT;
			fpga_axilite_write(fpga_axilite_write_addr, 4'b0111, soc_to_fpga_axilite_read_cpl_expect_value);

			//fpga issue cfg read request to soc
			fpga_axilite_read_req(fpga_axilite_write_addr);
			//fpga wait for read completion from soc
			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access :wait for soc_to_fpga_axilite_read_cpl_event");
			@(soc_to_fpga_axilite_read_cpl_event);		//wait for fpga get the read cpl.
			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access : got soc_to_fpga_axilite_read_cpl_event");

			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access : soc_to_fpga_axilite_read_cpl_captured=%x", soc_to_fpga_axilite_read_cpl_captured);

			//Data part
      check_cnt = check_cnt + 1;
			if ( soc_to_fpga_axilite_read_cpl_expect_value !== soc_to_fpga_axilite_read_cpl_captured) begin
				$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access [ERROR] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
				error_cnt = error_cnt + 1;
			end
			else
				$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access [PASS] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);

      //step 5. set sw
			//fpga issue cfg write request to soc
      fpga_axilite_write_addr = FPGA_to_SOC_UP_BASE + 'hc; 
      soc_to_fpga_axilite_read_cpl_expect_value = TST_SW;
			fpga_axilite_write(fpga_axilite_write_addr, 4'b0001, soc_to_fpga_axilite_read_cpl_expect_value);

			//fpga issue cfg read request to soc
			fpga_axilite_read_req(fpga_axilite_write_addr);
			//fpga wait for read completion from soc
			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access :wait for soc_to_fpga_axilite_read_cpl_event");
			@(soc_to_fpga_axilite_read_cpl_event);		//wait for fpga get the read cpl.
			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access : got soc_to_fpga_axilite_read_cpl_event");

			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access : soc_to_fpga_axilite_read_cpl_captured=%x", soc_to_fpga_axilite_read_cpl_captured);

			//Data part
      check_cnt = check_cnt + 1;
			if ( soc_to_fpga_axilite_read_cpl_expect_value !== soc_to_fpga_axilite_read_cpl_captured) begin
				$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access [ERROR] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
				error_cnt = error_cnt + 1;
			end
			else
				$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access [PASS] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);

			$display($time, "=> run_edge_detect_fpga_to_soc_CFG_access end");

		end
	endtask

	reg[31:0] idx3;
  reg[31:0] frm_cnt;
  reg[7:0]  tst_img_in_buf [TST_TOTAL_PIXEL_NUM];
  reg[7:0]  tst_img_out_buf[TST_TOTAL_PIXEL_NUM];
  reg[31:0] tst_crc32_img_in_buf[1];
  reg[31:0] tst_crc32_img_out_buf[1];


	task run_edge_detect_test002;		
    // 1. [testbech] start run_edge_detect_test002_fpga_axis_req
    // 2. [soc] do edge detect
    // 3. [testbech] check result

		begin
			for (i=0;i<CoreClkPhaseLoop;i=i+1) begin
				$display("run_edge_detect_test002: fpga_axis_req - loop %02d", i);

        $readmemh("../tests/edge_detect/pattern/in_img.hex",        tst_img_in_buf        );
        $readmemh("../tests/edge_detect/pattern/out_img.hex",       tst_img_out_buf       );
        $readmemh("../tests/edge_detect/pattern/crc32_in_img.hex",  tst_crc32_img_in_buf  );
        $readmemh("../tests/edge_detect/pattern/crc32_out_img.hex", tst_crc32_img_out_buf );


			  for (frm_cnt=0;frm_cnt<TST_TOTAL_FRAME_NUM;frm_cnt=frm_cnt+1) begin
				  $display("run_edge_detect_test002: fpga_axis_req - frame no %02d", frm_cnt);

          //step 1. start run_edge_detect_test002_fpga_axis_req
          soc_to_fpga_axis_expect_count = 0;
          run_edge_detect_test002_fpga_axis_req();		//target to Axis Switch

          $display($time, "=> wait for soc_to_fpga_axis_event");
          @(soc_to_fpga_axis_event);
          $display($time, "=> got soc_to_fpga_axis_event");
          $display($time, "=> final soc_to_fpga_axis_expect_count = %d", soc_to_fpga_axis_expect_count);
          $display($time, "=> final soc_to_fpga_axis_captured_count = %d", soc_to_fpga_axis_captured_count);
			    $display("-----------------");

          //step 2. report check
          //check edgedetect_done
          soc_to_fpga_axilite_read_cpl_expect_value = 32'h1;
    
          //fpga issue cfg read request to soc
          fpga_axilite_read_req(FPGA_to_SOC_UP_BASE + 'h18);
          //fpga wait for read completion from soc
          $display($time, "=> run_edge_detect_test002 :wait for soc_to_fpga_axilite_read_cpl_event");
          @(soc_to_fpga_axilite_read_cpl_event);		//wait for fpga get the read cpl.
          $display($time, "=> run_edge_detect_test002 : got soc_to_fpga_axilite_read_cpl_event");
    
          $display($time, "=> run_edge_detect_test002 : soc_to_fpga_axilite_read_cpl_captured=%x", soc_to_fpga_axilite_read_cpl_captured);
    
          //Data part
          check_cnt = check_cnt + 1;
          if ( soc_to_fpga_axilite_read_cpl_expect_value !== soc_to_fpga_axilite_read_cpl_captured) begin
            $display($time, "=> run_edge_detect_test002 [ERROR] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
            error_cnt = error_cnt + 1;
          end
          else
            $display($time, "=> run_edge_detect_test002 [PASS] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
			    $display("-----------------");

          //step3. check crc32_img_in
          soc_to_fpga_axilite_read_cpl_expect_value = tst_crc32_img_in_buf[0];
    
          //fpga issue cfg read request to soc
          fpga_axilite_read_req(FPGA_to_SOC_UP_BASE + 'h10);
          //fpga wait for read completion from soc
          $display($time, "=> run_edge_detect_test002 :wait for soc_to_fpga_axilite_read_cpl_event");
          @(soc_to_fpga_axilite_read_cpl_event);		//wait for fpga get the read cpl.
          $display($time, "=> run_edge_detect_test002 : got soc_to_fpga_axilite_read_cpl_event");
    
          $display($time, "=> run_edge_detect_test002 : soc_to_fpga_axilite_read_cpl_captured=%x", soc_to_fpga_axilite_read_cpl_captured);
    
          //Data part
          check_cnt = check_cnt + 1;
          if ( soc_to_fpga_axilite_read_cpl_expect_value !== soc_to_fpga_axilite_read_cpl_captured) begin
            $display($time, "=> run_edge_detect_test002 [ERROR] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
            error_cnt = error_cnt + 1;
          end
          else
            $display($time, "=> run_edge_detect_test002 [PASS] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
			    $display("-----------------");

          //step 4. check crc32_img_out
          soc_to_fpga_axilite_read_cpl_expect_value = tst_crc32_img_out_buf[0];
    
          //fpga issue cfg read request to soc
          fpga_axilite_read_req(FPGA_to_SOC_UP_BASE + 'h14);
          //fpga wait for read completion from soc
          $display($time, "=> run_edge_detect_test002 :wait for soc_to_fpga_axilite_read_cpl_event");
          @(soc_to_fpga_axilite_read_cpl_event);		//wait for fpga get the read cpl.
          $display($time, "=> run_edge_detect_test002 : got soc_to_fpga_axilite_read_cpl_event");
    
          $display($time, "=> run_edge_detect_test002 : soc_to_fpga_axilite_read_cpl_captured=%x", soc_to_fpga_axilite_read_cpl_captured);
    
          //Data part
          check_cnt = check_cnt + 1;
          if ( soc_to_fpga_axilite_read_cpl_expect_value !== soc_to_fpga_axilite_read_cpl_captured) begin
            $display($time, "=> run_edge_detect_test002 [ERROR] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
            error_cnt = error_cnt + 1;
          end
          else
            $display($time, "=> run_edge_detect_test002 [PASS] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
			    $display("-----------------");


          //step 5. clear edgedetect_done
          //fpga issue cfg write request to soc
          fpga_axilite_write_addr = FPGA_to_SOC_UP_BASE + 'h18; 
          fpga_axilite_write(fpga_axilite_write_addr, 4'b0001, 8'h01);
			    $display("-----------------");

          //step 6. check axis data

          check_cnt = check_cnt + 1;
          if ( soc_to_fpga_axis_expect_count != fpga_axis_test_length) begin
            $display($time, "=> run_edge_detect_test002 [ERROR] soc_to_fpga_axis_expect_count = %d, soc_to_fpga_axis_captured_count = %d", soc_to_fpga_axis_expect_count, soc_to_fpga_axis_captured_count);
            error_cnt = error_cnt + 1;
          end
          else
            $display($time, "=> run_edge_detect_test002 [PASS] soc_to_fpga_axis_expect_count = %d, soc_to_fpga_axis_captured_count = %d", soc_to_fpga_axis_expect_count, soc_to_fpga_axis_captured_count);
            
          for(idx3=0; idx3<fpga_axis_test_length; idx3=idx3+1)begin
					  check_cnt = check_cnt + 1;
            if (soc_to_fpga_axis_expect_value[idx3] != soc_to_fpga_axis_captured[idx3] ) begin
              $display($time, "=> run_edge_detect_test002 [ERROR] idx3=%d, soc_to_fpga_axis_expect_value[%d] = %x, soc_to_fpga_axis_captured[%d]  = %x", idx3, idx3, soc_to_fpga_axis_expect_value[idx3], idx3, soc_to_fpga_axis_captured[idx3]);
              error_cnt = error_cnt + 1;
            end
            else
              $display($time, "=> run_edge_detect_test002 [PASS] idx3=%d, soc_to_fpga_axis_expect_value[%d] = %x, soc_to_fpga_axis_captured[%d]  = %x", idx3, idx3, soc_to_fpga_axis_expect_value[idx3], idx3, soc_to_fpga_axis_captured[idx3]);
          end //for(idx3=0; idx3<fpga_axis_test_length; idx3=idx3+1)

          soc_to_fpga_axis_captured_count = 0;		//reset soc_to_fpga_axis_captured_count for next loop
          #200;
        end //for (frm_cnt=0;frm_cnt<TST_TOTAL_FRAME_NUM;frm_cnt=frm_cnt+1)
      end //for (i=0;i<CoreClkPhaseLoop;i=i+1) begin
    end
	endtask

  reg        sof;
  reg        eol;
  reg [31:0] hcnt;
  reg [31:0] vcnt;

	task run_edge_detect_test002_fpga_axis_req;
		//input [7:0] compare_data;

    reg [31:0] data;
		`ifdef USER_PROJECT_SIDEBAND_SUPPORT
		reg [pUSER_PROJECT_SIDEBAND_WIDTH-1:0]upsb;
		`endif
		//FPGA to SOC Axilite test
		begin

			@ (posedge fpga_coreclk);
			fpga_as_is_tready <= 1;

      for(vcnt=0; vcnt < TST_FRAME_HEIGHT; vcnt += 1)
        for(hcnt=0; hcnt < TST_FRAME_WIDTH; hcnt += 4) begin
          idx3 = vcnt * TST_FRAME_WIDTH + hcnt;
          data = {tst_img_in_buf[idx3+3], tst_img_in_buf[idx3+2], tst_img_in_buf[idx3+1], tst_img_in_buf[idx3+0]};
          sof = (vcnt==0 && hcnt==0);
          eol = (hcnt== TST_FRAME_WIDTH - 4);
		      `ifdef USER_PROJECT_SIDEBAND_SUPPORT
            upsb = {eol,sof};
            fpga_axis_req(data, TID_DN_UP, 0, upsb);	//target to User Project
          `else  //USER_PROJECT_SIDEBAND_SUPPORT
            fpga_axis_req(data, TID_DN_UP, 0);		//target to User Project
          `endif //USER_PROJECT_SIDEBAND_SUPPORT
			  end
			$display($time, "=> run_edge_detect_test002_fpga_axis_req done");
		end
	endtask

task fpga_axis_req;
		input [31:0] data;
		input [1:0] tid;
		input mode;	//0 for normal, 1 for random data
		`ifdef USER_PROJECT_SIDEBAND_SUPPORT
      input [pUSER_PROJECT_SIDEBAND_WIDTH-1:0] upsb;
		`endif //USER_PROJECT_SIDEBAND_SUPPORT
		reg [31:0] tdata;
		`ifdef USER_PROJECT_SIDEBAND_SUPPORT
			reg [pUSER_PROJECT_SIDEBAND_WIDTH-1:0]tupsb;
		`endif //USER_PROJECT_SIDEBAND_SUPPORT
		reg [3:0] tstrb;
		reg [3:0] tkeep;
		reg tlast;

    reg [31:0] exp_data;

		begin
			if (mode) begin		//for random data
				tdata = $random;
				`ifdef USER_PROJECT_SIDEBAND_SUPPORT
					tupsb = $random;
				`endif //USER_PROJECT_SIDEBAND_SUPPORT
				tstrb = $random;
				tkeep = $random;
				tlast = $random;
        exp_data = tdata;
			end
			else begin
				tdata = data;
				`ifdef USER_PROJECT_SIDEBAND_SUPPORT
					//tupsb = 5'b00000;
					//tupsb = tdata[4:0];
          tupsb = upsb;
				`endif  //USER_PROJECT_SIDEBAND_SUPPORT
				tstrb = 4'b0000;
				tkeep = 4'b0000;
				//tstrb = 4'b1111;
				//tkeep = 4'b1111;
				tlast = upsb[1];   //set tlast = eol
        exp_data = {tst_img_out_buf[idx3+3], tst_img_out_buf[idx3+2], tst_img_out_buf[idx3+1], tst_img_out_buf[idx3+0]};
			end
			`ifdef USER_PROJECT_SIDEBAND_SUPPORT
				fpga_as_is_tupsb <= tupsb;
			`endif //USER_PROJECT_SIDEBAND_SUPPORT
			fpga_as_is_tstrb <=  tstrb;
			fpga_as_is_tkeep <=  tkeep;
			fpga_as_is_tlast <=  tlast;
			fpga_as_is_tdata <= tdata;	//for axis write data
			`ifdef USER_PROJECT_SIDEBAND_SUPPORT
				$strobe($time, "=> fpga_axis_req send data, fpga_as_is_tupsb = %b, fpga_as_is_tstrb = %b, fpga_as_is_tkeep = %b, fpga_as_is_tlast = %b, fpga_as_is_tdata = %x", fpga_as_is_tupsb, fpga_as_is_tstrb, fpga_as_is_tkeep, fpga_as_is_tlast, fpga_as_is_tdata);
			`else //USER_PROJECT_SIDEBAND_SUPPORT
				$strobe($time, "=> fpga_axis_req send data, fpga_as_is_tstrb = %b, fpga_as_is_tkeep = %b, fpga_as_is_tlast = %b, fpga_as_is_tdata = %x", fpga_as_is_tstrb, fpga_as_is_tkeep, fpga_as_is_tlast, fpga_as_is_tdata);
			`endif //USER_PROJECT_SIDEBAND_SUPPORT

			fpga_as_is_tid <=  tid;		//set target
			fpga_as_is_tuser <=  TUSER_AXIS;		//for axis req
			fpga_as_is_tvalid <= 1;
			`ifdef USER_PROJECT_SIDEBAND_SUPPORT
				//soc_to_fpga_axis_expect_value[soc_to_fpga_axis_expect_count] <= {tupsb, tstrb, tkeep, tlast, tdata};
				soc_to_fpga_axis_expect_value[soc_to_fpga_axis_expect_count] <= {tupsb, tstrb, tkeep, tlast, exp_data};
			`else //USER_PROJECT_SIDEBAND_SUPPORT
				//soc_to_fpga_axis_expect_value[soc_to_fpga_axis_expect_count] <= {tstrb, tkeep, tlast, tdata};
				soc_to_fpga_axis_expect_value[soc_to_fpga_axis_expect_count] <= {tstrb, tkeep, tlast, exp_data};
			`endif //USER_PROJECT_SIDEBAND_SUPPORT
			soc_to_fpga_axis_expect_count <= soc_to_fpga_axis_expect_count+1;

			@ (posedge fpga_coreclk);
			while (fpga_is_as_tready == 0) begin		// wait util fpga_is_as_tready == 1 then change data
					@ (posedge fpga_coreclk);
			end
			fpga_as_is_tvalid <= 0;

		end
	endtask


	task test111;		//test111_soc_to_fpga_mailbox_write
    // test111 - for soc CFG write to mailbox
    // 1. [FW] SOC CFG write to mailbox
    // 1.A [testbech] check soc_to_fpga_mailbox_write

		begin
			for (i=0;i<CoreClkPhaseLoop;i=i+1) begin
				$display("test111: test111_soc_to_fpga_mailbox_write - loop %02d", i);
				fork
					//soc_apply_reset(40+i*10, 40);			//change coreclk phase in soc
					fpga_apply_reset(40,40);		//fix coreclk phase in fpga
				join
        init_fpga_as();
			end
		end
	endtask

	task test112;
    // test112 - for_soc_CFG_read_and_write_to_mailbox_then_send_to_fpga
    // 1. [FW] SOC CFG read and write to mailbox
    // 1.A [testbech] check soc_to_fpga_mailbox_write
    //SOC internal register read/write test and update the result to mailbox, testbench check the data when received the mail box write.

		begin
			for (i=0;i<CoreClkPhaseLoop;i=i+1) begin
				$display("test111: for_soc_CFG_read_and_write_to_mailbox_then_send_to_fpga - loop %02d", i);
				fork
					//soc_apply_reset(40+i*10, 40);			//change coreclk phase in soc
					fpga_apply_reset(40,40);		//fix coreclk phase in fpga
				join
        init_fpga_as();
			end
		end
	endtask

	task test103;   //test103_fpga_to_soc_cfg_read
    // test103 - test103_fpga_to_soc_cfg_read
    // 1. [testbech] fpga to soc CFG read
    // 2. [HW] SOC return CFG read cpl to fpga, (FW code only need to init REG_IS_BASE)
    // 2.A [testbech] check CFG read cpl in fpga
		begin
			for (i=0;i<CoreClkPhaseLoop;i=i+1) begin
				$display("test103: fpga_cfg_read test - loop %02d", i);
				fork
					//soc_apply_reset(40+i*10, 40);			//change coreclk phase in soc
					fpga_apply_reset(40,40);		//fix coreclk phase in fpga
				join
        init_fpga_as();
				test103_fpga_to_soc_cfg_read();
				#200;
			end
      finish_flag=1;
		end
	endtask

	task test104;   //test104_fpga_to_soc_mail_box_write
    // test104 - fpga to soc mailbox loopback test
    // 1. [testbech] fpga to soc mialbox write to offset 0
    // 2. [FW] soc read mailbox offset 0, if non zero then write the read_value to mailbox offset 4
    // 2.A [testbech] check soc to fpga mailbox write to offset 4 in fpga
		begin
			for (i=0;i<CoreClkPhaseLoop;i=i+1) begin
				$display("test104: TX/RX test - loop %02d", i);
				fork
					//soc_apply_reset(40+i*10, 40);			//change coreclk phase in soc
					fpga_apply_reset(40,40);		//fix coreclk phase in fpga
				join
        init_fpga_as();
				test104_fpga_to_soc_mail_box_write();		//target to AA
				#200;
			end
      finish_flag=1;
		end

	endtask

	task test104_fpga_to_soc_mail_box_write;
		//input [7:0] compare_data;

		//FPGA to SOC Axilite test
		begin
			@ (posedge fpga_coreclk);
			fpga_as_is_tready <= 1;

			$display($time, "=> test104_fpga_to_soc_mail_box_write start");
      soc_to_fpga_mailbox_write_addr_expect_value = 28'h000_2000 + 4;
      soc_to_fpga_mailbox_write_addr_BE_expect_value = 4'b1111;
      soc_to_fpga_mailbox_write_data_expect_value = 32'h1111_1111;

    // 1. [testbech] fpga to soc mialbox write to offset 0
    // 2.A [testbech] check soc to fpga mailbox write to offset 4 in fpga

			fpga_axilite_write(FPGA_to_SOC_AA_BASE + AA_MailBox_Reg_Offset + 0, soc_to_fpga_mailbox_write_addr_BE_expect_value, soc_to_fpga_mailbox_write_data_expect_value);
			$display($time, "=> test104_fpga_to_soc_mail_box_write fpga_axilite_write addr = %x, be=%x, data = %x", FPGA_to_SOC_AA_BASE + AA_MailBox_Reg_Offset + 0, soc_to_fpga_mailbox_write_addr_BE_expect_value, soc_to_fpga_mailbox_write_data_expect_value);
      wait_and_check_soc_to_fpga_mailbox_write_event();

			$display($time, "=> test104_fpga_to_soc_mail_box_write done");
		end
	endtask

	task test113;   //test113_fpga_to_soc_CFG_write
    // test113 - fpga to soc CFG write test
    // 1. [testbech] fpga to soc CFG write to AA_Internal_Reg_Offset + 0
    // 2. [FW] soc read AA_Internal_Reg_Offset + 0, if non zero then write the read_value to mailbox offset 4
    // 2.A [testbech] check soc to fpga mailbox write to offset 4 in fpga
		begin
			for (i=0;i<CoreClkPhaseLoop;i=i+1) begin
				$display("test113: TX/RX test - loop %02d", i);
				fork
					//soc_apply_reset(40+i*10, 40);			//change coreclk phase in soc
					fpga_apply_reset(40,40);		//fix coreclk phase in fpga
				join
        init_fpga_as();
				test113_fpga_to_soc_CFG_write();		//target to AA_Internal_Reg_Offset
				#200;
			end
      finish_flag=1;
		end

	endtask

	task test113_fpga_to_soc_CFG_write;
		//FPGA to SOC Axilite test
		begin
			@ (posedge fpga_coreclk);
			fpga_as_is_tready <= 1;

			$display($time, "=> test113_fpga_to_soc_CFG_write start");
      fpga_axilite_write_addr = FPGA_to_SOC_AA_BASE + AA_Internal_Reg_Offset + 0;
      soc_to_fpga_mailbox_write_addr_expect_value = FPGA_to_SOC_AA_BASE + AA_MailBox_Reg_Offset + 4;
      soc_to_fpga_mailbox_write_addr_BE_expect_value = 4'b1111;
      soc_to_fpga_mailbox_write_data_expect_value = 32'h1;

    // 1. [testbech] fpga to soc CFG write to AA_Internal_Reg_Offset + 0
    // 2.A [testbech] check soc to fpga mailbox write to offset 4 in fpga

			fpga_axilite_write(fpga_axilite_write_addr, soc_to_fpga_mailbox_write_addr_BE_expect_value, soc_to_fpga_mailbox_write_data_expect_value);
			$display($time, "=> test113_fpga_to_soc_CFG_write fpga_axilite_write_addr = %x, be=%x, data = %x", fpga_axilite_write_addr, soc_to_fpga_mailbox_write_addr_BE_expect_value, soc_to_fpga_mailbox_write_data_expect_value);
      wait_and_check_soc_to_fpga_mailbox_write_event();

			$display($time, "=> test113_fpga_to_soc_CFG_write done");
		end
	endtask

	task test114;   //fpga to soc mailbox write with interrupt test
    //test114 - fpga to soc mailbox write with interrupt test
    // 1. [FW] init interrupt handler
    // 1.A [FW] soc enable interrupt by set AA_Internal_Reg_Offset + 0 = 1
    // 2. [testbech] fpga to soc CFG write to AA_MailBox_Reg_Offset + 0 = value
    // 3. [FW] in isr read value from AA_MailBox_Reg_Offset + 0 and write to AA_MailBox_Reg_Offset + 4
    // 4. [testbech] fpga check AA_MailBox_Reg_Offset + 4 = value
		begin
		`ifdef USER_PROJ_IRQ0_EN
			$display("USER_PROJ_IRQ0 Test");
		`endif

			for (i=0;i<CoreClkPhaseLoop;i=i+1) begin
				$display("test114: TX/RX test - loop %02d", i);
				fork
					//soc_apply_reset(40+i*10, 40);			//change coreclk phase in soc
					fpga_apply_reset(40,40);		//fix coreclk phase in fpga
				join
        init_fpga_as();
				$display($time, "=> wait uut.mprj.u_fsic.U_AXIL_AXIS0.axi_ctrl_logic.aa_regs[0][0] (interrupt enable bit)");
        wait(uut.mprj.u_fsic.U_AXIL_AXIS0.axi_ctrl_logic.aa_regs[0][0]); //wait interrupt enable bit = 1
				$display($time, "=> detect uut.mprj.u_fsic.U_AXIL_AXIS0.axi_ctrl_logic.aa_regs[0][0]=1");
				test114_fpga_to_soc_CFG_write();		//target to AA_MailBox_Reg_Offset
				#200;
			end
      finish_flag=1;
		end

	endtask

	task test114_fpga_to_soc_CFG_write;
		//FPGA to SOC Axilite test
		begin
			@ (posedge fpga_coreclk);
			fpga_as_is_tready <= 1;

			$display($time, "=> test114_fpga_to_soc_CFG_write start");
      fpga_axilite_write_addr = FPGA_to_SOC_AA_BASE + AA_MailBox_Reg_Offset + 0;

      soc_to_fpga_mailbox_write_addr_expect_value = FPGA_to_SOC_AA_BASE + AA_MailBox_Reg_Offset + 4;
      soc_to_fpga_mailbox_write_addr_BE_expect_value = 4'b1111;
      soc_to_fpga_mailbox_write_data_expect_value = 32'h1;

    // 1. [testbech] fpga to soc CFG write to AA_MailBox_Reg_Offset + 0
    // 2.A [testbech] check soc to fpga mailbox write to offset 4 in fpga

			fpga_axilite_write(fpga_axilite_write_addr, soc_to_fpga_mailbox_write_addr_BE_expect_value, soc_to_fpga_mailbox_write_data_expect_value);
			$display($time, "=> test114_fpga_to_soc_CFG_write fpga_axilite_write_addr = %x, be=%x, data = %x", fpga_axilite_write_addr, soc_to_fpga_mailbox_write_addr_BE_expect_value, soc_to_fpga_mailbox_write_data_expect_value);
      wait_and_check_soc_to_fpga_mailbox_write_event();
			$display($time, "=> test114_fpga_to_soc_CFG_write wait 320us");
      			repeat(8000) @(posedge fpga_coreclk); //wait 320us

      fpga_axilite_write_addr = FPGA_to_SOC_AA_BASE + AA_MailBox_Reg_Offset + 0;
      soc_to_fpga_mailbox_write_addr_expect_value = FPGA_to_SOC_AA_BASE + AA_MailBox_Reg_Offset + 4;
      soc_to_fpga_mailbox_write_addr_BE_expect_value = 4'b1111;
      soc_to_fpga_mailbox_write_data_expect_value = 32'h5a5a_5a5a;

    // 1. [testbech] fpga to soc CFG write to AA_MailBox_Reg_Offset + 0
    // 2.A [testbech] check soc to fpga mailbox write to offset 4 in fpga

			fpga_axilite_write(fpga_axilite_write_addr, soc_to_fpga_mailbox_write_addr_BE_expect_value, soc_to_fpga_mailbox_write_data_expect_value);
			$display($time, "=> test114_fpga_to_soc_CFG_write fpga_axilite_write_addr = %x, be=%x, data = %x", fpga_axilite_write_addr, soc_to_fpga_mailbox_write_addr_BE_expect_value, soc_to_fpga_mailbox_write_data_expect_value);
      wait_and_check_soc_to_fpga_mailbox_write_event();

			$display($time, "=> test114_fpga_to_soc_CFG_write done");


		end
	endtask

	task fpga_axilite_write;
		input [27:0] address;
		input [3:0] BE;
		input [31:0] data;
		begin
			fpga_as_is_tdata <= (BE<<28) + address;	//for axilite write address phase
			//$strobe($time, "=> fpga_as_is_tdata in address phase = %x", fpga_as_is_tdata);
			fpga_as_is_tstrb <=  4'b0000;
			fpga_as_is_tkeep <=  4'b0000;
			fpga_as_is_tid <=  TID_DN_AA ;		//target to Axis-Axilite
			fpga_as_is_tuser <=  TUSER_AXILITE_WRITE;		//for axilite write
			fpga_as_is_tlast <=  1'b0;
			fpga_as_is_tvalid <= 1;

			@ (posedge fpga_coreclk);
			while (fpga_is_as_tready == 0) begin		// wait util fpga_is_as_tready == 1 then change data
					@ (posedge fpga_coreclk);
			end

			fpga_as_is_tdata <=  data;	//for axilite write data phase
			fpga_as_is_tstrb <=  4'b0000;
			fpga_as_is_tkeep <=  4'b0000;
			fpga_as_is_tid <=  TID_DN_AA;		//target to Axis-Axilite
			fpga_as_is_tuser <=  TUSER_AXILITE_WRITE;		//for axilite write
			fpga_as_is_tlast <=  1'b0;
			fpga_as_is_tvalid <= 1;

			@ (posedge fpga_coreclk);
			while (fpga_is_as_tready == 0) begin		// wait util fpga_is_as_tready == 1 then change data
					@ (posedge fpga_coreclk);
			end
			fpga_as_is_tvalid <= 0;

		end
	endtask

	reg[31:0]idx2;

	task test103_fpga_to_soc_cfg_read;		//target to io serdes
		//input [7:0] compare_data;

		//FPGA to SOC Axilite test
		begin

			@ (posedge fpga_coreclk);
			fpga_as_is_tready <= 1;

			for(idx2=0; idx2<32/4; idx2=idx2+1)begin		//
				//step 1. fpga issue cfg read request to soc
        soc_to_fpga_axilite_read_cpl_expect_value = 32'h3;
				fpga_axilite_read_req(FPGA_to_SOC_IS_BASE + idx2*4);
					//read address = h0000_3000 ~ h0000_301F for io serdes
				//step 2. fpga wait for read completion from soc
				$display($time, "=> test103_fpga_to_soc_cfg_read :wait for soc_to_fpga_axilite_read_cpl_event");
				@(soc_to_fpga_axilite_read_cpl_event);		//wait for fpga get the read cpl.
				$display($time, "=> test103_fpga_to_soc_cfg_read : got soc_to_fpga_axilite_read_cpl_event");

				$display($time, "=> test103_fpga_to_soc_cfg_read : soc_to_fpga_axilite_read_cpl_captured=%x", soc_to_fpga_axilite_read_cpl_captured);

				//Data part
        check_cnt = check_cnt + 1;
				if ( soc_to_fpga_axilite_read_cpl_expect_value !== soc_to_fpga_axilite_read_cpl_captured) begin
					$display($time, "=> test103_fpga_to_soc_cfg_read [ERROR] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
					error_cnt = error_cnt + 1;
				end
				else
					$display($time, "=> test103_fpga_to_soc_cfg_read [PASS] soc_to_fpga_axilite_read_cpl_expect_value=%x, soc_to_fpga_axilite_read_cpl_captured[27:0]=%x", soc_to_fpga_axilite_read_cpl_expect_value, soc_to_fpga_axilite_read_cpl_captured[27:0]);
			end
			$display($time, "=> test103_fpga_to_soc_cfg_read done");
		end
	endtask

	task fpga_axilite_read_req;
		input [31:0] address;
		begin
			fpga_as_is_tdata <= address;	//for axilite read address req phase
			$strobe($time, "=> fpga_axilite_read_req in address req phase = %x - tvalid", fpga_as_is_tdata);
			fpga_as_is_tstrb <=  4'b0000;
			fpga_as_is_tkeep <=  4'b0000;
			fpga_as_is_tid <=  TID_DN_AA;		//target to Axis-Axilite
			fpga_as_is_tuser <=  TUSER_AXILITE_READ_REQ;		//for axilite read req
			fpga_as_is_tlast <=  1'b0;
			fpga_as_is_tvalid <= 1;

			@ (posedge fpga_coreclk);
			while (fpga_is_as_tready == 0) begin		// wait util fpga_is_as_tready == 1 then change data
					@ (posedge fpga_coreclk);
			end
			$display($time, "=> fpga_axilite_read_req in address req phase = %x - transfer", fpga_as_is_tdata);
			fpga_as_is_tvalid <= 0;

		end
	endtask


//	reg[31:0]idx3;

	task test111_fpga_axis_req;
		//input [7:0] compare_data;

		//FPGA to SOC Axilite test
		begin
			//tony_Debug force uut.AXIS_SW0.up_as_tready = 1;

			@ (posedge fpga_coreclk);
			fpga_as_is_tready <= 1;

			for(idx3=0; idx3<32; idx3=idx3+1)begin		//
	      `ifdef USER_PROJECT_SIDEBAND_SUPPORT
            fpga_axis_req(32'h11111111 * (idx3 & 32'h0000_000F), TID_DN_UP, 0, 0);		//target to User Project
         `else  //USER_PROJECT_SIDEBAND_SUPPORT
            fpga_axis_req(32'h11111111 * (idx3 & 32'h0000_000F), TID_DN_UP, 0);		//target to User Project
         `endif //USER_PROJECT_SIDEBAND_SUPPORT
				//if (idx3 > 12 ) 			force dut.AXIS_SW0.up_as_tready = 1;
			end
			//tony_Debug release dut.AXIS_SW0.up_as_tready;

			$display($time, "=> test111_fpga_axis_req done");
		end
	endtask


	task fpga_as_to_is_init;
		//input [7:0] compare_data;

		begin
			//init fpga as to is signal, set fpga_as_is_tready = 1 for receives data from soc
			@ (posedge fpga_coreclk);
			fpga_as_is_tdata <=  32'h0;
      `ifdef USER_PROJECT_SIDEBAND_SUPPORT
        fpga_as_is_tupsb <= 5'h00;
      `endif
			fpga_as_is_tstrb <=  4'b0000;
			fpga_as_is_tkeep <=  4'b0000;
			fpga_as_is_tid <=  TID_DN_UP;
			fpga_as_is_tuser <=  TUSER_AXIS;
			fpga_as_is_tlast <=  1'b0;
			fpga_as_is_tvalid <= 0;
			fpga_as_is_tready <= 1;
			$display($time, "=> fpga_as_to_is_init done");
		end
	endtask

	task fpga_cfg_write;		//input addr, data, strb and valid_delay
		input [pADDR_WIDTH-1:0] axi_awaddr;
		input [pDATA_WIDTH-1:0] axi_wdata;
		input [3:0] axi_wstrb;
		input [7:0] valid_delay;

		begin
			fpga_axi_awaddr <= axi_awaddr;
			fpga_axi_awvalid <= 0;
			fpga_axi_wdata <= axi_wdata;
			fpga_axi_wstrb <= axi_wstrb;
			fpga_axi_wvalid <= 0;
			//$display($time, "=> fpga_delay_valid before : valid_delay=%x", valid_delay);
			repeat (valid_delay) @ (posedge fpga_coreclk);
			//$display($time, "=> fpga_delay_valid after  : valid_delay=%x", valid_delay);
			fpga_axi_awvalid <= 1;
			fpga_axi_wvalid <= 1;
			@ (posedge fpga_coreclk);
			while (fpga_axi_awready == 0) begin		//assume both fpga_axi_awready and fpga_axi_wready assert as the same time.
					@ (posedge fpga_coreclk);
			end
			$display($time, "=> fpga_cfg_write : fpga_axi_awaddr=%x, fpga_axi_awvalid=%b, fpga_axi_awready=%b, fpga_axi_wdata=%x, axi_wstrb=%x, fpga_axi_wvalid=%b, fpga_axi_wready=%b", fpga_axi_awaddr, fpga_axi_awvalid, fpga_axi_awready, fpga_axi_wdata, axi_wstrb, fpga_axi_wvalid, fpga_axi_wready);
			fpga_axi_awvalid <= 0;
			fpga_axi_wvalid <= 0;
		end

	endtask

  task wait_and_check_soc_to_fpga_mailbox_write_event;
    begin
      wait(soc_to_fpga_mailbox_write_event.triggered);
			$display($time, "=> wait_and_check_soc_to_fpga_mailbox_write_event : got soc_to_fpga_mailbox_write_event");
			//Address part
      check_cnt = check_cnt + 1;
			if ( soc_to_fpga_mailbox_write_addr_expect_value !== soc_to_fpga_mailbox_write_addr_captured[27:0]) begin
				$display($time, "=> wait_and_check_soc_to_fpga_mailbox_write_event [ERROR] soc_to_fpga_mailbox_write_addr_expect_value=%x, soc_to_fpga_mailbox_write_addr_captured[27:0]=%x", soc_to_fpga_mailbox_write_addr_expect_value, soc_to_fpga_mailbox_write_addr_captured[27:0]);
				error_cnt = error_cnt + 1;
			end
			else
				$display($time, "=> wait_and_check_soc_to_fpga_mailbox_write_event [PASS] soc_to_fpga_mailbox_write_addr_expect_value=%x, soc_to_fpga_mailbox_write_addr_captured[27:0]=%x", soc_to_fpga_mailbox_write_addr_expect_value, soc_to_fpga_mailbox_write_addr_captured[27:0]);

			//BE part
      check_cnt = check_cnt + 1;
			if ( soc_to_fpga_mailbox_write_addr_BE_expect_value !== soc_to_fpga_mailbox_write_addr_captured[31:28]) begin
				$display($time, "=> wait_and_check_soc_to_fpga_mailbox_write_event [ERROR] soc_to_fpga_mailbox_write_addr_BE_expect_value=%x, soc_to_fpga_mailbox_write_addr_captured[31:28]=%x", soc_to_fpga_mailbox_write_addr_BE_expect_value, soc_to_fpga_mailbox_write_addr_captured[31:28]);
				error_cnt = error_cnt + 1;
			end
			else
				$display($time, "=> wait_and_check_soc_to_fpga_mailbox_write_event [PASS] soc_to_fpga_mailbox_write_addr_BE_expect_value=%x, soc_to_fpga_mailbox_write_addr_captured[31:28]=%x", soc_to_fpga_mailbox_write_addr_BE_expect_value, soc_to_fpga_mailbox_write_addr_captured[31:28]);

			//data part
      check_cnt = check_cnt + 1;
			if (soc_to_fpga_mailbox_write_data_expect_value !== soc_to_fpga_mailbox_write_data_captured) begin
				$display($time, "=> wait_and_check_soc_to_fpga_mailbox_write_event [ERROR] soc_to_fpga_mailbox_write_data_expect_value=%x, soc_to_fpga_mailbox_write_data_captured=%x", soc_to_fpga_mailbox_write_data_expect_value, soc_to_fpga_mailbox_write_data_captured);
				error_cnt = error_cnt + 1;
			end
			else
				$display($time, "=> wait_and_check_soc_to_fpga_mailbox_write_event [PASS] soc_to_fpga_mailbox_write_data_expect_value=%x, soc_to_fpga_mailbox_write_data_captured=%x", soc_to_fpga_mailbox_write_data_expect_value, soc_to_fpga_mailbox_write_data_captured);
			$display("-----------------");
      @(posedge fpga_coreclk);

    end
  endtask

	initial begin		//get upstream soc_to_fpga_axilite_read_completion
		while (1) begin
			@(posedge fpga_coreclk);
			if (fpga_is_as_tvalid == 1 && fpga_is_as_tid == TID_UP_AA && fpga_is_as_tuser == TUSER_AXILITE_READ_CPL) begin
				$display($time, "=> get soc_to_fpga_axilite_read_cpl_captured be : soc_to_fpga_axilite_read_cpl_captured =%x, fpga_is_as_tdata=%x", soc_to_fpga_axilite_read_cpl_captured, fpga_is_as_tdata);
				soc_to_fpga_axilite_read_cpl_captured = fpga_is_as_tdata ;		//use block assignment
				$display($time, "=> get soc_to_fpga_axilite_read_cpl_captured af : soc_to_fpga_axilite_read_cpl_captured =%x, fpga_is_as_tdata=%x", soc_to_fpga_axilite_read_cpl_captured, fpga_is_as_tdata);
				#0 -> soc_to_fpga_axilite_read_cpl_event;
				$display($time, "=> soc_to_fpga_axilite_read_cpl_captured : send soc_to_fpga_axilite_read_cpl_event");
			end
		end
	end

  reg soc_to_fpga_axis_event_triggered;

  initial begin            //get upstream soc_to_fpga_axis - for loop back test
    soc_to_fpga_axis_captured_count = 0;
    soc_to_fpga_axis_event_triggered = 0;
    while (1) begin
      @(posedge fpga_coreclk);
      `ifdef USER_PROJECT_SIDEBAND_SUPPORT
        if (fpga_is_as_tvalid == 1 && fpga_is_as_tid == TID_UP_UP && fpga_is_as_tuser == TUSER_AXIS) begin
          $display($time, "=> get soc_to_fpga_axis be : soc_to_fpga_axis_captured_count=%d,  soc_to_fpga_axis_captured[%d] =%x, fpga_is_as_tupsb=%x, fpga_is_as_tstrb=%x, fpga_is_as_tkeep=%x , fpga_is_as_tlast=%x, fpga_is_as_tdata=%x", soc_to_fpga_axis_captured_count, soc_to_fpga_axis_captured_count, soc_to_fpga_axis_captured[soc_to_fpga_axis_captured_count], fpga_is_as_tupsb, fpga_is_as_tstrb, fpga_is_as_tkeep , fpga_is_as_tlast, fpga_is_as_tdata);
          soc_to_fpga_axis_captured[soc_to_fpga_axis_captured_count] = {fpga_is_as_tupsb, fpga_is_as_tstrb, fpga_is_as_tkeep , fpga_is_as_tlast, fpga_is_as_tdata} ;            //use block assignment
          $display($time, "=> get soc_to_fpga_axis af : soc_to_fpga_axis_captured_count=%d,  soc_to_fpga_axis_captured[%d] =%x, fpga_is_as_tupsb=%x, fpga_is_as_tstrb=%x, fpga_is_as_tkeep=%x , fpga_is_as_tlast=%x, fpga_is_as_tdata=%x", soc_to_fpga_axis_captured_count, soc_to_fpga_axis_captured_count, soc_to_fpga_axis_captured[soc_to_fpga_axis_captured_count], fpga_is_as_tupsb, fpga_is_as_tstrb, fpga_is_as_tkeep , fpga_is_as_tlast, fpga_is_as_tdata);
          soc_to_fpga_axis_captured_count = soc_to_fpga_axis_captured_count+1;
        end      
        if ( (soc_to_fpga_axis_captured_count == fpga_axis_test_length) && !soc_to_fpga_axis_event_triggered) begin
          $display($time, "=> soc_to_fpga_axis_captured : send soc_to_fpga_axiis_event");
          #0 -> soc_to_fpga_axis_event;
          soc_to_fpga_axis_event_triggered = 1;
        end 
      `else  //USER_PROJECT_SIDEBAND_SUPPORT
        if (fpga_is_as_tvalid == 1 && fpga_is_as_tid == TID_UP_UP && fpga_is_as_tuser == TUSER_AXIS) begin
          $display($time, "=> get soc_to_fpga_axis be : soc_to_fpga_axis_captured_count=%d,  soc_to_fpga_axis_captured[%d] =%x, fpga_is_as_tstrb=%x, fpga_is_as_tkeep=%x , fpga_is_as_tlast=%x, fpga_is_as_tdata=%x", soc_to_fpga_axis_captured_count, soc_to_fpga_axis_captured_count, soc_to_fpga_axis_captured[soc_to_fpga_axis_captured_count], fpga_is_as_tstrb, fpga_is_as_tkeep , fpga_is_as_tlast, fpga_is_as_tdata);
          soc_to_fpga_axis_captured[soc_to_fpga_axis_captured_count] = {fpga_is_as_tstrb, fpga_is_as_tkeep , fpga_is_as_tlast, fpga_is_as_tdata} ;            //use block assignment
          $display($time, "=> get soc_to_fpga_axis af : soc_to_fpga_axis_captured_count=%d,  soc_to_fpga_axis_captured[%d] =%x, fpga_is_as_tstrb=%x, fpga_is_as_tkeep=%x , fpga_is_as_tlast=%x, fpga_is_as_tdata=%x", soc_to_fpga_axis_captured_count, soc_to_fpga_axis_captured_count, soc_to_fpga_axis_captured[soc_to_fpga_axis_captured_count], fpga_is_as_tstrb, fpga_is_as_tkeep , fpga_is_as_tlast, fpga_is_as_tdata);
          soc_to_fpga_axis_captured_count = soc_to_fpga_axis_captured_count+1;
        end      
        if ( (soc_to_fpga_axis_captured_count == fpga_axis_test_length) && !soc_to_fpga_axis_event_triggered) begin
          $display($time, "=> soc_to_fpga_axis_captured : send soc_to_fpga_axiis_event");
          #0 -> soc_to_fpga_axis_event;
          soc_to_fpga_axis_event_triggered = 1;
        end 
      `endif //USER_PROJECT_SIDEBAND_SUPPORT

          
    if (soc_to_fpga_axis_captured_count != fpga_axis_test_length)
        soc_to_fpga_axis_event_triggered = 0;

    end
  end

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






