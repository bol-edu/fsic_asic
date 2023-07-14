// `default_nettype none
/*
 *  SPDX-FileCopyrightText: 2017 Clifford Wolf
 *
 *  PicoSoC - A simple example SoC using PicoRV32
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *  SPDX-License-Identifier: ISC
 */

`timescale 1 ns / 1 ps

//
// Simple SPI flash simulation model
//
// This model samples io input signals 1ns before the SPI clock edge and
// updates output signals 1ns after the SPI clock edge.
//
// Supported commands:
//    AB, B9, FF, 03, BB, EB, ED
//
// Well written SPI flash data sheets:
//    Cypress S25FL064L http://www.cypress.com/file/316661/download
//    Cypress S25FL128L http://www.cypress.com/file/316171/download
//

module spiflash #(
	parameter FILENAME = "firmware.hex"
)(
	input wire csb,
	input wire clk,
	inout wire io0, // MOSI
	inout wire io1, // MISO
	inout wire io2,
	inout wire io3
);
	localparam verbose = 0;
	localparam integer latency = 8;
	
	reg [7:0] buffer;
	reg [3:0] reset_count = 0;
	reg [3:0] reset_monitor = 0;
	integer bitcount = 0;
	integer bytecount = 0;
	integer dummycount = 0;

	reg  [7:0] spi_cmd;
	reg  [7:0] xip_cmd = 0;
	reg [23:0] spi_addr;

	reg [7:0] spi_in;
	reg [7:0] spi_out;
	reg spi_io_vld;

        // change from 0 to 1 for initial read
	reg powered_up = 1;

	localparam [3:0] mode_spi         = 1;
	localparam [3:0] mode_dspi_rd     = 2;
	localparam [3:0] mode_dspi_wr     = 3;
	localparam [3:0] mode_qspi_rd     = 4;
	localparam [3:0] mode_qspi_wr     = 5;
	localparam [3:0] mode_qspi_ddr_rd = 6;
	localparam [3:0] mode_qspi_ddr_wr = 7;

	reg [3:0] mode = 0;
	reg [3:0] next_mode = 0;

	reg io0_oe = 0;
	reg io1_oe = 0;
	reg io2_oe = 0;
	reg io3_oe = 0;

	reg io0_dout = 0;
	reg io1_dout = 0;
	reg io2_dout = 0;
	reg io3_dout = 0;

	assign #1 io0 = io0_oe ? io0_dout : 1'bz;
	assign #1 io1 = io1_oe ? io1_dout : 1'bz;
	assign #1 io2 = io2_oe ? io2_dout : 1'bz;
	assign #1 io3 = io3_oe ? io3_dout : 1'bz;

	wire io0_delayed;
	wire io1_delayed;
	wire io2_delayed;
	wire io3_delayed;

	assign #1 io0_delayed = io0;
	assign #1 io1_delayed = io1;
	assign #1 io2_delayed = io2;
	assign #1 io3_delayed = io3;

	// 16 MB (128Mb) Flash
	reg [7:0] memory [0:16*1024*1024-1];

	initial begin
                /*
		$display("Memory 5 bytes = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", memory[1048576], 
                                                                                memory[1048577],
                                                                                memory[1048578],
                                                                                memory[1048579],
                                                                                memory[1048580]);
                */
		$display("Reading %s",  FILENAME);
		$readmemh(FILENAME, memory);
		$display("%s loaded into memory", FILENAME);
                /*
		$display("Memory 5 bytes = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", memory[1048576],
                                                                                memory[1048577],
                                                                                memory[1048578],
                                                                                memory[1048579],
                                                                                memory[1048580]);
                */
		$display("Memory 5 bytes = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", memory[0],
                                                                                memory[1],
                                                                                memory[2],
                                                                                memory[3],
                                                                                memory[4]);
	end

	task spi_action;
		begin
			spi_in = buffer;

			if (bytecount == 1) begin
				spi_cmd = buffer;

				if (spi_cmd == 8'h ab)      // release of power-down
					powered_up = 1;

				if (spi_cmd == 8'h b9)      // enter deep power-down
					powered_up = 0;

				if (spi_cmd == 8'h ff)
					xip_cmd = 0;
			end

			if (powered_up && spi_cmd == 'h 03) begin  // read
				if (bytecount == 2)
					spi_addr[23:16] = buffer;

				if (bytecount == 3)
					spi_addr[15:8] = buffer;

				if (bytecount == 4)
					spi_addr[7:0] = buffer;

				if (bytecount >= 4) begin
					buffer = memory[spi_addr];
					spi_addr = spi_addr + 1;
				end
			end

			if (powered_up && spi_cmd == 'h bb) begin  // dual I/O read
				if (bytecount == 1)
					mode = mode_dspi_rd;

				if (bytecount == 2)
					spi_addr[23:16] = buffer;

				if (bytecount == 3)
					spi_addr[15:8] = buffer;

				if (bytecount == 4)
					spi_addr[7:0] = buffer;

				if (bytecount == 5) begin
					xip_cmd = (buffer == 8'h a5) ? spi_cmd : 8'h 00;
					mode = mode_dspi_wr;
					dummycount = latency;
				end

				if (bytecount >= 5) begin
					buffer = memory[spi_addr];
					spi_addr = spi_addr + 1;
				end
			end

			if (powered_up && spi_cmd == 'h eb) begin  // quad I/O read
				if (bytecount == 1)
					mode = mode_qspi_rd;

				if (bytecount == 2)
					spi_addr[23:16] = buffer;

				if (bytecount == 3)
					spi_addr[15:8] = buffer;

				if (bytecount == 4)
					spi_addr[7:0] = buffer;

				if (bytecount == 5) begin
					xip_cmd = (buffer == 8'h a5) ? spi_cmd : 8'h 00;
					mode = mode_qspi_wr;
					dummycount = latency;
				end

				if (bytecount >= 5) begin
					buffer = memory[spi_addr];
					spi_addr = spi_addr + 1;
				end
			end

			if (powered_up && spi_cmd == 'h ed) begin   // DDR quad I/O read
				if (bytecount == 1)
					next_mode = mode_qspi_ddr_rd;

				if (bytecount == 2)
					spi_addr[23:16] = buffer;

				if (bytecount == 3)
					spi_addr[15:8] = buffer;

				if (bytecount == 4)
					spi_addr[7:0] = buffer;

				if (bytecount == 5) begin
					xip_cmd = (buffer == 8'h a5) ? spi_cmd : 8'h 00;
					mode = mode_qspi_ddr_wr;
					dummycount = latency;
				end

				if (bytecount >= 5) begin
					buffer = memory[spi_addr];
					spi_addr = spi_addr + 1;
				end
			end

			spi_out = buffer;
			spi_io_vld = 1;

			if (verbose) begin
				if (bytecount == 1)
					$write("<SPI-START>");
				$write("<SPI:%02x:%02x>", spi_in, spi_out);
			end

		end
	endtask

	task ddr_rd_edge;
		begin
			buffer = {buffer, io3_delayed, io2_delayed, io1_delayed, io0_delayed};
			bitcount = bitcount + 4;
			if (bitcount == 8) begin
				bitcount = 0;
				bytecount = bytecount + 1;
				spi_action;
			end
		end
	endtask

	task ddr_wr_edge;
		begin
			io0_oe = 1;
			io1_oe = 1;
			io2_oe = 1;
			io3_oe = 1;

			io0_dout = buffer[4];
			io1_dout = buffer[5];
			io2_dout = buffer[6];
			io3_dout = buffer[7];

			buffer = {buffer, 4'h 0};
			bitcount = bitcount + 4;
			if (bitcount == 8) begin
				bitcount = 0;
				bytecount = bytecount + 1;
				spi_action;
			end
		end
	endtask

	always @(csb) begin
		if (csb) begin
			if (verbose) begin
				$display("");
				$fflush;
			end
			buffer = 0;
			bitcount = 0;
			bytecount = 0;
			mode = mode_spi;
			io0_oe = 0;
			io1_oe = 0;
			io2_oe = 0;
			io3_oe = 0;

			// Handle MBR.  If in XIP continuous mode, the following
			// 8 clock cycles are normally not expected to be a command.
			// If followed by CSB high, however, if the address bits
			// are consistent with io0 == 1 for 8 clk cycles, then an
			// MBR has been issued and the system must exit XIP
			// continuous mode.
			if (xip_cmd == 8'hbb || xip_cmd == 8'heb
					|| xip_cmd == 8'hed) begin
				if (reset_count == 4'h8 && reset_monitor == 4'h8) begin
					xip_cmd = 8'h00;
					spi_cmd = 8'h03;
				end
			end
		end else
		if (xip_cmd) begin
			buffer = xip_cmd;
			bitcount = 0;
			bytecount = 1;
			spi_action;
		end
	end

	always @(posedge clk or posedge csb) begin
		if (csb == 1'b1) begin
			reset_count = 0;
			reset_monitor = 0;
		end else begin
			if (reset_count < 4'h9) begin
				reset_count = reset_count + 1;
				if (io0_delayed == 1'b1) begin
				    reset_monitor = reset_monitor + 1;
				end
			end
		end
	end

	always @(csb, clk) begin
		spi_io_vld = 0;
		if (!csb && !clk) begin
			if (dummycount > 0) begin
				io0_oe = 0;
				io1_oe = 0;
				io2_oe = 0;
				io3_oe = 0;
			end else
			case (mode)
				mode_spi: begin
					io0_oe = 0;
					io1_oe = 1;
					io2_oe = 0;
					io3_oe = 0;
					io1_dout = buffer[7];
				end
				mode_dspi_rd: begin
					io0_oe = 0;
					io1_oe = 0;
					io2_oe = 0;
					io3_oe = 0;
				end
				mode_dspi_wr: begin
					io0_oe = 1;
					io1_oe = 1;
					io2_oe = 0;
					io3_oe = 0;
					io0_dout = buffer[6];
					io1_dout = buffer[7];
				end
				mode_qspi_rd: begin
					io0_oe = 0;
					io1_oe = 0;
					io2_oe = 0;
					io3_oe = 0;
				end
				mode_qspi_wr: begin
					io0_oe = 1;
					io1_oe = 1;
					io2_oe = 1;
					io3_oe = 1;
					io0_dout = buffer[4];
					io1_dout = buffer[5];
					io2_dout = buffer[6];
					io3_dout = buffer[7];
				end
				mode_qspi_ddr_rd: begin
					ddr_rd_edge;
				end
				mode_qspi_ddr_wr: begin
					ddr_wr_edge;
				end
			endcase
			if (next_mode) begin
				case (next_mode)
					mode_qspi_ddr_rd: begin
						io0_oe = 0;
						io1_oe = 0;
						io2_oe = 0;
						io3_oe = 0;
					end
					mode_qspi_ddr_wr: begin
						io0_oe = 1;
						io1_oe = 1;
						io2_oe = 1;
						io3_oe = 1;
						io0_dout = buffer[4];
						io1_dout = buffer[5];
						io2_dout = buffer[6];
						io3_dout = buffer[7];
					end
				endcase
				mode = next_mode;
				next_mode = 0;
			end
		end
	end

	always @(posedge clk) begin
		if (!csb) begin
			if (dummycount > 0) begin
				dummycount = dummycount - 1;
			end else
			case (mode)
				mode_spi: begin
					buffer = {buffer, io0};
					bitcount = bitcount + 1;
					if (bitcount == 8) begin
						bitcount = 0;
						bytecount = bytecount + 1;
						spi_action;
					end
				end
				mode_dspi_rd, mode_dspi_wr: begin
					buffer = {buffer, io1, io0};
					bitcount = bitcount + 2;
					if (bitcount == 8) begin
						bitcount = 0;
						bytecount = bytecount + 1;
						spi_action;
					end
				end
				mode_qspi_rd, mode_qspi_wr: begin
					buffer = {buffer, io3, io2, io1, io0};
					bitcount = bitcount + 4;
					if (bitcount == 8) begin
						bitcount = 0;
						bytecount = bytecount + 1;
						spi_action;
					end
				end
				mode_qspi_ddr_rd: begin
					ddr_rd_edge;
				end
				mode_qspi_ddr_wr: begin
					ddr_wr_edge;
				end
			endcase
		end
	end
endmodule
