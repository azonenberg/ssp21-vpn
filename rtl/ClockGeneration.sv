`timescale 1ns / 1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* SSP21-VPN v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2019 Andrew D. Zonenberg                                                                               *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file
	@brief 	Clock synthesis
	@author	Andrew D. Zonenberg
 */
module ClockGeneration(
	input wire	clk_25mhz,

	output wire	clk_system,	//100 MHz main system clock
	output wire	clk_125mhz,	//125 MHz Ethernet clock
	output wire	clk_250mhz,	//250 MHz RGMII SERDES clock
	output wire	clk_200mhz,	//200 MHz IODELAY clock

	output wire	pll_locked
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock generation PLL

	wire[1:0]	clk_unused;

	ReconfigurablePLL #(
		.OUTPUT_BUF_GLOBAL(3'b001111),
		.OUTPUT_BUF_LOCAL(6'b000000),
		.OUTPUT_BUF_IO(6'b000000),
		.IN0_PERIOD(40),
		.IN1_PERIOD(40),
		.OUT0_MIN_PERIOD(10),	//100 MHz for system core
		.OUT1_MIN_PERIOD(8),	//125 MHz for Ethernet
		.OUT2_MIN_PERIOD(4),	//250 MHz for RGMII oversampling (DDR)
		.OUT3_MIN_PERIOD(5),	//IODELAY clock
		.OUT4_MIN_PERIOD(10),
		.OUT5_MIN_PERIOD(10),
		.OUT0_DEFAULT_PHASE(0),
		.OUT1_DEFAULT_PHASE(0),
		.OUT2_DEFAULT_PHASE(0),
		.OUT5_DEFAULT_PHASE(270),
		.ACTIVE_ON_START(1)
	) pll (
		.clkin({ clk_25mhz, clk_25mhz }),
		.clksel(1'b0),
		.clkout({clk_unused, clk_200mhz, clk_250mhz, clk_125mhz, clk_system}),
		.reset(1'b0),
		.locked(pll_locked),
		.busy(),
		.reconfig_clk(clk_25mhz),
		.reconfig_start(1'b0),
		.reconfig_finish(1'b0),
		.reconfig_cmd_done(),
		.reconfig_vco_en(1'b0),
		.reconfig_vco_mult(7'h0),
		.reconfig_vco_indiv(7'h0),
		.reconfig_vco_bandwidth(1'h0),
		.reconfig_output_en(1'h0),
		.reconfig_output_idx(3'h0),
		.reconfig_output_div(8'h0),
		.reconfig_output_phase(9'h0)
	);

endmodule
