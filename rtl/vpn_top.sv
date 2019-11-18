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
	@brief 	Top level of the VPN system
	@author	Andrew D. Zonenberg
 */
module vpn_top(

	//Reference clock from module
	input wire			clk_25mhz,

	//RGMII bus to on-chip PHY (eth0)
	output wire			eth0_tx_clk,
	output wire			eth0_tx_ctl,
	output wire[3:0]	eth0_txd,
	input wire			eth0_rx_clk,
	input wire			eth0_rx_ctl,
	input wire[3:0]		eth0_rxd,
	output wire			eth0_mdc,
	inout wire			eth0_mdio,
	output logic		eth0_rst_n		= 0,

	//RGMII bus to first off-chip PHY (eth1)
	output wire			eth1_tx_clk,
	output wire			eth1_tx_ctl,
	output wire[3:0]	eth1_txd,
	input wire			eth1_rx_clk,
	input wire			eth1_rx_ctl,
	input wire[3:0]		eth1_rxd,
	output wire			eth1_mdc,
	inout wire			eth1_mdio,
	output logic		eth1_rst_n		= 0,

	//I2C bus to MAC addr eeprom
	inout wire			i2c_sda,
	output wire			i2c_scl,

	//Debug LEDs
	output logic[3:0]	led				= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock synthesis

	wire	clk_system;
	wire	clk_125mhz;
	wire	clk_250mhz;
	wire	clk_200mhz;

	wire	pll_locked;

	ClockGeneration clkgen(
		.clk_25mhz(clk_25mhz),

		.clk_system(clk_system),
		.clk_125mhz(clk_125mhz),
		.clk_250mhz(clk_250mhz),
		.clk_200mhz(clk_200mhz),
		.pll_locked(pll_locked)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// I/O delay calibration

	IODelayCalibration cal(.refclk(clk_200mhz));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Reset control

	//Bring up the PHY after a little while
	logic[15:0] eth_rst_count = 1;
	always_ff @(posedge clk_25mhz) begin
		if(eth_rst_count == 0) begin
			eth0_rst_n		<= 1;
			eth1_rst_n		<= 1;
		end
		else
			eth_rst_count	<= eth_rst_count + 1'h1;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// I2C bus for module-side peripherals

	`include "I2CTransceiver.svh"

	i2c_in_t txvr_cin;
	i2c_out_t txvr_cout;

	I2CTransceiver i2c_txvr(
		.clk(clk_system),
		.clkdiv(16'd250),		//400 kHz
		.i2c_scl(i2c_scl),
		.i2c_sda(i2c_sda),
		.cin(txvr_cin),
		.cout(txvr_cout)
	);

	i2c_in_t	mac_driver_cin;
	i2c_out_t	mac_driver_cout;

	wire		mac_driver_request;
	wire		mac_driver_done;
	wire		mac_driver_ack;

	I2CArbiter #(
		.NUM_PORTS(1)
	) i2c_arbiter(
		.clk(clk_system),

		.driver_request(mac_driver_request),
		.driver_done(mac_driver_done),
		.driver_ack(mac_driver_ack),
		.driver_cin(mac_driver_cin),
		.driver_cout(mac_driver_cout),

		.txvr_cin(txvr_cin),
		.txvr_cout(txvr_cout)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MAC address generator

	wire		mac_addr_done;
	wire[47:0]	mac_addr;

	//Read from the EEPROM
	I2CMACAddressReader mac_reader(
		.clk(clk_system),

		.driver_req(mac_driver_request),
		.driver_ack(mac_driver_ack),
		.driver_done(mac_driver_done),
		.driver_cin(mac_driver_cin),
		.driver_cout(mac_driver_cout),

		.done(mac_addr_done),
		.mac(mac_addr)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MAC and RGMII bridge for both Ethernet interfaces

	`include "GmiiBus.svh"
	`include "EthernetBus.svh"
	`include "IPv4Bus.svh"
	`include "UDPv4Bus.svh"
	`include "TCPv4Bus.svh"

	wire				eth0_link_up;
	wire				eth0_mac_rx_clk;
	EthernetRxBus		eth0_mac_rx_bus;
	EthernetTxBus		eth0_mac_tx_bus;
	wire				eth0_mac_tx_ready;

	RGMIIMACWrapper eth0_mac(
		.rgmii_rxc(eth0_rx_clk),
		.rgmii_rxd(eth0_rxd),
		.rgmii_rx_ctl(eth0_rx_ctl),

		.rgmii_txc(eth0_tx_clk),
		.rgmii_txd(eth0_txd),
		.rgmii_tx_ctl(eth0_tx_ctl),

		.clk_125mhz(clk_125mhz),
		.clk_250mhz(clk_250mhz),

		.mac_rx_clk(eth0_mac_rx_clk),
		.mac_rx_bus(eth0_mac_rx_bus),
		.mac_tx_bus(eth0_mac_tx_bus),
		.mac_tx_ready(eth0_mac_tx_ready),

		.link_up(eth0_link_up)
	);

	wire				eth1_link_up;
	wire				eth1_mac_rx_clk;
	EthernetRxBus		eth1_mac_rx_bus;
	EthernetTxBus		eth1_mac_tx_bus;
	wire				eth1_mac_tx_ready;

	RGMIIMACWrapper eth1_mac(
		.rgmii_rxc(eth1_rx_clk),
		.rgmii_rxd(eth1_rxd),
		.rgmii_rx_ctl(eth1_rx_ctl),

		.rgmii_txc(eth1_tx_clk),
		.rgmii_txd(eth1_txd),
		.rgmii_tx_ctl(eth1_tx_ctl),

		.clk_125mhz(clk_125mhz),
		.clk_250mhz(clk_250mhz),

		.mac_rx_clk(eth1_mac_rx_clk),
		.mac_rx_bus(eth1_mac_rx_bus),
		.mac_tx_bus(eth1_mac_tx_bus),
		.mac_tx_ready(eth1_mac_tx_ready),

		.link_up(eth1_link_up)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// IP stack for eth0

	IPv4Config ip_config;
	assign ip_config.address	= {8'd10, 8'd2, 8'd6, 8'd10};
	assign ip_config.mask		= 32'hffffff00;
	assign ip_config.gateway	= {8'd10, 8'd2, 8'd6, 8'd252};

	UDPv4RxBus	eth0_udpv4_rx_bus;
	UDPv4TxBus	eth0_udpv4_tx_bus = {$bits(UDPv4TxBus){1'b0}};

	TCPIPStack #(
		.LINK_SPEED_IS_10G(0),
		.CLK_IPSTACK_HZ(100000000)
	) ipstack (
		.clk_ipstack(clk_system),

		.ip_config(ip_config),
		.mac_address(mac_addr),
		.promisc_mode(1'b0),
		.config_update(mac_addr_done),

		.mac_rx_clk(eth0_mac_rx_clk),
		.mac_rx_bus(eth0_mac_rx_bus),
		.mac_tx_clk(clk_125mhz),
		.mac_tx_bus(eth0_mac_tx_bus),
		.mac_tx_ready(eth0_mac_tx_ready),

		.udpv4_rx_bus(eth0_udpv4_rx_bus),
		.udpv4_tx_bus(eth0_udpv4_tx_bus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LA

	/*
	ila_0 ila(
		.clk(clk_system),
		.probe0(reader.driver_req),
		.probe1(reader.driver_ack),
		.probe2(reader.driver_done),
		.probe3(reader.done),
		.probe4(reader.mac),
		.probe5(reader.open),
		.probe6(reader.ready),
		.probe7(reader.select),
		.probe8(reader.rdata_valid),
		.probe9(reader.rdata),
		.probe10(reader.err),
		.probe11(reader.burst_done),
		.probe12(reader.state)
	);
	*/

endmodule
