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

	//RGMII bus to second off-chip PHY (eth2)
	output wire			eth2_tx_clk,
	output wire			eth2_tx_ctl,
	output wire[3:0]	eth2_txd,
	input wire			eth2_rx_clk,
	input wire			eth2_rx_ctl,
	input wire[3:0]		eth2_rxd,
	output wire			eth2_mdc,
	inout wire			eth2_mdio,
	output logic		eth2_rst_n		= 0,

	//RGMII bus to third off-chip PHY (eth3)
	output wire			eth3_tx_clk,
	output wire			eth3_tx_ctl,
	output wire[3:0]	eth3_txd,
	input wire			eth3_rx_clk,
	input wire			eth3_rx_ctl,
	input wire[3:0]		eth3_rxd,
	output wire			eth3_mdc,
	inout wire			eth3_mdio,
	output logic		eth3_rst_n		= 0,

	//LEDs for on-module PHY
	output logic[1:0]	eth0_led		= 2'b10,

	//Level shifting of LEDs to 1.8V PHY
	input wire[1:0]		eth3_led_1v8_n,
	output wire[1:0]	eth3_led_n,

	//I2C bus to MAC addr eeprom
	inout wire			i2c_sda,
	output wire			i2c_scl,

	//Debug LEDs
	output logic[3:0]	led				= 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Level shifting

	assign eth3_led_n = eth3_led_1v8_n;

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
			eth2_rst_n		<= 1;
			eth3_rst_n		<= 1;
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
		//.clkdiv(16'd250),		//400 kHz
		.clkdiv(16'd150),		//666 kHz
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
	wire		mac_addr_ready;
	wire		mac_addr_fail;
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
		.ready(mac_addr_ready),
		.fail(mac_addr_fail),
		.mac(mac_addr)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MDIO controllers for talking to the PHYs

	wire	eth0_mdio_tx_data;
	wire	eth0_mdio_tx_en;
	wire	eth0_mdio_rx_data;
	BidirectionalBuffer eth0_mdio_obuf(
		.fabric_in(eth0_mdio_rx_data),
		.fabric_out(eth0_mdio_tx_data),
		.oe(eth0_mdio_tx_en),
		.pad(eth0_mdio)
	);

	wire[4:0]	eth0_phy_reg_addr;
	wire[15:0]	eth0_phy_wr_data;
	wire[15:0]	eth0_phy_rd_data;
	wire		eth0_phy_reg_wr;
	wire		eth0_phy_reg_rd;

	wire		eth0_wr;
	logic		eth0_wr_ff	= 0;

	wire		eth0_rd;
	logic		eth0_rd_ff	= 0;

	always_ff @(posedge clk_125mhz) begin
		eth0_wr_ff	<= eth0_wr;
		eth0_rd_ff	<= eth0_rd;
	end

	assign eth0_phy_reg_wr = eth0_wr && !eth0_wr_ff;
	assign eth0_phy_reg_rd = eth0_rd && !eth0_rd_ff;

	EthernetMDIOTransceiver eth0_mdio_txvr(
		.clk_125mhz(clk_125mhz),
		.phy_md_addr(5'h0),
		.mdio_tx_data(eth0_mdio_tx_data),
		.mdio_tx_en(eth0_mdio_tx_en),
		.mdio_rx_data(eth0_mdio_rx_data),
		.mdc(eth0_mdc),

		.mgmt_busy_fwd(),
		.phy_reg_addr(eth0_phy_reg_addr),
		.phy_wr_data(eth0_phy_wr_data),
		.phy_rd_data(eth0_phy_rd_data),
		.phy_reg_wr(eth0_phy_reg_wr),
		.phy_reg_rd(eth0_phy_reg_rd)
	);

	wire	eth1_mdio_tx_data;
	wire	eth1_mdio_tx_en;
	wire	eth1_mdio_rx_data;
	BidirectionalBuffer eth1_mdio_obuf(
		.fabric_in(eth1_mdio_rx_data),
		.fabric_out(eth1_mdio_tx_data),
		.oe(eth1_mdio_tx_en),
		.pad(eth1_mdio)
	);

	wire[4:0]	eth1_phy_reg_addr;
	wire[15:0]	eth1_phy_wr_data;
	wire[15:0]	eth1_phy_rd_data;
	wire		eth1_phy_reg_wr;
	wire		eth1_phy_reg_rd;

	wire		eth1_wr;
	logic		eth1_wr_ff	= 0;

	wire		eth1_rd;
	logic		eth1_rd_ff	= 0;

	always_ff @(posedge clk_125mhz) begin
		eth1_wr_ff	<= eth1_wr;
		eth1_rd_ff	<= eth1_rd;
	end

	assign eth1_phy_reg_wr = eth1_wr && !eth1_wr_ff;
	assign eth1_phy_reg_rd = eth1_rd && !eth1_rd_ff;

	wire[4:0] eth1_phy_md_addr;

	EthernetMDIOTransceiver eth1_mdio_txvr(
		.clk_125mhz(clk_125mhz),
		.phy_md_addr(eth1_phy_md_addr),
		.mdio_tx_data(eth1_mdio_tx_data),
		.mdio_tx_en(eth1_mdio_tx_en),
		.mdio_rx_data(eth1_mdio_rx_data),
		.mdc(eth1_mdc),

		.mgmt_busy_fwd(),
		.phy_reg_addr(eth1_phy_reg_addr),
		.phy_wr_data(eth1_phy_wr_data),
		.phy_rd_data(eth1_phy_rd_data),
		.phy_reg_wr(eth1_phy_reg_wr),
		.phy_reg_rd(eth1_phy_reg_rd)
	);

	wire	eth2_mdio_tx_data;
	wire	eth2_mdio_tx_en;
	wire	eth2_mdio_rx_data;
	BidirectionalBuffer eth2_mdio_obuf(
		.fabric_in(eth2_mdio_rx_data),
		.fabric_out(eth2_mdio_tx_data),
		.oe(eth2_mdio_tx_en),
		.pad(eth2_mdio)
	);

	wire[4:0]	eth2_phy_reg_addr;
	wire[15:0]	eth2_phy_wr_data;
	wire[15:0]	eth2_phy_rd_data;
	wire		eth2_phy_reg_wr;
	wire		eth2_phy_reg_rd;

	wire		eth2_wr;
	logic		eth2_wr_ff	= 0;

	wire		eth2_rd;
	logic		eth2_rd_ff	= 0;

	always_ff @(posedge clk_125mhz) begin
		eth2_wr_ff	<= eth2_wr;
		eth2_rd_ff	<= eth2_rd;
	end

	assign eth2_phy_reg_wr = eth2_wr && !eth2_wr_ff;
	assign eth2_phy_reg_rd = eth2_rd && !eth2_rd_ff;

	EthernetMDIOTransceiver eth2_mdio_txvr(
		.clk_125mhz(clk_125mhz),
		.phy_md_addr(5'h0),
		.mdio_tx_data(eth2_mdio_tx_data),
		.mdio_tx_en(eth2_mdio_tx_en),
		.mdio_rx_data(eth2_mdio_rx_data),
		.mdc(eth2_mdc),

		.mgmt_busy_fwd(),
		.phy_reg_addr(eth2_phy_reg_addr),
		.phy_wr_data(eth2_phy_wr_data),
		.phy_rd_data(eth2_phy_rd_data),
		.phy_reg_wr(eth2_phy_reg_wr),
		.phy_reg_rd(eth2_phy_reg_rd)
	);

	wire	eth3_mdio_tx_data;
	wire	eth3_mdio_tx_en;
	wire	eth3_mdio_rx_data;
	BidirectionalBuffer eth3_mdio_obuf(
		.fabric_in(eth3_mdio_rx_data),
		.fabric_out(eth3_mdio_tx_data),
		.oe(eth3_mdio_tx_en),
		.pad(eth3_mdio)
	);

	wire[4:0]	eth3_phy_reg_addr;
	wire[15:0]	eth3_phy_wr_data;
	wire[15:0]	eth3_phy_rd_data;
	wire		eth3_phy_reg_wr;
	wire		eth3_phy_reg_rd;

	wire		eth3_wr;
	logic		eth3_wr_ff	= 0;

	wire		eth3_rd;
	logic		eth3_rd_ff	= 0;

	always_ff @(posedge clk_125mhz) begin
		eth3_wr_ff	<= eth3_wr;
		eth3_rd_ff	<= eth3_rd;
	end

	assign eth3_phy_reg_wr = eth3_wr && !eth3_wr_ff;
	assign eth3_phy_reg_rd = eth3_rd && !eth3_rd_ff;

	EthernetMDIOTransceiver eth3_mdio_txvr(
		.clk_125mhz(clk_125mhz),
		.phy_md_addr(5'h0),
		.mdio_tx_data(eth3_mdio_tx_data),
		.mdio_tx_en(eth3_mdio_tx_en),
		.mdio_rx_data(eth3_mdio_rx_data),
		.mdc(eth3_mdc),

		.mgmt_busy_fwd(),
		.phy_reg_addr(eth3_phy_reg_addr),
		.phy_wr_data(eth3_phy_wr_data),
		.phy_rd_data(eth3_phy_rd_data),
		.phy_reg_wr(eth3_phy_reg_wr),
		.phy_reg_rd(eth3_phy_reg_rd)
	);


	vio_0 vio(
		.clk(clk_125mhz),
		.probe_in0(eth0_phy_rd_data),
		.probe_in1(eth1_phy_rd_data),
		.probe_in2(eth2_phy_rd_data),
		.probe_in3(eth3_phy_rd_data),
		.probe_out0(eth0_phy_reg_addr),
		.probe_out1(eth0_phy_wr_data),
		.probe_out2(eth0_wr),
		.probe_out3(eth0_rd),
		.probe_out4(eth1_phy_reg_addr),
		.probe_out5(eth1_phy_wr_data),
		.probe_out6(eth1_wr),
		.probe_out7(eth1_rd),
		.probe_out8(eth2_phy_reg_addr),
		.probe_out9(eth2_phy_wr_data),
		.probe_out10(eth2_wr),
		.probe_out11(eth2_rd),
		.probe_out12(eth3_phy_reg_addr),
		.probe_out13(eth3_phy_wr_data),
		.probe_out14(eth3_wr),
		.probe_out15(eth3_rd),
		.probe_out16(eth1_phy_md_addr)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MAC and RGMII bridge for all Ethernet interfaces

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

	//Drive left LED to link state, right is RX activity state (TODO include TX)
	logic[23:0] bcount = 0;
	always_ff @(posedge eth0_rx_clk) begin
		eth0_led[1]	<= eth0_link_up;

		if(eth0_mac_rx_bus.start) begin
			eth0_led[0]	<= 1;
			bcount		<= 1;
		end

		if(bcount == 0)
			eth0_led[0]	<= 0;
		else
			bcount <= bcount + 1;
	end

	always_comb begin
		led[0]	<= eth0_link_up;
		led[1]	<= eth1_link_up;
		led[2]	<= eth2_link_up;
		led[3]	<= eth3_link_up;
	end

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

	wire				eth2_link_up;
	wire				eth2_mac_rx_clk;
	EthernetRxBus		eth2_mac_rx_bus;
	EthernetTxBus		eth2_mac_tx_bus;
	wire				eth2_mac_tx_ready;

	RGMIIMACWrapper eth2_mac(
		.rgmii_rxc(eth2_rx_clk),
		.rgmii_rxd(eth2_rxd),
		.rgmii_rx_ctl(eth2_rx_ctl),

		.rgmii_txc(eth2_tx_clk),
		.rgmii_txd(eth2_txd),
		.rgmii_tx_ctl(eth2_tx_ctl),

		.clk_125mhz(clk_125mhz),
		.clk_250mhz(clk_250mhz),

		.mac_rx_clk(eth2_mac_rx_clk),
		.mac_rx_bus(eth2_mac_rx_bus),
		.mac_tx_bus(eth2_mac_tx_bus),
		.mac_tx_ready(eth2_mac_tx_ready),

		.link_up(eth2_link_up)
	);

	wire				eth3_link_up;
	wire				eth3_mac_rx_clk;
	EthernetRxBus		eth3_mac_rx_bus;
	EthernetTxBus		eth3_mac_tx_bus;
	wire				eth3_mac_tx_ready;

	RGMIIMACWrapper eth3_mac(
		.rgmii_rxc(eth3_rx_clk),
		.rgmii_rxd(eth3_rxd),
		.rgmii_rx_ctl(eth3_rx_ctl),

		.rgmii_txc(eth3_tx_clk),
		.rgmii_txd(eth3_txd),
		.rgmii_tx_ctl(eth3_tx_ctl),

		.clk_125mhz(clk_125mhz),
		.clk_250mhz(clk_250mhz),

		.mac_rx_clk(eth3_mac_rx_clk),
		.mac_rx_bus(eth3_mac_rx_bus),
		.mac_tx_bus(eth3_mac_tx_bus),
		.mac_tx_ready(eth3_mac_tx_ready),

		.link_up(eth3_link_up)
	);

	//DEBUG
	assign eth2_mac_tx_bus = {$bits(EthernetTxBus){1'b0}};
	assign eth3_mac_tx_bus = {$bits(EthernetTxBus){1'b0}};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// IP stack for eth0 (WAN interface using the on-module PHY)

	IPv4Config ip_config;
	assign ip_config.address	= {8'd10, 8'd2, 8'd6, 8'd10};
	assign ip_config.mask		= 32'hffffff00;
	assign ip_config.gateway	= {8'd10, 8'd2, 8'd6, 8'd252};

	UDPv4RxBus	eth0_udpv4_rx_bus;
	UDPv4TxBus	eth0_udpv4_tx_bus;

	TCPIPStack #(
		.LINK_SPEED_IS_10G(0),
		.CLK_IPSTACK_HZ(100000000),
		.TX_HEADER_DEPTH(256),
		.TX_PACKET_DEPTH(4096)
	) eth0_ipstack (
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
		.udpv4_tx_bus(eth0_udpv4_tx_bus),

		//not using any TCP
		.tcpv4_rx_bus(),
		.tcpv4_tx_bus({$bits(TCPv4TxBus){1'b0}})
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SSP21 protocol engine

	`include "SSP21.svh"

	SSP21UDPServer server(
		.clk(clk_system),

		.crypto_mode(SHARED_SECRET),
		.crypto_psk(256'h0),					//all zeroes key for testing

		.udp_rx_bus(eth0_udpv4_rx_bus),
		.udp_tx_bus(eth0_udpv4_tx_bus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LA

	ila_2 tx_ila(
		.clk(clk_system),
		.probe0(eth0_ipstack.ipv4_tx_l2_bus),
		.probe1(eth0_ipstack.ipv4_tx_l3_bus),
		.probe2(eth0_ipstack.ipv4_tx_arp_bus),
		.probe3(eth0_ipstack.arp_query_en),
		.probe4(eth0_ipstack.arp_query_ip),
		.probe5(eth0_ipstack.tx_l2_bus)
	);

endmodule
