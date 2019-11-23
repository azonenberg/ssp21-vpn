set_property IOSTANDARD LVCMOS33 [get_ports {eth0_rxd[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth0_rxd[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth0_rxd[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth0_rxd[0]}]
set_property PACKAGE_PIN R8 [get_ports {eth0_rxd[3]}]
set_property PACKAGE_PIN R7 [get_ports {eth0_rxd[2]}]
set_property PACKAGE_PIN T7 [get_ports {eth0_rxd[1]}]
set_property PACKAGE_PIN N6 [get_ports {eth0_rxd[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth0_txd[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth0_txd[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth0_txd[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth0_txd[0]}]
set_property PACKAGE_PIN P9 [get_ports {eth0_txd[3]}]
set_property PACKAGE_PIN T9 [get_ports {eth0_txd[2]}]
set_property PACKAGE_PIN T10 [get_ports {eth0_txd[1]}]
set_property PACKAGE_PIN R10 [get_ports {eth0_txd[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth1_rxd[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth1_rxd[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth1_rxd[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth1_rxd[0]}]
set_property PACKAGE_PIN C9 [get_ports {eth1_rxd[3]}]
set_property PACKAGE_PIN A10 [get_ports {eth1_rxd[2]}]
set_property PACKAGE_PIN B10 [get_ports {eth1_rxd[1]}]
set_property PACKAGE_PIN B9 [get_ports {eth1_rxd[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth1_txd[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth1_txd[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth1_txd[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth1_txd[0]}]
set_property PACKAGE_PIN A12 [get_ports {eth1_txd[3]}]
set_property PACKAGE_PIN A13 [get_ports {eth1_txd[2]}]
set_property PACKAGE_PIN A14 [get_ports {eth1_txd[1]}]
set_property PACKAGE_PIN B14 [get_ports {eth1_txd[0]}]
set_property PACKAGE_PIN E12 [get_ports clk_25mhz]
set_property IOSTANDARD LVCMOS33 [get_ports clk_25mhz]
set_property IOSTANDARD LVCMOS33 [get_ports eth1_mdc]
set_property PACKAGE_PIN A8 [get_ports eth1_mdc]
set_property PACKAGE_PIN D10 [get_ports eth1_mdio]
set_property IOSTANDARD LVCMOS33 [get_ports eth1_mdio]
set_property PACKAGE_PIN D9 [get_ports eth1_rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports eth1_rst_n]
set_property PACKAGE_PIN C11 [get_ports eth1_rx_clk]
set_property IOSTANDARD LVCMOS33 [get_ports eth1_rx_clk]
set_property PACKAGE_PIN A9 [get_ports eth1_rx_ctl]
set_property IOSTANDARD LVCMOS33 [get_ports eth1_rx_ctl]
set_property PACKAGE_PIN B12 [get_ports eth1_tx_clk]
set_property IOSTANDARD LVCMOS33 [get_ports eth1_tx_clk]
set_property PACKAGE_PIN B11 [get_ports eth1_tx_ctl]
set_property IOSTANDARD LVCMOS33 [get_ports eth1_tx_ctl]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property PACKAGE_PIN H12 [get_ports {led[3]}]
set_property PACKAGE_PIN H13 [get_ports {led[2]}]
set_property PACKAGE_PIN J16 [get_ports {led[1]}]
set_property PACKAGE_PIN J15 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports eth0_mdc]
set_property PACKAGE_PIN R5 [get_ports eth0_mdc]
set_property PACKAGE_PIN T5 [get_ports eth0_mdio]
set_property IOSTANDARD LVCMOS33 [get_ports eth0_mdio]
set_property PACKAGE_PIN R6 [get_ports eth0_rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports eth0_rst_n]
set_property PACKAGE_PIN N11 [get_ports eth0_rx_clk]
set_property IOSTANDARD LVCMOS33 [get_ports eth0_rx_clk]
set_property PACKAGE_PIN P6 [get_ports eth0_rx_ctl]
set_property IOSTANDARD LVCMOS33 [get_ports eth0_rx_ctl]
set_property PACKAGE_PIN P8 [get_ports eth0_tx_clk]
set_property IOSTANDARD LVCMOS33 [get_ports eth0_tx_clk]
set_property PACKAGE_PIN T8 [get_ports eth0_tx_ctl]
set_property IOSTANDARD LVCMOS33 [get_ports eth0_tx_ctl]
set_property IOSTANDARD LVCMOS33 [get_ports i2c_scl]
set_property IOSTANDARD LVCMOS33 [get_ports i2c_sda]
set_property PACKAGE_PIN H14 [get_ports i2c_sda]
set_property PACKAGE_PIN H16 [get_ports i2c_scl]

create_clock -period 40.000 -name clk_25mhz -waveform {0.000 20.000} [get_ports clk_25mhz]
create_clock -period 8.000 -name eth0_rx_clk -waveform {0.000 4.000} [get_ports eth0_rx_clk]
create_clock -period 8.000 -name eth1_rx_clk -waveform {0.000 4.000} [get_ports eth1_rx_clk]
create_generated_clock -name clk_system -source [get_pins clkgen/pll/mmcm/CLKIN1] -master_clock clk_25mhz [get_pins clkgen/pll/mmcm/CLKOUT0]
create_generated_clock -name clk_125mhz -source [get_pins clkgen/pll/mmcm/CLKIN1] -master_clock clk_25mhz [get_pins clkgen/pll/mmcm/CLKOUT1]
create_generated_clock -name clk_250mhz -source [get_pins clkgen/pll/mmcm/CLKIN1] -master_clock clk_25mhz [get_pins clkgen/pll/mmcm/CLKOUT2]
set_clock_groups -asynchronous -group [get_clocks eth0_rx_clk] -group [get_clocks clk_125mhz]
set_clock_groups -asynchronous -group [get_clocks eth1_rx_clk] -group [get_clocks clk_125mhz]
set_clock_groups -asynchronous -group [get_clocks clk_125mhz] -group [get_clocks eth0_rx_clk]
set_clock_groups -asynchronous -group [get_clocks clk_125mhz] -group [get_clocks eth1_rx_clk]

set_clock_groups -asynchronous -group [get_clocks eth0_rx_clk] -group [get_clocks clk_system]
set_clock_groups -asynchronous -group [get_clocks clk_system] -group [get_clocks eth0_rx_clk]

set_max_delay -datapath_only -from [get_cells -hierarchical -filter { NAME =~  "*reg_a_ff*" && NAME =~  "*sync*" }] -to [get_cells -hierarchical *reg_b*] 5.000
set_max_delay -datapath_only -from [get_cells -hierarchical -filter { NAME =~  "*dout0_reg*" && NAME =~  "*sync*" }] -to [get_cells -hierarchical -filter { NAME =~  "*dout1_reg*" && NAME =~  "*sync*" }] 5.000

set_property DRIVE 4 [get_ports i2c_scl]
set_property DRIVE 4 [get_ports i2c_sda]

set_property SLEW SLOW [get_ports i2c_sda]


set_property PULLDOWN true [get_ports eth1_rx_clk]
set_property PULLDOWN true [get_ports eth0_rx_clk]
set_property PULLUP true [get_ports {eth0_rxd[3]}]
set_property PULLUP true [get_ports {eth0_rxd[2]}]
set_property PULLUP true [get_ports {eth0_rxd[1]}]
set_property PULLDOWN true [get_ports {eth0_rxd[0]}]
set_property PULLUP true [get_ports {eth1_rxd[3]}]
set_property PULLUP true [get_ports {eth1_rxd[2]}]
set_property PULLUP true [get_ports {eth1_rxd[1]}]
set_property PULLDOWN true [get_ports {eth1_rxd[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {eth2_rxd[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth2_rxd[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth2_rxd[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth2_rxd[0]}]
set_property PULLUP true [get_ports {eth2_rxd[3]}]
set_property PULLUP true [get_ports {eth2_rxd[2]}]
set_property PULLUP true [get_ports {eth2_rxd[1]}]
set_property PULLDOWN true [get_ports {eth2_rxd[0]}]
set_property PACKAGE_PIN E15 [get_ports {eth2_rxd[3]}]
set_property PACKAGE_PIN E13 [get_ports {eth2_rxd[2]}]
set_property PACKAGE_PIN D16 [get_ports {eth2_rxd[1]}]
set_property PACKAGE_PIN D15 [get_ports {eth2_rxd[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth2_txd[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth2_txd[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth2_txd[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth2_txd[0]}]
set_property PACKAGE_PIN F14 [get_ports {eth2_txd[3]}]
set_property PACKAGE_PIN F13 [get_ports {eth2_txd[2]}]
set_property PACKAGE_PIN G16 [get_ports {eth2_txd[1]}]
set_property PACKAGE_PIN G15 [get_ports {eth2_txd[0]}]
set_property PULLUP true [get_ports {eth3_rxd[3]}]
set_property PULLUP true [get_ports {eth3_rxd[2]}]
set_property PULLUP true [get_ports {eth3_rxd[1]}]
set_property PULLDOWN true [get_ports {eth3_rxd[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth3_rxd[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth3_rxd[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth3_rxd[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth3_rxd[0]}]
set_property PACKAGE_PIN K2 [get_ports {eth3_rxd[3]}]
set_property PACKAGE_PIN K5 [get_ports {eth3_rxd[2]}]
set_property PACKAGE_PIN K1 [get_ports {eth3_rxd[1]}]
set_property PACKAGE_PIN L3 [get_ports {eth3_rxd[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth3_txd[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth3_txd[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth3_txd[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth3_txd[0]}]
set_property PACKAGE_PIN J3 [get_ports {eth3_txd[3]}]
set_property PACKAGE_PIN H3 [get_ports {eth3_txd[2]}]
set_property PACKAGE_PIN H1 [get_ports {eth3_txd[1]}]
set_property PACKAGE_PIN H2 [get_ports {eth3_txd[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports eth3_mdc]
set_property IOSTANDARD LVCMOS18 [get_ports eth3_mdio]
set_property IOSTANDARD LVCMOS18 [get_ports eth3_rst_n]
set_property IOSTANDARD LVCMOS18 [get_ports eth3_rx_clk]
set_property IOSTANDARD LVCMOS18 [get_ports eth3_rx_ctl]
set_property IOSTANDARD LVCMOS18 [get_ports eth3_tx_clk]
set_property IOSTANDARD LVCMOS18 [get_ports eth3_tx_ctl]
set_property PACKAGE_PIN F5 [get_ports eth3_mdc]
set_property PACKAGE_PIN J5 [get_ports eth3_mdio]
set_property PACKAGE_PIN H5 [get_ports eth3_rst_n]
set_property PACKAGE_PIN D4 [get_ports eth3_rx_clk]
set_property PACKAGE_PIN L2 [get_ports eth3_rx_ctl]
set_property PACKAGE_PIN J1 [get_ports eth3_tx_clk]
set_property PACKAGE_PIN K3 [get_ports eth3_tx_ctl]
set_property IOSTANDARD LVCMOS33 [get_ports eth2_mdc]
set_property IOSTANDARD LVCMOS33 [get_ports eth2_mdio]
set_property IOSTANDARD LVCMOS33 [get_ports eth2_rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports eth2_rx_ctl]
set_property IOSTANDARD LVCMOS33 [get_ports eth2_rx_clk]
set_property IOSTANDARD LVCMOS33 [get_ports eth2_tx_clk]
set_property IOSTANDARD LVCMOS33 [get_ports eth2_tx_ctl]
set_property PACKAGE_PIN C16 [get_ports eth2_mdc]
set_property PACKAGE_PIN B16 [get_ports eth2_mdio]
set_property PACKAGE_PIN A15 [get_ports eth2_rst_n]
set_property PACKAGE_PIN E11 [get_ports eth2_rx_clk]
set_property PACKAGE_PIN C14 [get_ports eth2_rx_ctl]
set_property PACKAGE_PIN F15 [get_ports eth2_tx_clk]
set_property PACKAGE_PIN E16 [get_ports eth2_tx_ctl]

set_property IOSTANDARD LVCMOS18 [get_ports {eth3_led_1v8_n[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {eth3_led_1v8_n[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth3_led_n[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {eth3_led_n[0]}]
set_property PACKAGE_PIN D8 [get_ports {eth3_led_n[1]}]
set_property PACKAGE_PIN B15 [get_ports {eth3_led_n[0]}]
set_property PACKAGE_PIN G5 [get_ports {eth3_led_1v8_n[1]}]
set_property PACKAGE_PIN H4 [get_ports {eth3_led_1v8_n[0]}]

create_clock -period 8.000 -name eth2_rx_clk -waveform {0.000 4.000} [get_ports eth2_rx_clk]
create_clock -period 8.000 -name eth3_rx_clk -waveform {0.000 4.000} [get_ports eth3_rx_clk]
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk_system]

set_property PULLDOWN true [get_ports eth2_rx_clk]
set_property PULLDOWN true [get_ports eth3_rx_clk]
