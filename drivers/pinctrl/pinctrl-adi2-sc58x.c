/*
 * Pinctrl Driver for ADI SC58X SOC
 *
 * Copyright 2007-2018 Analog Devices Inc.
 *
 * Licensed under the GPLv2 or later
 */

#include <mach/gpio.h>
#include <mach/portmux-sc58x.h>
#include "pinctrl-adi2.h"

static const struct pinctrl_pin_desc adi_pads[] = {
	PINCTRL_PIN(0, "PA0"),
	PINCTRL_PIN(1, "PA1"),
	PINCTRL_PIN(2, "PA2"),
	PINCTRL_PIN(3, "PA3"),
	PINCTRL_PIN(4, "PA4"),
	PINCTRL_PIN(5, "PA5"),
	PINCTRL_PIN(6, "PA6"),
	PINCTRL_PIN(7, "PA7"),
	PINCTRL_PIN(8, "PA8"),
	PINCTRL_PIN(9, "PA9"),
	PINCTRL_PIN(10, "PA10"),
	PINCTRL_PIN(11, "PA11"),
	PINCTRL_PIN(12, "PA12"),
	PINCTRL_PIN(13, "PA13"),
	PINCTRL_PIN(14, "PA14"),
	PINCTRL_PIN(15, "PA15"),
	PINCTRL_PIN(16, "PB0"),
	PINCTRL_PIN(17, "PB1"),
	PINCTRL_PIN(18, "PB2"),
	PINCTRL_PIN(19, "PB3"),
	PINCTRL_PIN(20, "PB4"),
	PINCTRL_PIN(21, "PB5"),
	PINCTRL_PIN(22, "PB6"),
	PINCTRL_PIN(23, "PB7"),
	PINCTRL_PIN(24, "PB8"),
	PINCTRL_PIN(25, "PB9"),
	PINCTRL_PIN(26, "PB10"),
	PINCTRL_PIN(27, "PB11"),
	PINCTRL_PIN(28, "PB12"),
	PINCTRL_PIN(29, "PB13"),
	PINCTRL_PIN(30, "PB14"),
	PINCTRL_PIN(31, "PB15"),
	PINCTRL_PIN(32, "PC0"),
	PINCTRL_PIN(33, "PC1"),
	PINCTRL_PIN(34, "PC2"),
	PINCTRL_PIN(35, "PC3"),
	PINCTRL_PIN(36, "PC4"),
	PINCTRL_PIN(37, "PC5"),
	PINCTRL_PIN(38, "PC6"),
	PINCTRL_PIN(39, "PC7"),
	PINCTRL_PIN(40, "PC8"),
	PINCTRL_PIN(41, "PC9"),
	PINCTRL_PIN(42, "PC10"),
	PINCTRL_PIN(43, "PC11"),
	PINCTRL_PIN(44, "PC12"),
	PINCTRL_PIN(45, "PC13"),
	PINCTRL_PIN(46, "PC14"),
	PINCTRL_PIN(47, "PC15"),
	PINCTRL_PIN(48, "PD0"),
	PINCTRL_PIN(49, "PD1"),
	PINCTRL_PIN(50, "PD2"),
	PINCTRL_PIN(51, "PD3"),
	PINCTRL_PIN(52, "PD4"),
	PINCTRL_PIN(53, "PD5"),
	PINCTRL_PIN(54, "PD6"),
	PINCTRL_PIN(55, "PD7"),
	PINCTRL_PIN(56, "PD8"),
	PINCTRL_PIN(57, "PD9"),
	PINCTRL_PIN(58, "PD10"),
	PINCTRL_PIN(59, "PD11"),
	PINCTRL_PIN(60, "PD12"),
	PINCTRL_PIN(61, "PD13"),
	PINCTRL_PIN(62, "PD14"),
	PINCTRL_PIN(63, "PD15"),
	PINCTRL_PIN(64, "PE0"),
	PINCTRL_PIN(65, "PE1"),
	PINCTRL_PIN(66, "PE2"),
	PINCTRL_PIN(67, "PE3"),
	PINCTRL_PIN(68, "PE4"),
	PINCTRL_PIN(69, "PE5"),
	PINCTRL_PIN(70, "PE6"),
	PINCTRL_PIN(71, "PE7"),
	PINCTRL_PIN(72, "PE8"),
	PINCTRL_PIN(73, "PE9"),
	PINCTRL_PIN(74, "PE10"),
	PINCTRL_PIN(75, "PE11"),
	PINCTRL_PIN(76, "PE12"),
	PINCTRL_PIN(77, "PE13"),
	PINCTRL_PIN(78, "PE14"),
	PINCTRL_PIN(79, "PE15"),
	PINCTRL_PIN(80, "PF0"),
	PINCTRL_PIN(81, "PF1"),
	PINCTRL_PIN(82, "PF2"),
	PINCTRL_PIN(83, "PF3"),
	PINCTRL_PIN(84, "PF4"),
	PINCTRL_PIN(85, "PF5"),
	PINCTRL_PIN(86, "PF6"),
	PINCTRL_PIN(87, "PF7"),
	PINCTRL_PIN(88, "PF8"),
	PINCTRL_PIN(89, "PF9"),
	PINCTRL_PIN(90, "PF10"),
	PINCTRL_PIN(91, "PF11"),
	PINCTRL_PIN(92, "PF12"),
	PINCTRL_PIN(93, "PF13"),
	PINCTRL_PIN(94, "PF14"),
	PINCTRL_PIN(95, "PF15"),
	PINCTRL_PIN(96, "PG0"),
	PINCTRL_PIN(97, "PG1"),
	PINCTRL_PIN(98, "PG2"),
	PINCTRL_PIN(99, "PG3"),
	PINCTRL_PIN(100, "PG4"),
	PINCTRL_PIN(101, "PG5"),
};

static const unsigned uart0_pins[] = {
	GPIO_PC13, GPIO_PC14,
};

static const unsigned uart0_hwflow_pins[] = {
	GPIO_PC13, GPIO_PC14,
	GPIO_PC15, GPIO_PD0,
};

static const unsigned uart1_pins[] = {
	GPIO_PB2, GPIO_PB3,
};

static const unsigned uart1_hwflow_pins[] = {
	GPIO_PB2, GPIO_PB3,
	GPIO_PE1, GPIO_PE2,
};

static const unsigned uart2_pins[] = {
	GPIO_PD12, GPIO_PD13,
};

static const unsigned uart2_hwflow_pins[] = {
	GPIO_PD12, GPIO_PD13,
	GPIO_PE10, GPIO_PE11,
};

static const unsigned eth0_pins[] = {
	GPIO_PA0, GPIO_PA1, GPIO_PA2, GPIO_PA3, GPIO_PA4, GPIO_PA5,
	GPIO_PA6, GPIO_PA7, GPIO_PA8, GPIO_PA9, GPIO_PA10, GPIO_PA11,
	GPIO_PA12, GPIO_PA13,
};

static const unsigned eth0_ptp_pins[] = {
	GPIO_PA0, GPIO_PA1, GPIO_PA2, GPIO_PA3, GPIO_PA4, GPIO_PA5,
	GPIO_PA6, GPIO_PA7, GPIO_PA8, GPIO_PA9, GPIO_PA10, GPIO_PA11,
	GPIO_PA12, GPIO_PA13, GPIO_PA14, GPIO_PA15, GPIO_PB0,
	GPIO_PB1, GPIO_PB2, GPIO_PB3,
};

static const unsigned eth1_pins[] = {
	GPIO_PF13, GPIO_PF14, GPIO_PF15, GPIO_PG0, GPIO_PG1, GPIO_PG2,
	GPIO_PG3, GPIO_PG4, GPIO_PG5,
};

static const unsigned spi0_pins[] = {
	GPIO_PC9, GPIO_PC10, GPIO_PC11,
};

static const unsigned spi1_pins[] = {
	GPIO_PE13, GPIO_PE14, GPIO_PE15,
};

static const unsigned spi2_pins[] = {
	GPIO_PC1, GPIO_PC2, GPIO_PC3,
};

static const unsigned spi2_quad_pins[] = {
	GPIO_PC1, GPIO_PC2, GPIO_PC3, GPIO_PC4, GPIO_PC5,
};

static const unsigned can0_pins[] = {
	GPIO_PC7, GPIO_PC8,
};

static const unsigned can1_pins[] = {
	GPIO_PB10, GPIO_PB9,
};

static const unsigned smc0_pins[] = {
	GPIO_PA0, GPIO_PA1, GPIO_PA2, GPIO_PA3, GPIO_PA4, GPIO_PA5,
	GPIO_PA6, GPIO_PA7, GPIO_PA8, GPIO_PA9, GPIO_PA10, GPIO_PA11,
	GPIO_PA12, GPIO_PA13, GPIO_PA14, GPIO_PA15, GPIO_PB0, GPIO_PB1,
	GPIO_PB2, GPIO_PB3, GPIO_PB4, GPIO_PB5, GPIO_PB6, GPIO_PB7,
	GPIO_PB8, GPIO_PB9, GPIO_PB10, GPIO_PB11, GPIO_PB12, GPIO_PB13,
	GPIO_PB14, GPIO_PB15, GPIO_PC0,GPIO_PC7, GPIO_PC8, GPIO_PC12,
	GPIO_PC15, GPIO_PD0, GPIO_PD1, GPIO_PD12, GPIO_PD13, GPIO_PD14,
	GPIO_PD15, GPIO_PE0, GPIO_PE9, GPIO_PE10, GPIO_PE11, GPIO_PE12,
	GPIO_PE13, GPIO_PE14, GPIO_PE15,
};

static const unsigned lp0_pins[] = {
	GPIO_PD2, GPIO_PD3, GPIO_PD4, GPIO_PD5, GPIO_PD6, GPIO_PD7,
	GPIO_PD8, GPIO_PD9, GPIO_PD10, GPIO_PD11,
};

static const unsigned lp1_pins[] = {
	GPIO_PB7, GPIO_PB8, GPIO_PB9, GPIO_PB10, GPIO_PB11, GPIO_PB12,
	GPIO_PB13, GPIO_PB14, GPIO_PC0, GPIO_PB15,
};

static const unsigned ppi0_8b_pins[] = {
	GPIO_PE1, GPIO_PE2, GPIO_PE3, GPIO_PC15,
	GPIO_PE12, GPIO_PE11, GPIO_PE10, GPIO_PE9, GPIO_PE8, GPIO_PE7,
	GPIO_PE6, GPIO_PE5,
};

static const unsigned ppi0_16b_pins[] = {
	GPIO_PE1, GPIO_PE2, GPIO_PE3, GPIO_PC15,
	GPIO_PE12, GPIO_PE11, GPIO_PE10, GPIO_PE9, GPIO_PE8, GPIO_PE7,
	GPIO_PE6, GPIO_PE5, GPIO_PE4, GPIO_PE0, GPIO_PD15, GPIO_PD14,
	GPIO_PB4, GPIO_PB5, GPIO_PB0, GPIO_PB1,
};

static const unsigned ppi0_24b_pins[] = {
	GPIO_PE1, GPIO_PE2, GPIO_PE3, GPIO_PC15,
	GPIO_PE12, GPIO_PE11, GPIO_PE10, GPIO_PE9, GPIO_PE8, GPIO_PE7,
	GPIO_PE6, GPIO_PE5, GPIO_PE4, GPIO_PE0, GPIO_PD15, GPIO_PD14,
	GPIO_PB4, GPIO_PB5, GPIO_PB0, GPIO_PB1, GPIO_PB2, GPIO_PB3,
	GPIO_PD13, GPIO_PD12, GPIO_PE13, GPIO_PE14, GPIO_PE15, GPIO_PD0,
};

static const unsigned mmc0_8b_pins[] = {
	GPIO_PF2, GPIO_PF3, GPIO_PF4, GPIO_PF5, GPIO_PF6, GPIO_PF7,
	GPIO_PF8, GPIO_PF9, GPIO_PF10, GPIO_PF11, GPIO_PF12,
};

static const unsigned mmc0_4b_pins[] = {
	GPIO_PF2, GPIO_PF3, GPIO_PF4, GPIO_PF5,
	GPIO_PF10, GPIO_PF11, GPIO_PF12,
};

static const unsigned short uart0_mux[] = {
	P_UART0_TX, P_UART0_RX,
	0
};

static const unsigned short uart0_hwflow_mux[] = {
	P_UART0_TX, P_UART0_RX,
	P_UART0_RTS, P_UART0_CTS,
	0
};

static const unsigned short uart1_mux[] = {
	P_UART1_TX, P_UART1_RX,
	0
};

static const unsigned short uart1_hwflow_mux[] = {
	P_UART1_TX, P_UART1_RX,
	P_UART1_RTS, P_UART1_CTS,
	0
};

static const unsigned short uart2_mux[] = {
	P_UART2_TX, P_UART2_RX,
	0
};

static const unsigned short uart2_hwflow_mux[] = {
	P_UART2_TX, P_UART2_RX,
	P_UART2_RTS, P_UART2_CTS,
	0
};

static const unsigned short spi0_mux[] = {
	P_SPI0_SCK, P_SPI0_MISO, P_SPI0_MOSI, 0
};

static const unsigned short spi1_mux[] = {
	P_SPI1_SCK, P_SPI1_MISO, P_SPI1_MOSI, 0
};

static const unsigned short spi2_mux[] = {
	P_SPI2_SCK, P_SPI2_MISO, P_SPI2_MOSI, 0
};

static const unsigned short spi2_quad_mux[] = {
	P_SPI2_SCK, P_SPI2_MISO, P_SPI2_MOSI, P_SPI2_D2, P_SPI2_D3, 0
};

static const unsigned short can0_mux[] = {
	P_CAN0_RX, P_CAN0_TX, 0
};

static const unsigned short can1_mux[] = {
	P_CAN1_RX, P_CAN1_TX, 0
};

static const unsigned short smc0_mux[] = {
	P_SMC_A1, P_SMC_A2, P_SMC_A3, P_SMC_A4, P_SMC_A5, P_SMC_A6,
	P_SMC_A7, P_SMC_A8, P_SMC_A9, P_SMC_A10, P_SMC_A11, P_SMC_A12,
	P_SMC_A13, P_SMC_A14, P_SMC_A15, P_SMC_A16, P_SMC_A17, P_SMC_A18,
	P_SMC_A19, P_SMC_A20, P_SMC_A21, P_SMC_A22, P_SMC_A23, P_SMC_A24,
	P_SMC_A25, P_SMC_ARDY, P_SMC_D0, P_SMC_D1, P_SMC_D2, P_SMC_D3,
	P_SMC_D4, P_SMC_D5, P_SMC_D6, P_SMC_D7, P_SMC_D8, P_SMC_D9,
	P_SMC_D10, P_SMC_D11, P_SMC_D12, P_SMC_D13, P_SMC_D14, P_SMC_D15,
	P_SMC_AMS0, P_SMC_AMS1, P_SMC_AMS2, P_SMC_AMS3, P_SMC_AWE, P_SMC_ARE,
	P_SMC_AOE, P_SMC_ABE0, P_SMC_ABE1,
};

static const unsigned short lp0_mux[] = {
	P_LP0_D0, P_LP0_D1, P_LP0_D2, P_LP0_D3, P_LP0_D4,
	P_LP0_D5, P_LP0_D6, P_LP0_D7, P_LP0_CLK, P_LP0_ACK,
	0
};

static const unsigned short lp1_mux[] = {
	P_LP1_D0, P_LP1_D1, P_LP1_D2, P_LP1_D3, P_LP1_D4,
	P_LP1_D5, P_LP1_D6, P_LP1_D7, P_LP1_CLK, P_LP1_ACK,
	0
};

static const unsigned short eth0_mux[] = {
	P_ETH0_TXD0, P_ETH0_TXD1, P_ETH0_MDC, P_ETH0_MDIO, P_ETH0_RXD0,
	P_ETH0_RXD1, P_ETH0_RXCLK, P_ETH0_CRS, P_ETH0_RXD2, P_ETH0_RXD3,
	P_ETH0_TXEN, P_ETH0_TXCLK, P_ETH0_TXD2, P_ETH0_TXD3,
	0
};

static const unsigned short eth0_ptp_mux[] = {
	P_ETH0_TXD0, P_ETH0_TXD1, P_ETH0_MDC, P_ETH0_MDIO, P_ETH0_RXD0,
	P_ETH0_RXD1, P_ETH0_RXCLK, P_ETH0_CRS, P_ETH0_RXD2, P_ETH0_RXD3,
	P_ETH0_TXEN, P_ETH0_TXCLK, P_ETH0_TXD2, P_ETH0_TXD3,
	P_ETH0_PTPPPS3, P_ETH0_PTPPPS2, P_ETH0_PTPPPS1,
	P_ETH0_PTPPPS0, P_ETH0_PTPCLK0, P_ETH0_PTPAUX0,
	0
};

static const unsigned short eth1_mux[] = {
	P_ETH1_CRS, P_ETH1_MDC, P_ETH1_MDIO, P_ETH1_TXCLK, P_ETH1_TXEN,
	P_ETH1_TXD0, P_ETH1_TXD1, P_ETH1_RXD0, P_ETH1_RXD1,
	0
};

static const unsigned short ppi0_8b_mux[] = {
	P_PPI0_CLK, P_PPI0_FS1, P_PPI0_FS2, P_PPI0_FS3,
	P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3, P_PPI0_D4, P_PPI0_D5,
	P_PPI0_D6, P_PPI0_D7,
	0
};

static const unsigned short ppi0_16b_mux[] = {
	P_PPI0_CLK, P_PPI0_FS1, P_PPI0_FS2, P_PPI0_FS3,
	P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3, P_PPI0_D4, P_PPI0_D5,
	P_PPI0_D6, P_PPI0_D7, P_PPI0_D8, P_PPI0_D9, P_PPI0_D10, P_PPI0_D11,
	P_PPI0_D12, P_PPI0_D13, P_PPI0_D14, P_PPI0_D15,
	0
};

static const unsigned short ppi0_24b_mux[] = {
	P_PPI0_CLK, P_PPI0_FS1, P_PPI0_FS2, P_PPI0_FS3,
	P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3, P_PPI0_D4, P_PPI0_D5,
	P_PPI0_D6, P_PPI0_D7, P_PPI0_D8, P_PPI0_D9, P_PPI0_D10, P_PPI0_D11,
	P_PPI0_D12, P_PPI0_D13, P_PPI0_D14, P_PPI0_D15,	P_PPI0_D16, P_PPI0_D17,
	P_PPI0_D18, P_PPI0_D19, P_PPI0_D20, P_PPI0_D21, P_PPI0_D22, P_PPI0_D23,
	0
};

static const unsigned short mmc0_8b_mux[] = {
	P_MSI0_D0, P_MSI0_D1, P_MSI0_D2, P_MSI0_D3, P_MSI0_D4, P_MSI0_D5,
	P_MSI0_D6, P_MSI0_D7, P_MSI0_CMD, P_MSI0_CLK, P_MSI0_CDb,
	0
};

static const unsigned short mmc0_4b_mux[] = {
	P_MSI0_D0, P_MSI0_D1, P_MSI0_D2, P_MSI0_D3,
	P_MSI0_CMD, P_MSI0_CLK, P_MSI0_CDb,
	0
};

static const struct adi_pin_group adi_pin_groups[] = {
	ADI_PIN_GROUP("uart0grp", uart0_pins, uart0_mux),
	ADI_PIN_GROUP("uart0_hwflowgrp", uart0_hwflow_pins, uart0_hwflow_mux),
	ADI_PIN_GROUP("uart1grp", uart1_pins, uart1_mux),
	ADI_PIN_GROUP("uart1_hwflowgrp", uart1_hwflow_pins, uart1_hwflow_mux),
	ADI_PIN_GROUP("uart2grp", uart2_pins, uart2_mux),
	ADI_PIN_GROUP("uart2_hwflowgrp", uart2_hwflow_pins, uart2_hwflow_mux),
	ADI_PIN_GROUP("eth0grp", eth0_pins, eth0_mux),
	ADI_PIN_GROUP("eth0ptpgrp", eth0_ptp_pins, eth0_ptp_mux),
	ADI_PIN_GROUP("eth1grp", eth1_pins, eth1_mux),
	ADI_PIN_GROUP("spi0grp", spi0_pins, spi0_mux),
	ADI_PIN_GROUP("spi1grp", spi1_pins, spi1_mux),
	ADI_PIN_GROUP("spi2grp", spi2_pins, spi2_mux),
	ADI_PIN_GROUP("spi2quadgrp", spi2_quad_pins, spi2_quad_mux),
	ADI_PIN_GROUP("can0grp", can0_pins, can0_mux),
	ADI_PIN_GROUP("can1grp", can1_pins, can1_mux),
	ADI_PIN_GROUP("smc0grp", smc0_pins, smc0_mux),
	ADI_PIN_GROUP("lp0grp", lp0_pins, lp0_mux),
	ADI_PIN_GROUP("lp1grp", lp1_pins, lp1_mux),
	ADI_PIN_GROUP("ppi0_8bgrp", ppi0_8b_pins, ppi0_8b_mux),
	ADI_PIN_GROUP("ppi0_16bgrp", ppi0_16b_pins, ppi0_16b_mux),
	ADI_PIN_GROUP("ppi0_24bgrp", ppi0_24b_pins, ppi0_24b_mux),
	ADI_PIN_GROUP("mmc0_8bgrp", mmc0_8b_pins, mmc0_8b_mux),
	ADI_PIN_GROUP("mmc0_4bgrp", mmc0_4b_pins, mmc0_4b_mux),
};

static const char * const uart0grp[] = { "uart0grp",
					"uart0_hwflowgrp" };
static const char * const uart1grp[] = { "uart1grp",
					"uart1_hwflowgrp" };
static const char * const uart2grp[] = { "uart2grp",
					"uart2_hwflowgrp" };
static const char * const eth0grp[] = { "eth0grp", "eth0ptpgrp" };
static const char * const eth1grp[] = { "eth1grp" };
static const char * const spi0grp[] = { "spi0grp" };
static const char * const spi1grp[] = { "spi1grp" };
static const char * const spi2grp[] = { "spi2grp", "spi2quadgrp" };
static const char * const can0grp[] = { "can0grp" };
static const char * const can1grp[] = { "can1grp" };
static const char * const smc0grp[] = { "smc0grp" };
static const char * const ppi0grp[] = { "ppi0_8bgrp",
					"ppi0_16bgrp",
					"ppi0_24bgrp" };
static const char * const lp0grp[] = { "lp0grp" };
static const char * const lp1grp[] = { "lp1grp" };
static const char * const mmc0grp[] = { "mmc0_8bgrp",
					"mmc0_4bgrp" };

static const struct adi_pmx_func adi_pmx_functions[] = {
	ADI_PMX_FUNCTION("uart0", uart0grp),
	ADI_PMX_FUNCTION("uart1", uart1grp),
	ADI_PMX_FUNCTION("uart2", uart2grp),
	ADI_PMX_FUNCTION("spi0", spi0grp),
	ADI_PMX_FUNCTION("spi1", spi1grp),
	ADI_PMX_FUNCTION("spi2", spi2grp),
	ADI_PMX_FUNCTION("can0", can0grp),
	ADI_PMX_FUNCTION("can1", can1grp),
	ADI_PMX_FUNCTION("smc0", smc0grp),
	ADI_PMX_FUNCTION("lp0", lp0grp),
	ADI_PMX_FUNCTION("lp1", lp1grp),
	ADI_PMX_FUNCTION("eth0", eth0grp),
	ADI_PMX_FUNCTION("eth1", eth1grp),
	ADI_PMX_FUNCTION("mmc0", mmc0grp),
	ADI_PMX_FUNCTION("ppi0", ppi0grp),
};

static const struct adi_pinctrl_soc_data adi_sc58x_soc = {
	.functions = adi_pmx_functions,
	.nfunctions = ARRAY_SIZE(adi_pmx_functions),
	.groups = adi_pin_groups,
	.ngroups = ARRAY_SIZE(adi_pin_groups),
	.pins = adi_pads,
	.npins = ARRAY_SIZE(adi_pads),
};

void adi_pinctrl_soc_init(const struct adi_pinctrl_soc_data **soc)
{
	*soc = &adi_sc58x_soc;
}
