/*
 * Copyright (c) 2020 Gerson Fernando Budke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SIGFOX_ST_S2LP_REGS_H_
#define ZEPHYR_DRIVERS_SIGFOX_ST_S2LP_REGS_H_

/*- Definitions ------------------------------------------------------------*/
#define S2LP_FRAME_HEADER_SIZE                     2
#define S2LP_FRAME_FOOTER_SIZE                     3
#define S2LP_FRAME_FCS_LENGTH                      2
#define S2LP_FRAME_MIN_PHR_SIZE                    5
#define S2LP_FRAME_PHR_INDEX                       1
#define S2LP_FRAME_LQI_INDEX                       2
#define S2LP_FRAME_ED_INDEX                        3
#define S2LP_FRAME_TRAC_INDEX                      4
#define S2LP_MAX_PSDU_LENGTH                     127
#define S2LP_MAX_FRAME_SIZE                      132
#define S2LP_DIG_DOMAIN_XTAL_THRESH         30000000

/*- Types ------------------------------------------------------------------*/
#define S2LP_GPIO0_CONF_REG                     0x00
#define S2LP_GPIO1_CONF_REG                     0x01
#define S2LP_GPIO2_CONF_REG                     0x02
#define S2LP_GPIO3_CONF_REG                     0x03
#define S2LP_MCU_CK_CONF_REG                    0x04
#define S2LP_SYNT3_REG                          0x05
#define S2LP_SYNT2_REG                          0x06
#define S2LP_SYNT1_REG                          0x07
#define S2LP_SYNT0_REG                          0x08
#define S2LP_IF_OFFSET_ANA_REG                  0x09
#define S2LP_IF_OFFSET_DIG_REG                  0x0A
#define S2LP_CH_SPACE_REG                       0x0C
#define S2LP_CHNUM_REG                          0x0D
#define S2LP_MOD4_REG                           0x0E
#define S2LP_MOD3_REG                           0x0F
#define S2LP_MOD2_REG                           0x10
#define S2LP_MOD1_REG                           0x11
#define S2LP_MOD0_REG                           0x12
#define S2LP_CHFLT_REG                          0x13
#define S2LP_AFC2_REG                           0x14
#define S2LP_AFC1_REG                           0x15
#define S2LP_AFC0_REG                           0x16
#define S2LP_RSSI_FLT_REG                       0x17
#define S2LP_RSSI_TH_REG                        0x18
#define S2LP_AGCCTRL5_REG                       0x19
#define S2LP_AGCCTRL4_REG                       0x1A
#define S2LP_AGCCTRL3_REG                       0x1B
#define S2LP_AGCCTRL2_REG                       0x1C
#define S2LP_AGCCTRL1_REG                       0x1D
#define S2LP_AGCCTRL0_REG                       0x1E
#define S2LP_ANT_SELECT_CONF_REG                0x1F
#define S2LP_CLOCKREC1_REG                      0x20
#define S2LP_CLOCKREC0_REG                      0x21
#define S2LP_PCKTCTRL6_REG                      0x2B
#define S2LP_PCKTCTRL5_REG                      0x2C
#define S2LP_PCKTCTRL4_REG                      0x2D
#define S2LP_PCKTCTRL3_REG                      0x2E
#define S2LP_PCKTCTRL2_REG                      0x2F
#define S2LP_PCKTCTRL1_REG                      0x30
#define S2LP_PCKTLEN1_REG                       0x31
#define S2LP_PCKTLEN0_REG                       0x32
#define S2LP_SYNC3_REG                          0x33
#define S2LP_SYNC2_REG                          0x34
#define S2LP_SYNC1_REG                          0x35
#define S2LP_SYNC0_REG                          0x36
#define S2LP_QI_REG                             0x37
#define S2LP_PCKT_PSTMBL_REG                    0x38
#define S2LP_PROTOCOL2_REG                      0x39
#define S2LP_PROTOCOL1_REG                      0x3A
#define S2LP_PROTOCOL0_REG                      0x3B
#define S2LP_FIFO_CONFIG3_REG                   0x3C
#define S2LP_FIFO_CONFIG2_REG                   0x3D
#define S2LP_FIFO_CONFIG1_REG                   0x3E
#define S2LP_FIFO_CONFIG0_REG                   0x3F
#define S2LP_PCKT_FLT_OPTIONS_REG               0x40
#define S2LP_PCKT_FLT_GOALS4_REG                0x41
#define S2LP_PCKT_FLT_GOALS3_REG                0x42
#define S2LP_PCKT_FLT_GOALS2_REG                0x43
#define S2LP_PCKT_FLT_GOALS1_REG                0x44
#define S2LP_PCKT_FLT_GOALS0_REG                0x45
#define S2LP_TIMERS5_REG                        0x46
#define S2LP_TIMERS4_REG                        0x47
#define S2LP_TIMERS3_REG                        0x48
#define S2LP_TIMERS2_REG                        0x49
#define S2LP_TIMERS1_REG                        0x4A
#define S2LP_TIMERS0_REG                        0x4B
#define S2LP_CSMA_CONF3_REG                     0x4C
#define S2LP_CSMA_CONF2_REG                     0x4D
#define S2LP_CSMA_CONF1_REG                     0x4E
#define S2LP_CSMA_CONF0_REG                     0x4F
#define S2LP_IRQ_MASK3_REG                      0x50
#define S2LP_IRQ_MASK2_REG                      0x51
#define S2LP_IRQ_MASK1_REG                      0x52
#define S2LP_IRQ_MASK0_REG                      0x53
#define S2LP_FAST_RX_TIMER_REG                  0x54
#define S2LP_PA_POWER8_REG                      0x5A
#define S2LP_PA_POWER7_REG                      0x5B
#define S2LP_PA_POWER6_REG                      0x5C
#define S2LP_PA_POWER5_REG                      0x5D
#define S2LP_PA_POWER4_REG                      0x5E
#define S2LP_PA_POWER3_REG                      0x5F
#define S2LP_PA_POWER2_REG                      0x60
#define S2LP_PA_POWER1_REG                      0x61
#define S2LP_PA_POWER0_REG                      0x62
#define S2LP_PA_CONFIG1_REG                     0x63
#define S2LP_PA_CONFIG0_REG                     0x64
#define S2LP_SYNTH_CONFIG2_REG                  0x65
#define S2LP_VCO_CONFIG_REG                     0x68
#define S2LP_VCO_CALIBR_IN2_REG                 0x69
#define S2LP_VCO_CALIBR_IN1_REG                 0x6A
#define S2LP_VCO_CALIBR_IN0_REG                 0x6B
#define S2LP_XO_RCO_CONF1_REG                   0x6C
#define S2LP_XO_RCO_CONF0_REG                   0x6D
#define S2LP_RCO_CALIBR_CONF3_REG               0x6E
#define S2LP_RCO_CALIBR_CONF2_REG               0x6F
#define S2LP_PM_CONF4_REG                       0x75
#define S2LP_PM_CONF3_REG                       0x76
#define S2LP_PM_CONF2_REG                       0x77
#define S2LP_PM_CONF1_REG                       0x78
#define S2LP_PM_CONF0_REG                       0x79
#define S2LP_MC_STATE1_REG                      0x8D
#define S2LP_MC_STATE0_REG                      0x8E
#define S2LP_TX_FIFO_STATUS_REG                 0x8F
#define S2LP_RX_FIFO_STATUS_REG                 0x90
#define S2LP_RCO_CALIBR_OUT4_REG                0x94
#define S2LP_RCO_CALIBR_OUT3_REG                0x95
#define S2LP_VCO_CALIBR_OUT1_REG                0x99
#define S2LP_VCO_CALIBR_OUT0_REG                0x9A
#define S2LP_TX_PCKT_INFO_REG                   0x9C
#define S2LP_RX_PCKT_INFO_REG                   0x9D
#define S2LP_AFC_CORR_REG                       0x9E
#define S2LP_LINK_QUALIF2_REG                   0x9F
#define S2LP_LINK_QUALIF1_REG                   0xA0
#define S2LP_RSSI_LEVEL_REG                     0xA2
#define S2LP_RX_PCKT_LEN1_REG                   0xA4
#define S2LP_RX_PCKT_LEN0_REG                   0xA5
#define S2LP_CRC_FIELD3_REG                     0xA6
#define S2LP_CRC_FIELD2_REG                     0xA7
#define S2LP_CRC_FIELD1_REG                     0xA8
#define S2LP_CRC_FIELD0_REG                     0xA9
#define S2LP_RX_REGE_FIELD1_REG                 0xAA
#define S2LP_RX_REGE_FIELD0_REG                 0xAB
#define S2LP_RSSI_LEVEL_RUN_REG                 0xEF
#define S2LP_DEVICE_INFO1_REG                   0xF0
#define S2LP_DEVICE_INFO0_REG                   0xF1
#define S2LP_IRQ_STATUS3_REG                    0xFA
#define S2LP_IRQ_STATUS2_REG                    0xFB
#define S2LP_IRQ_STATUS1_REG                    0xFC
#define S2LP_IRQ_STATUS0_REG                    0xFD
#define S2LP_FIFO_REG                           0xFF

/* GPIO_CONF */
#define S2LP_GPIO_SELECT                        0xF8
#define S2LP_GPIO_MODE                          0x03
#define S2LP_GPIO_HIGH_LEVEL			0x9A
#define S2LP_GPIO_LOW_LEVEL			0xA2

/* MCU_CK_CONF */
#define S2LP_EN_MCU_CLK                         0x80
#define S2LP_CLOCK_TAIL                         0x60
#define S2LP_XO_RATIO                           0x1E
#define S2LP_RCO_RATIO                          0x01

/* SYNT3 */
#define S2LP_PLL_CP_ISEL                        0xE0
#define S2LP_BS                                 0x10
#define S2LP_SYNT_27_24                         0x0F

/* SYNT2 */
#define S2LP_SYNT_23_16                         0xFF

/* SYNT1 */
#define S2LP_SYNT_15_8                          0xFF

/* SYNT0 */
#define S2LP_SYNT_7_0                           0xFF

/* IF_OFFSET_ANA */
#define S2LP_IF_OFFSET_ANA                      0xFF

/* IF_OFFSET_DIG */
#define S2LP_IF_OFFSET_DIG                      0xFF

/* CH_SPACE */
#define S2LP_CH_SPACE                           0xFF

/* CHNUM */
#define S2LP_CH_NUM                             0xFF

/* MOD4 */
#define S2LP_DATARATE_M_15_8                    0xFF

/* MOD3 */
#define S2LP_DATARATE_M_7_0                     0xFF

/* MOD2 */
#define S2LP_MOD_TYPE                           0xF0
#define S2LP_DATARATE_E                         0x0F

/* MOD1 */
#define S2LP_PA_INTERP_EN                       0x80
#define S2LP_MOD_INTERP_EN                      0x40
#define S2LP_G4FSK_CONST_MAP                    0x30
#define S2LP_FDEV_E                             0x0F

/* MOD0 */
#define S2LP_FDEV_M                             0xFF

/* CHFLT */
#define S2LP_CHFLT_M                            0xF0
#define S2LP_CHFLT_E                            0x0F

/* AFC2 */
#define S2LP_AFC_FREEZE_ON_SYNC                 0x80
#define S2LP_AFC_ENABLED                        0x40
#define S2LP_AFC_MODE                           0x20

/* AFC1 */
#define S2LP_AFC_FAST_PERIOD                    0xFF

/* AFC0 */
#define S2LP_AFC_FAST_GAIN                      0xF0
#define S2LP_AFC_SLOW_GAIN                      0x0F

/* RSSI_FLT */
#define S2LP_RSSI_FLT                           0xF0
#define S2LP_CS_MODE                            0x0C

/* RSSI_TH */
#define S2LP_RSSI_TH                            0xFF

/* AGCCTRL4 */
#define S2LP_LOW_THRESHOLD_0                    0xF0
#define S2LP_LOW_THRESHOLD_1                    0x0F

/* AGCCTRL3 */
#define S2LP_LOW_THRESHOLD_SEL                  0xFF

/* AGCCTRL2 */
#define S2LP_FREEZE_ON_SYNC                     0x20
#define S2LP_MEAS_TIME                          0x0F

/* AGCCTRL1 */
#define S2LP_HIGH_THRESHOLD                     0xF0

/* AGCCTRL0 */
#define S2LP_AGC_ENABLE                         0x80
#define S2LP_HOLD_TIME                          0x1F

/* ANT_SELECT_CONF */
#define S2LP_EQU_CTRL                           0x60
#define S2LP_CS_BLANKING                        0x10
#define S2LP_AS_ENABLE                          0x08
#define S2LP_AS_MEAS_TIME                       0x07

/* CLOCKREC1 */
#define S2LP_CLK_REC_P_GAIN_SLOW                0xE0
#define S2LP_CLK_REC_ALGO_SEL                   0x10
#define S2LP_CLK_REC_I_GAIN_SLOW                0x0F

/* CLOCKREC0 */
#define S2LP_CLK_REC_P_GAIN_FAST                0xE0
#define S2LP_PSTFLT_LEN                         0x10
#define S2LP_CLK_REC_I_GAIN_FAST                0x0F

/* PCKTCTRL6 */
#define S2LP_SYNC_LEN                           0xFC
#define S2LP_PREAMBLE_LEN_9_8                   0x03

/* PCKTCTRL5 */
#define S2LP_PREAMBLE_LEN_7_0                   0xFF

/* PCKTCTRL4 */
#define S2LP_LEN_WID                            0x80
#define S2LP_REGESS_LEN                         0x08

/* PCKTCTRL3 */
#define S2LP_PCKT_FRMT                          0xC0
#define S2LP_RX_MODE                            0x30
#define S2LP_FSK4_SYM_SWAP                      0x08
#define S2LP_BYTE_SWAP                          0x04
#define S2LP_PREAMBLE_SEL                       0x03

/* PCKTCTRL2 */
#define S2LP_FCS_TYPE_4G                        0x20
#define S2LP_FEC_TYPE_4G                        0x10
#define S2LP_INT_EN_4G                          0x08
#define S2LP_MBUS_3OF6_EN                       0x04
#define S2LP_MANCHESTER_EN                      0x02
#define S2LP_FIX_VAR_LEN                        0x01


/* PCKTCTRL1 */
#define S2LP_CRC_MODE                           0xE0
#define S2LP_WHIT_EN                            0x10
#define S2LP_TXSOURCE                           0x0C
#define S2LP_SECOND_SYNC_SEL                    0x02
#define S2LP_FEC_EN                             0x01

/* PCKTLEN1 */
#define S2LP_PCKTLEN1                           0xFF

/* PCKTLEN0 */
#define S2LP_PCKTLEN0                           0xFF

/* SYNC3 */
#define S2LP_SYNC3                              0xFF

/* SYNC2 */
#define S2LP_SYNC2                              0xFF

/* SYNC1 */
#define S2LP_SYNC1                              0xFF

/* SYNC0 */
#define S2LP_SYNC0                              0xFF

/* QI */
#define S2LP_SQI_TH                             0xE0
#define S2LP_PQI_TH                             0x1E
#define S2LP_SQI_EN                             0x01

/* PCKT_PSTMBL */
#define S2LP_PCKT_PSTMBL                        0xFF

/* PROTOCOL2 */
#define S2LP_CS_TIMEOUT_MASK                    0x80
#define S2LP_SQI_TIMEOUT_MASK                   0x40
#define S2LP_PQI_TIMEOUT_MASK                   0x20
#define S2LP_TX_SEQ_NUM_RELOAD                  0x18
#define S2LP_FIFO_GPIO_OUT_MUX_SEL              0x04
#define S2LP_LDC_TIMER_MULT                     0x03

/* PROTOCOL1 */
#define S2LP_LDC_MODE                           0x80
#define S2LP_LDC_RELOAD_ON_SYNC                 0x40
#define S2LP_PIGGYBACKING                       0x20
#define S2LP_FAST_CS_TERM_EN                    0x10
#define S2LP_SEED_RELOAD                        0x08
#define S2LP_CSMA_ON                            0x04
#define S2LP_CSMA_PERS_ON                       0x02
#define S2LP_AUTO_PCKT_FLT                      0x01

/* PROTOCOL0 */
#define S2LP_NMAX_RETX                          0xF0
#define S2LP_NACK_TX                            0x08
#define S2LP_AUTO_ACK                           0x04
#define S2LP_PERS_RX                            0x02

/* FIFO_CONFIG3 */
#define S2LP_RX_AFTHR                           0x7F

/* FIFO_CONFIG2 */
#define S2LP_RX_AETHR                           0x7F

/* FIFO_CONFIG1 */
#define S2LP_TX_AFTHR                           0x7F

/* FIFO_CONFIG0 */
#define S2LP_TX_AETHR                           0x7F

/* PCKT_FLT_OPTIONS */
#define S2LP_RX_TIMEOUT_AND_OR_SEL              0x40
#define S2LP_SOURCE_REG_FLT                     0x10
#define S2LP_DEST_VS_BROADCAST_REG              0x08
#define S2LP_DEST_VS_MULTICAST_REG              0x04
#define S2LP_DEST_VS_SOURCE_REG                 0x02
#define S2LP_CRC_FLT                            0x01

/* PCKT_FLT_GOALS4 */
#define S2LP_RX_SOURCE_MASK_DUAL_SYNC3          0xFF

/* PCKT_FLT_GOALS3 */
#define S2LP_RX_SOURCE_REG_DUAL_SYNC2           0xFF

/* PCKT_FLT_GOALS2 */
#define S2LP_BROADCAST_REG_DUAL_SYNC1           0xFF

/* PCKT_FLT_GOALS1 */
#define S2LP_MULTICAST_REG_DUAL_SYNC0           0xFF

/* PCKT_FLT_GOALS0 */
#define S2LP_TX_SOURCE_REG                      0xFF

/* TIMERS5 */
#define S2LP_RX_TIMER_CNTR                      0xFF

/* TIMERS4 */
#define S2LP_RX_TIMER_PRESC                     0xFF

/* TIMERS3 */
#define S2LP_LDC_TIMER_PRESC                    0xFF

/* TIMERS2 */
#define S2LP_LDC_TIMER_CNTR                     0xFF

/* TIMERS1 */
#define S2LP_LDC_RELOAD_PRSC                    0xFF

/* TIMERS0 */
#define S2LP_LDC_RELOAD_CNTR                    0xFF

/* CSMA_CONF3 */
#define S2LP_BU_CNTR_SEED_14_8                  0xFF

/* CSMA_CONF2 */
#define S2LP_BU_CNTR_SEED_7_0                   0xFF

/* CSMA_CONF1 */
#define S2LP_BU_PRSC                            0xFC
#define S2LP_CCA_PERIOD                         0x03

/* CSMA_CONF0 */
#define S2LP_CCA_LEN                            0xF0
#define S2LP_NBACKOFF_MAX                       0x07

/* IRQ_MASK3 */
#define S2LP_INT_MASK_31_24                     0xFF

/* IRQ_MASK2 */
#define S2LP_INT_MASK_23_16                     0xFF

/* IRQ_MASK1 */
#define S2LP_INT_MASK_15_8                      0xFF

/* IRQ_MASK0 */
#define S2LP_INT_MASK_7_0                       0xFF

/* FAST_RX_TIMER */
#define S2LP_RSSI_SETTLING_LIMIT                0xFF

/* PA_POWER8 */
#define S2LP_PA_LEVEL8                          0x7F

/* PA_POWER7 */
#define S2LP_PA_LEVEL_7                         0x7F

/* PA_POWER6 */
#define S2LP_PA_LEVEL_6                         0x7F

/* PA_POWER5 */
#define S2LP_PA_LEVEL_5                         0x7F

/* PA_POWER4 */
#define S2LP_PA_LEVEL_4                         0x7F

/* PA_POWER3 */
#define S2LP_PA_LEVEL_3                         0x7F

/* PA_POWER2 */
#define S2LP_PA_LEVEL_2                         0x7F

/* PA_POWER1 */
#define S2LP_PA_LEVEL_1                         0x7F

/* PA_POWER0 */
#define S2LP_DIG_SMOOTH_EN                      0x80
#define S2LP_PA_MAXDBM                          0x40
#define S2LP_PA_RAMP_EN                         0x20
#define S2LP_PA_RAMP_STEP_LEN                   0x18
#define S2LP_PA_LEVEL_MAX_IDX                   0x07

/* PA_CONFIG1 */
#define S2LP_LIN_NLOG                           0x10
#define S2LP_FIR_CFG                            0xC0
#define S2LP_FIR_EN                             0x02

/* PA_CONFIG0 */
#define S2LP_PA_DEGEN_TRIM                      0xF0
#define S2LP_PA_DEGEN_ON                        0x08
#define S2LP_PA_SAFE_ASK_CAL                    0x04
#define S2LP_PA_FC                              0x03

/* SYNTH_CONFIG2 */
#define S2LP_PLL_PFD_SPLIT_EN                   0x04

/* VCO_CONFIG */
#define S2LP_VCO_CALAMP_EXT_SEL                 0x20
#define S2LP_VCO_CALFREQ_EXT_SEL                0x20

/* VCO_CALIBR_IN2 */
#define S2LP_VCO_CALAMP_TX                      0xF0
#define S2LP_VCO_CALAMP_RX                      0x0F

/* VCO_CALIBR_IN1 */
#define S2LP_VCO_CALFREQ_TX                     0x7F

/* VCO_CALIBR_IN0 */
#define S2LP_VCO_CALFREQ_RX                     0x7F

/* XO_RCO_CONF1 */
#define S2LP_PD_CLKDIV                          0x10

/* XO_RCO_CONF0 */
#define S2LP_EXT_REF                            0x80
#define S2LP_GM_CONF                            0x70
#define S2LP_REFDIV                             0x08
#define S2LP_EXT_RCO_OSC                        0x02
#define S2LP_RCO_CALIBRATION                    0x01

/* RCO_CALIBR_CONF3 */
#define S2LP_RWT_IN                             0xF0
#define S2LP_RFB_IN_4_1                         0x0F

/* RCO_CALIBR_CONF2 */
#define S2LP_RFB_IN_0                           0x80

/* PM_CONF4 */
#define S2LP_TEMP_SENSOR_EN                     0x80
#define S2LP_TEMP_SENS_BUFF_EN                  0x40
#define S2LP_EXT_SMPS                           0x20

/* PM_CONF3 */
#define S2LP_KRM_EN                             0x80
#define S2LP_KRM_14_8                           0x7F

/* PM_CONF2 */
#define S2LP_KRM_7_0                            0xFF

/* PM_CONF1 */
#define S2LP_BATTERY_LVL_EN                     0x40
#define S2LP_SET_BLD_TH                         0x30

/* PM_CONF0 */
#define S2LP_SET_SMPS_LVL                       0x70
#define S2LP_SLEEP_MODE_SEL                     0x01

/* MC_STATE1 */
#define S2LP_RCO_CAL_OK                         0x10
#define S2LP_ANT_SEL                            0x08
#define S2LP_TX_FIFO_FULL                       0x04
#define S2LP_RX_FIFO_EMPTY                      0x02
#define S2LP_ERROR_LOCK                         0x01

/* MC_STATE0 */
#define S2LP_STATE                              0xFE
#define S2LP_XO_ON                              0x01

/* TX_FIFO_STATUS */
#define S2LP_NELEM_TXFIFO                       0x7F

/* RX_FIFO_STATUS */
#define S2LP_NELEM_RXFIFO                       0x7F

/* RCO_CALIBR_OUT4 */
#define S2LP_RWT_OUT                            0xF0
#define S2LP_RFB_OUT_4_1                        0x0F

/* RCO_CALIBR_OUT3 */
#define S2LP_RFB_OUT_0                          0x80

/* VCO_CALIBR_OUT1 */
#define S2LP_VCO_CAL_AMP_OUT                    0x0F

/* VCO_CALIBR_OUT0 */
#define S2LP_VCO_CAL_FREQ_OUT                   0x7F

/* TX_PCKT_INFO */
#define S2LP_TX_SEQ_NUM                         0x30
#define S2LP_N_RETX                             0x0F

/* RX_PCKT_INFO */
#define S2LP_NACK_RX                            0x04
#define S2LP_RX_SEQ_NUM                         0x03

/* AFC_CORR */
#define S2LP_AFC_CORR                           0xFF

/* LINK_QUALIF2 */
#define S2LP_PQI                                0xFF

/* LINK_QUALIF1 */
#define S2LP_CS                                 0x80
#define S2LP_SQI                                0x7F

/* RSSI_LEVEL */
#define S2LP_RSSI_LEVEL                         0xFF

/* RX_PCKT_LEN1 */
#define S2LP_RX_PCKT_LEN_14_8                   0xFF

/* RX_PCKT_LEN0 */
#define S2LP_RX_PCKT_LEN_7_0                    0xFF

/* CRC_FIELD3 */
#define S2LP_CRC_FIELD3                         0xFF

/* CRC_FIELD2 */
#define S2LP_CRC_FIELD2                         0xFF

/* CRC_FIELD1 */
#define S2LP_CRC_FIELD1                         0xFF

/* CRC_FIELD0 */
#define S2LP_CRC_FIELD0                         0xFF

/* RX_REGE_FIELD1 */
#define S2LP_RX_REGE_FIELD1                     0xFF

/* RX_REGE_FIELD0 */
#define S2LP_RX_REGE_FIELD0                     0xFF

/* RSSI_LEVEL_RUN */
#define S2LP_RSSI_LEVEL_RUN                     0xFF

/* DEVICE_INFO1 */
#define S2LP_PARTNUM                            0x03

/* DEVICE_INFO0 */
#define S2LP_VERSION                            0xC1

/* IRQ_STATUS3 */
#define S2LP_INT_LEVEL_31_24                    0xFF

/* IRQ_STATUS2 */
#define S2LP_INT_LEVEL_23_16                    0xFF

/* IRQ_STATUS1 */
#define S2LP_INT_LEVEL_15_8                     0xFF

/* IRQ_STATUS0 */
#define S2LP_INT_LEVEL_7_0                      0xFF

#endif /* ZEPHYR_DRIVERS_SIGFOX_ST_S2LP_REGS_H_ */
