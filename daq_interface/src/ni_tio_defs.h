#define Gi_Auto_Increment_Mask 		0xff
#define Gi_Up_Down_Shift 		5

#define Gi_Arm_Bit			0x1
#define Gi_Save_Trace_Bit		0x2
#define Gi_Load_Bit			0x4
#define Gi_Disarm_Bit			0x10
#define Gi_Up_Down_Mask			(0x3 << Gi_Up_Down_Shift)
#define Gi_Always_Down_Bits		(0x0 << Gi_Up_Down_Shift)
#define Gi_Always_Up_Bits		(0x1 << Gi_Up_Down_Shift)
#define Gi_Up_Down_Hardware_IO_Bits	(0x2 << Gi_Up_Down_Shift)
#define Gi_Up_Down_Hardware_Gate_Bits	(0x3 << Gi_Up_Down_Shift)
#define Gi_Write_Switch_Bit		0x80
#define Gi_Synchronize_Gate_Bit		0x100
#define Gi_Little_Big_Endian_Bit	0x200
#define Gi_Bank_Switch_Start_Bit	0x400
#define Gi_Bank_Switch_Mode_Bit		0x800
#define Gi_Bank_Switch_Enable_Bit	0x1000
#define Gi_Arm_Copy_Bit			0x2000
#define Gi_Save_Trace_Copy_Bit		0x4000
#define Gi_Disarm_Copy_Bit		0x8000

#define Gi_Index_Phase_Bitshift 	5
#define Gi_HW_Arm_Select_Shift		8

#define Gi_Counting_Mode_Mask		0x7
#define Gi_Counting_Mode_Normal_Bits	0x0
#define Gi_Counting_Mode_QuadratureX1_Bits 0x1
#define Gi_Counting_Mode_QuadratureX2_Bits 0x2
#define Gi_Counting_Mode_QuadratureX4_Bits 0x3
#define Gi_Counting_Mode_Two_Pulse_Bits	0x4
#define Gi_Counting_Mode_Sync_Source_Bits 0x6
#define Gi_Index_Mode_Bit		0x10
#define Gi_Index_Phase_Mask		(0x3 << Gi_Index_Phase_Bitshift)
#define Gi_Index_Phase_LowA_LowB	(0x0 << Gi_Index_Phase_Bitshift)
#define Gi_Index_Phase_LowA_HighB	(0x1 << Gi_Index_Phase_Bitshift)
#define Gi_Index_Phase_HighA_LowB	(0x2 << Gi_Index_Phase_Bitshift)
#define Gi_Index_Phase_HighA_HighB	(0x3 << Gi_Index_Phase_Bitshift)

/* From m-series example code,
   not documented in 660x register level manual */
#define Gi_HW_Arm_Enable_Bit		0x80
/* From m-series example code,
   not documented in 660x register level manual */
#define Gi_660x_HW_Arm_Select_Mask	(0x7 << Gi_HW_Arm_Select_Shift)
#define Gi_660x_Prescale_X8_Bit		0x1000
#define Gi_M_Series_Prescale_X8_Bit	0x2000
#define Gi_M_Series_HW_Arm_Select_Mask	(0x1f << Gi_HW_Arm_Select_Shift)
/* Must be set for clocks over 40MHz,
   which includes synchronous counting and quadrature modes */
#define Gi_660x_Alternate_Sync_Bit	0x2000
#define Gi_M_Series_Alternate_Sync_Bit	0x4000
/* From m-series example code,
   not documented in 660x register level manual */
#define Gi_660x_Prescale_X2_Bit		0x4000
#define Gi_M_Series_Prescale_X2_Bit	0x8000


#define NI_660x_Timebase_1_Clock	0x0 /* 20MHz */
#define NI_660x_Source_Pin_i_Clock	0x1
#define NI_660x_Next_Gate_Clock		0xa
#define NI_660x_Timebase_2_Clock	0x12 /* 100KHz */
#define NI_660x_Next_TC_Clock		0x13
#define NI_660x_Timebase_3_Clock	0x1e /* 80MHz */
#define NI_660x_Logic_Low_Clock		0x1f

#define ni_660x_max_rtsi_channel	6
#define ni_660x_max_source_pin		7

/* Clock sources for ni e and m series boards,
   get bits with Gi_Source_Select_Bits() */
#define NI_M_Series_Timebase_1_Clock	0x0 /* 20MHz */
#define NI_M_Series_Timebase_2_Clock	0x12 /* 100KHz */
#define NI_M_Series_Next_TC_Clock	0x13
#define NI_M_Series_Next_Gate_Clock	0x14 /* when Gi_Src_SubSelect = 0 */
#define NI_M_Series_PXI_Star_Trigger_Clock 0x14 /* when Gi_Src_SubSelect = 1 */
#define NI_M_Series_PXI10_Clock		0x1d
#define NI_M_Series_Timebase_3_Clock	0x1e /* 80MHz, when Gi_Src_SubSelect = 0 */
#define NI_M_Series_Analog_Trigger_Out_Clock 0x1e /* when Gi_Src_SubSelect = 1 */
#define NI_M_Series_Logic_Low_Clock	0x1f

#define ni_m_series_max_pfi_channel	15
#define ni_m_series_max_rtsi_channel	7

#define NI_660x_Source_Pin_i_Gate_Select 0x0
#define NI_660x_Gate_Pin_i_Gate_Select	0x1
#define NI_660x_Next_SRC_Gate_Select	0xa
#define NI_660x_Next_Out_Gate_Select	0x14
#define NI_660x_Logic_Low_Gate_Select	0x1f
#define ni_660x_max_gate_pin 7


#define NI_M_Series_Timestamp_Mux_Gate_Select	0x0
#define NI_M_Series_AI_START2_Gate_Select	0x12
#define NI_M_Series_PXI_Star_Trigger_Gate_Select 0x13
#define NI_M_Series_Next_Out_Gate_Select	0x14
#define NI_M_Series_AI_START1_Gate_Select	0x1c
#define NI_M_Series_Next_SRC_Gate_Select	0x1d
#define NI_M_Series_Analog_Trigger_Out_Gate_Select 0x1e
#define NI_M_Series_Logic_Low_Gate_Select	0x1f


#define Gi_Source_Select_Shift 2
#define Gi_Gate_Select_Shift 7

#define Gi_Read_Acknowledges_Irq	0x1 /* not present on 660x */
#define Gi_Write_Acknowledges_Irq	0x2 /* not present on 660x */
#define Gi_Source_Select_Mask		0x7c
#define Gi_Gate_Select_Mask		(0x1f << Gi_Gate_Select_Shift)
#define Gi_Gate_Select_Load_Source_Bit	0x1000
#define Gi_Or_Gate_Bit			0x2000
#define Gi_Output_Polarity_Bit		0x4000 /* set to invert */
#define Gi_Source_Polarity_Bit		0x8000 /* set to invert */

#define Gi_Source_Select_Bits(x) ((x << Gi_Source_Select_Shift) & \
				  Gi_Source_Select_Mask)
#define Gi_Gate_Select_Bits(x) ((x << Gi_Gate_Select_Shift) & \
				Gi_Gate_Select_Mask)

#define Gi_Gating_Mode_Mask		0x3
#define Gi_Gating_Disabled_Bits		0x0
#define Gi_Level_Gating_Bits		0x1
#define Gi_Rising_Edge_Gating_Bits	0x2
#define Gi_Falling_Edge_Gating_Bits	0x3
#define Gi_Gate_On_Both_Edges_Bit	0x4 /* used in conjunction with
					       rising edge gating mode */
#define Gi_Trigger_Mode_for_Edge_Gate_Mask 0x18
#define Gi_Edge_Gate_Starts_Stops_Bits	0x0
#define Gi_Edge_Gate_Stops_Starts_Bits	0x8
#define Gi_Edge_Gate_Starts_Bits	0x10
#define Gi_Edge_Gate_No_Starts_or_Stops_Bits 0x18
#define Gi_Stop_Mode_Mask		0x60
#define Gi_Stop_on_Gate_Bits		0x00
#define Gi_Stop_on_Gate_or_TC_Bits	0x20
#define Gi_Stop_on_Gate_or_Second_TC_Bits 0x40
#define Gi_Load_Source_Select_Bit	0x80
#define Gi_Output_Mode_Mask		0x300
#define Gi_Output_TC_Pulse_Bits		0x100
#define Gi_Output_TC_Toggle_Bits	0x200
#define Gi_Output_TC_or_Gate_Toggle_Bits 0x300
#define Gi_Counting_Once_Mask		0xc00
#define Gi_No_Hardware_Disarm_Bits	0x000
#define Gi_Disarm_at_TC_Bits		0x400
#define Gi_Disarm_at_Gate_Bits		0x800
#define Gi_Disarm_at_TC_or_Gate_Bits	0xc00
#define Gi_Loading_On_TC_Bit		0x1000
#define Gi_Gate_Polarity_Bit		0x2000
#define Gi_Loading_On_Gate_Bit		0x4000
#define Gi_Reload_Source_Switching_Bit	0x8000

#define NI_660x_Source_Pin_i_Second_Gate_Select		0x0
#define NI_660x_Up_Down_Pin_i_Second_Gate_Select	0x1
#define NI_660x_Next_SRC_Second_Gate_Select		0xa
#define NI_660x_Next_Out_Second_Gate_Select		0x14
#define NI_660x_Selected_Gate_Second_Gate_Select	0x1e
#define NI_660x_Logic_Low_Second_Gate_Select		0x1f

#define ni_660x_max_up_down_pin		7

#define Gi_Second_Gate_Select_Shift	7

/*FIXME: m-series has a second gate subselect bit */
/*FIXME: m-series second gate sources are undocumented (by NI)*/
#define Gi_Second_Gate_Mode_Bit		0x1
#define Gi_Second_Gate_Select_Mask	(0x1f << Gi_Second_Gate_Select_Shift)
#define Gi_Second_Gate_Polarity_Bit	0x2000
#define Gi_Second_Gate_Subselect_Bit	0x4000 /* m-series only */
#define Gi_Source_Subselect_Bit		0x8000 /* m-series only */


#define G0_Save_Bit		0x1
#define G1_Save_Bit		0x2
#define G0_Counting_Bit		0x4
#define G1_Counting_Bit		0x8
#define G0_Next_Load_Source_Bit	0x10
#define G1_Next_Load_Source_Bit	0x20
#define G0_Stale_Data_Bit	0x40
#define G1_Stale_Data_Bit	0x80
#define G0_Armed_Bit		0x100
#define G1_Armed_Bit		0x200
#define G0_No_Load_Between_Gates_Bit 0x400
#define G1_No_Load_Between_Gates_Bit 0x800
#define G0_TC_Error_Bit		0x1000
#define G1_TC_Error_Bit		0x2000
#define G0_Gate_Error_Bit	0x4000
#define G1_Gate_Error_Bit	0x8000






#define G0_Output_Bit		0x1
#define G1_Output_Bit		0x2
#define G0_HW_Save_Bit		0x1000
#define G1_HW_Save_Bit		0x2000
#define G0_Permanent_Stale_Bit	0x4000
#define G1_Permanent_Stale_Bit	0x8000

#define Gi_DMA_Enable_Bit	0x1
#define Gi_DMA_Write_Bit	0x2
#define Gi_DMA_Int_Bit		0x4

#define Gi_DMA_Readbank_Bit	0x2000
#define Gi_DRQ_Error_Bit	0x4000
#define Gi_DRQ_Status_Bit	0x8000

#define G0_Gate_Error_Confirm_Bit	0x20
#define G0_TC_Error_Confirm_Bit		0x40

#define G1_Gate_Error_Confirm_Bit	0x2
#define G1_TC_Error_Confirm_Bit		0x4

/* Bits that are the same in G0/G2 and G1/G3 interrupt acknowledge registers */
#define Gi_TC_Interrupt_Ack_Bit		0x4000
#define Gi_Gate_Interrupt_Ack_Bit 	0x8000

#define Gi_Gate_Interrupt_Bit	0x4
#define Gi_TC_Bit		0x8
#define Gi_Interrupt_Bit	0x8000

#define G0_TC_Interrupt_Enable_Bit	0x40
#define G0_Gate_Interrupt_Enable_Bit	0x100

#define G1_TC_Interrupt_Enable_Bit	0x200
#define G1_Gate_Interrupt_Enable_Bit	0x400

#define counter_status_mask (A4L_COUNTER_ARMED | A4L_COUNTER_COUNTING)

#define NI_USUAL_PFI_SELECT(x)	((x < 10) ? (0x1 + x) : (0xb + x))
#define NI_USUAL_RTSI_SELECT(x)	((x < 7 ) ? (0xb + x) : (0x1b + x))

/* Mode bits for NI general-purpose counters, set with
   INSN_CONFIG_SET_COUNTER_MODE */
#define NI_GPCT_COUNTING_MODE_SHIFT		16
#define NI_GPCT_INDEX_PHASE_BITSHIFT 		20
#define NI_GPCT_COUNTING_DIRECTION_SHIFT	24

#define NI_GPCT_GATE_ON_BOTH_EDGES_BIT		0x4
#define NI_GPCT_EDGE_GATE_MODE_MASK		0x18
#define NI_GPCT_EDGE_GATE_STARTS_STOPS_BITS	0x0
#define NI_GPCT_EDGE_GATE_STOPS_STARTS_BITS	0x8
#define NI_GPCT_EDGE_GATE_STARTS_BITS		0x10
#define NI_GPCT_EDGE_GATE_NO_STARTS_NO_STOPS_BITS 0x18
#define NI_GPCT_STOP_MODE_MASK			0x60
#define NI_GPCT_STOP_ON_GATE_BITS		0x00
#define NI_GPCT_STOP_ON_GATE_OR_TC_BITS		0x20
#define NI_GPCT_STOP_ON_GATE_OR_SECOND_TC_BITS	0x40
#define NI_GPCT_LOAD_B_SELECT_BIT		0x80
#define NI_GPCT_OUTPUT_MODE_MASK		0x300
#define NI_GPCT_OUTPUT_TC_PULSE_BITS		0x100
#define NI_GPCT_OUTPUT_TC_TOGGLE_BITS		0x200
#define NI_GPCT_OUTPUT_TC_OR_GATE_TOGGLE_BITS	0x300
#define NI_GPCT_HARDWARE_DISARM_MASK		0xc00
#define NI_GPCT_NO_HARDWARE_DISARM_BITS		0x000
#define NI_GPCT_DISARM_AT_TC_BITS		0x400
#define NI_GPCT_DISARM_AT_GATE_BITS		0x800
#define NI_GPCT_DISARM_AT_TC_OR_GATE_BITS	0xc00
#define NI_GPCT_LOADING_ON_TC_BIT		0x1000
#define NI_GPCT_LOADING_ON_GATE_BIT		0x4000
#define NI_GPCT_COUNTING_MODE_MASK		0x7 << NI_GPCT_COUNTING_MODE_SHIFT
#define NI_GPCT_COUNTING_MODE_NORMAL_BITS	0x0 << NI_GPCT_COUNTING_MODE_SHIFT
#define NI_GPCT_COUNTING_MODE_QUADRATURE_X1_BITS 0x1 << NI_GPCT_COUNTING_MODE_SHIFT
#define NI_GPCT_COUNTING_MODE_QUADRATURE_X2_BITS 0x2 << NI_GPCT_COUNTING_MODE_SHIFT
#define NI_GPCT_COUNTING_MODE_QUADRATURE_X4_BITS 0x3 << NI_GPCT_COUNTING_MODE_SHIFT
#define NI_GPCT_COUNTING_MODE_TWO_PULSE_BITS	0x4 << NI_GPCT_COUNTING_MODE_SHIFT
#define NI_GPCT_COUNTING_MODE_SYNC_SOURCE_BITS	0x6 << NI_GPCT_COUNTING_MODE_SHIFT
#define NI_GPCT_INDEX_PHASE_MASK		0x3 << NI_GPCT_INDEX_PHASE_BITSHIFT
#define NI_GPCT_INDEX_PHASE_LOW_A_LOW_B_BITS	0x0 << NI_GPCT_INDEX_PHASE_BITSHIFT
#define NI_GPCT_INDEX_PHASE_LOW_A_HIGH_B_BITS	0x1 << NI_GPCT_INDEX_PHASE_BITSHIFT
#define NI_GPCT_INDEX_PHASE_HIGH_A_LOW_B_BITS	0x2 << NI_GPCT_INDEX_PHASE_BITSHIFT
#define NI_GPCT_INDEX_PHASE_HIGH_A_HIGH_B_BITS	0x3 << NI_GPCT_INDEX_PHASE_BITSHIFT
#define NI_GPCT_INDEX_ENABLE_BIT		0x400000
#define NI_GPCT_COUNTING_DIRECTION_MASK		0x3 << NI_GPCT_COUNTING_DIRECTION_SHIFT
#define NI_GPCT_COUNTING_DIRECTION_DOWN_BITS	0x00 << NI_GPCT_COUNTING_DIRECTION_SHIFT
#define NI_GPCT_COUNTING_DIRECTION_UP_BITS	0x1 << NI_GPCT_COUNTING_DIRECTION_SHIFT
#define NI_GPCT_COUNTING_DIRECTION_HW_UP_DOWN_BITS 0x2 << NI_GPCT_COUNTING_DIRECTION_SHIFT
#define NI_GPCT_COUNTING_DIRECTION_HW_GATE_BITS 0x3 << NI_GPCT_COUNTING_DIRECTION_SHIFT
#define NI_GPCT_RELOAD_SOURCE_MASK		0xc000000
#define NI_GPCT_RELOAD_SOURCE_FIXED_BITS	0x0
#define NI_GPCT_RELOAD_SOURCE_SWITCHING_BITS	0x4000000
#define NI_GPCT_RELOAD_SOURCE_GATE_SELECT_BITS	0x8000000
#define NI_GPCT_OR_GATE_BIT			0x10000000
#define NI_GPCT_INVERT_OUTPUT_BIT		0x20000000

/* Bits for setting a clock source with INSN_CONFIG_SET_CLOCK_SRC when
   using NI general-purpose counters. */
#define NI_GPCT_CLOCK_SRC_SELECT_MASK		0x3f
#define NI_GPCT_TIMEBASE_1_CLOCK_SRC_BITS	0x0
#define NI_GPCT_TIMEBASE_2_CLOCK_SRC_BITS	0x1
#define NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS	0x2
#define NI_GPCT_LOGIC_LOW_CLOCK_SRC_BITS	0x3
#define NI_GPCT_NEXT_GATE_CLOCK_SRC_BITS	0x4
#define NI_GPCT_NEXT_TC_CLOCK_SRC_BITS		0x5
#define NI_GPCT_SOURCE_PIN_i_CLOCK_SRC_BITS	0x6 /* NI 660x-specific */
#define NI_GPCT_PXI10_CLOCK_SRC_BITS		0x7
#define NI_GPCT_PXI_STAR_TRIGGER_CLOCK_SRC_BITS	0x8
#define NI_GPCT_ANALOG_TRIGGER_OUT_CLOCK_SRC_BITS 0x9
#define NI_GPCT_PRESCALE_MODE_CLOCK_SRC_MASK	0x30000000
#define NI_GPCT_NO_PRESCALE_CLOCK_SRC_BITS	0x0
#define NI_GPCT_PRESCALE_X2_CLOCK_SRC_BITS	0x10000000 /* divide source by 2 */
#define NI_GPCT_PRESCALE_X8_CLOCK_SRC_BITS	0x20000000 /* divide source by 8 */
#define NI_GPCT_INVERT_CLOCK_SRC_BIT		0x80000000
#define NI_GPCT_SOURCE_PIN_CLOCK_SRC_BITS(x)	(0x10 + x)
#define NI_GPCT_RTSI_CLOCK_SRC_BITS(x)		(0x18 + x)
#define NI_GPCT_PFI_CLOCK_SRC_BITS(x)		(0x20 + x)

/* Possibilities for setting a gate source with
   INSN_CONFIG_SET_GATE_SRC when using NI general-purpose counters.
   May be bitwise-or'd with CR_EDGE or CR_INVERT. */
/* M-series gates */
#define NI_GPCT_TIMESTAMP_MUX_GATE_SELECT	0x0
#define NI_GPCT_AI_START2_GATE_SELECT		0x12
#define NI_GPCT_PXI_STAR_TRIGGER_GATE_SELECT	0x13
#define NI_GPCT_NEXT_OUT_GATE_SELECT		0x14
#define NI_GPCT_AI_START1_GATE_SELECT		0x1c
#define NI_GPCT_NEXT_SOURCE_GATE_SELECT		0x1d
#define NI_GPCT_ANALOG_TRIGGER_OUT_GATE_SELECT	0x1e
#define NI_GPCT_LOGIC_LOW_GATE_SELECT		0x1f
/* More gates for 660x */
#define NI_GPCT_SOURCE_PIN_i_GATE_SELECT	0x100
#define NI_GPCT_GATE_PIN_i_GATE_SELECT		0x101
/* More gates for 660x "second gate" */
#define NI_GPCT_UP_DOWN_PIN_i_GATE_SELECT	0x201
#define NI_GPCT_SELECTED_GATE_GATE_SELECT	0x21e
/* M-series "second gate" sources are unknown, we should add them here
   with an offset of 0x300 when known. */
#define NI_GPCT_DISABLED_GATE_SELECT		0x8000
#define NI_GPCT_GATE_PIN_GATE_SELECT(x) 	(0x102 + x)
#define NI_GPCT_RTSI_GATE_SELECT(x) 		NI_USUAL_RTSI_SELECT(x)
#define NI_GPCT_PFI_GATE_SELECT(x)		NI_USUAL_PFI_SELECT(x)
#define NI_GPCT_UP_DOWN_PIN_GATE_SELECT(x)	(0x202 + x)

/* Possibilities for setting a source with INSN_CONFIG_SET_OTHER_SRC
   when using NI general-purpose counters. */
#define NI_GPCT_SOURCE_ENCODER_A 0
#define NI_GPCT_SOURCE_ENCODER_B 1
#define NI_GPCT_SOURCE_ENCODER_Z 2
/* M-series gates */
/* Still unknown, probably only need NI_GPCT_PFI_OTHER_SELECT */
#define NI_GPCT_DISABLED_OTHER_SELECT	0x8000
#define NI_GPCT_PFI_OTHER_SELECT(x) NI_USUAL_PFI_SELECT(x)

/* Start sources for ni general-purpose counters for use with
   INSN_CONFIG_ARM */
#define NI_GPCT_ARM_IMMEDIATE		0x0
/* Start both the counter and the adjacent paired counter
   simultaneously */
#define NI_GPCT_ARM_PAIRED_IMMEDIATE	0x1
/* NI doesn't document bits for selecting hardware arm triggers.  If
   the NI_GPCT_ARM_UNKNOWN bit is set, we will pass the least significant
   bits (3 bits for 660x or 5 bits for m-series) through to the
   hardware. This will at least allow someone to figure out what the bits
   do later. */
#define NI_GPCT_ARM_UNKNOWN		0x1000

/* Digital filtering options for ni 660x for use with
   INSN_CONFIG_FILTER. */
#define NI_GPCT_FILTER_OFF		0x0
#define NI_GPCT_FILTER_TIMEBASE_3_SYNC	0x1
#define NI_GPCT_FILTER_100x_TIMEBASE_1	0x2
#define NI_GPCT_FILTER_20x_TIMEBASE_1	0x3
#define NI_GPCT_FILTER_10x_TIMEBASE_1	0x4
#define NI_GPCT_FILTER_2x_TIMEBASE_1	0x5
#define NI_GPCT_FILTER_2x_TIMEBASE_3	0x6

/* Master clock sources for ni mio boards and
   INSN_CONFIG_SET_CLOCK_SRC */
#define NI_MIO_INTERNAL_CLOCK		0
#define NI_MIO_RTSI_CLOCK		1
/* Doesn't work for m-series, use NI_MIO_PLL_RTSI_CLOCK() the
   NI_MIO_PLL_* sources are m-series only */
#define NI_MIO_PLL_PXI_STAR_TRIGGER_CLOCK 2
#define NI_MIO_PLL_PXI10_CLOCK		3
#define NI_MIO_PLL_RTSI0_CLOCK		4

#define NI_MIO_PLL_RTSI_CLOCK(x) (NI_MIO_PLL_RTSI0_CLOCK + (x))

/* Signals which can be routed to an NI RTSI pin with
   INSN_CONFIG_SET_ROUTING. The numbers assigned are not arbitrary, they
   correspond to the bits required to program the board. */
#define NI_RTSI_OUTPUT_ADR_START1	0
#define NI_RTSI_OUTPUT_ADR_START2	1
#define NI_RTSI_OUTPUT_SCLKG		2
#define NI_RTSI_OUTPUT_DACUPDN		3
#define NI_RTSI_OUTPUT_DA_START1	4
#define NI_RTSI_OUTPUT_G_SRC0		5
#define NI_RTSI_OUTPUT_G_GATE0		6
#define NI_RTSI_OUTPUT_RGOUT0		7
#define NI_RTSI_OUTPUT_RTSI_BRD_0	8
/* Pre-m-series always have RTSI clock on line 7 */
#define NI_RTSI_OUTPUT_RTSI_OSC		12

#define NI_RTSI_OUTPUT_RTSI_BRD(x) (NI_RTSI_OUTPUT_RTSI_BRD_0 + (x))
