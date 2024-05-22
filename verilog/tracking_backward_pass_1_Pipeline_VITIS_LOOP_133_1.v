// ==============================================================
// RTL generated by Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.1 (64-bit)
// Version: 2022.1
// Copyright (C) Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

`timescale 1 ns / 1 ps 

module tracking_backward_pass_1_Pipeline_VITIS_LOOP_133_1 (
        ap_clk,
        ap_rst,
        ap_start,
        ap_done,
        ap_idle,
        ap_ready,
        p_col_p1_load,
        p_col_p1_load_12,
        p_col_p1_load_13,
        p_col_p1_load_14,
        p_col_p1_load_15,
        p_col_p1_load_16,
        p_col_p1_load_17,
        p_col_p1_load_18,
        p_col_p1_load_19,
        p_col_p1_load_20,
        p_col_p1_load_21,
        p_col_p1_load_22,
        u1_0,
        u1_0_ap_vld,
        BdynT_1_address0,
        BdynT_1_ce0,
        BdynT_1_q0,
        BdynT_1_address1,
        BdynT_1_ce1,
        BdynT_1_q1,
        u1_1,
        u1_1_ap_vld,
        u1_2,
        u1_2_ap_vld,
        u1_3,
        u1_3_ap_vld,
        grp_fu_770_p_din0,
        grp_fu_770_p_din1,
        grp_fu_770_p_opcode,
        grp_fu_770_p_dout0,
        grp_fu_770_p_ce,
        grp_fu_774_p_din0,
        grp_fu_774_p_din1,
        grp_fu_774_p_opcode,
        grp_fu_774_p_dout0,
        grp_fu_774_p_ce
);

parameter    ap_ST_fsm_pp0_stage0 = 6'd1;
parameter    ap_ST_fsm_pp0_stage1 = 6'd2;
parameter    ap_ST_fsm_pp0_stage2 = 6'd4;
parameter    ap_ST_fsm_pp0_stage3 = 6'd8;
parameter    ap_ST_fsm_pp0_stage4 = 6'd16;
parameter    ap_ST_fsm_pp0_stage5 = 6'd32;

input   ap_clk;
input   ap_rst;
input   ap_start;
output   ap_done;
output   ap_idle;
output   ap_ready;
input  [31:0] p_col_p1_load;
input  [31:0] p_col_p1_load_12;
input  [31:0] p_col_p1_load_13;
input  [31:0] p_col_p1_load_14;
input  [31:0] p_col_p1_load_15;
input  [31:0] p_col_p1_load_16;
input  [31:0] p_col_p1_load_17;
input  [31:0] p_col_p1_load_18;
input  [31:0] p_col_p1_load_19;
input  [31:0] p_col_p1_load_20;
input  [31:0] p_col_p1_load_21;
input  [31:0] p_col_p1_load_22;
output  [31:0] u1_0;
output   u1_0_ap_vld;
output  [5:0] BdynT_1_address0;
output   BdynT_1_ce0;
input  [31:0] BdynT_1_q0;
output  [5:0] BdynT_1_address1;
output   BdynT_1_ce1;
input  [31:0] BdynT_1_q1;
output  [31:0] u1_1;
output   u1_1_ap_vld;
output  [31:0] u1_2;
output   u1_2_ap_vld;
output  [31:0] u1_3;
output   u1_3_ap_vld;
output  [31:0] grp_fu_770_p_din0;
output  [31:0] grp_fu_770_p_din1;
output  [0:0] grp_fu_770_p_opcode;
input  [31:0] grp_fu_770_p_dout0;
output   grp_fu_770_p_ce;
output  [31:0] grp_fu_774_p_din0;
output  [31:0] grp_fu_774_p_din1;
output  [0:0] grp_fu_774_p_opcode;
input  [31:0] grp_fu_774_p_dout0;
output   grp_fu_774_p_ce;

reg ap_idle;
reg u1_0_ap_vld;
reg[5:0] BdynT_1_address0;
reg BdynT_1_ce0;
reg[5:0] BdynT_1_address1;
reg BdynT_1_ce1;
reg u1_1_ap_vld;
reg u1_2_ap_vld;
reg u1_3_ap_vld;

(* fsm_encoding = "none" *) reg   [5:0] ap_CS_fsm;
wire    ap_CS_fsm_pp0_stage0;
reg    ap_enable_reg_pp0_iter0;
reg    ap_enable_reg_pp0_iter1;
reg    ap_enable_reg_pp0_iter2;
reg    ap_idle_pp0;
wire    ap_CS_fsm_pp0_stage5;
wire    ap_block_state6_pp0_stage5_iter0;
wire    ap_block_state12_pp0_stage5_iter1;
wire    ap_block_pp0_stage5_subdone;
reg   [0:0] icmp_ln133_reg_580;
reg    ap_condition_exit_pp0_iter0_stage5;
wire    ap_loop_exit_ready;
reg    ap_ready_int;
wire   [31:0] grp_fu_291_p2;
reg   [31:0] reg_301;
wire    ap_CS_fsm_pp0_stage1;
wire    ap_block_state2_pp0_stage1_iter0;
wire    ap_block_state8_pp0_stage1_iter1;
wire    ap_block_state14_pp0_stage1_iter2;
wire    ap_block_pp0_stage1_11001;
wire    ap_CS_fsm_pp0_stage3;
wire    ap_block_state4_pp0_stage3_iter0;
wire    ap_block_state10_pp0_stage3_iter1;
wire    ap_block_pp0_stage3_11001;
wire    ap_CS_fsm_pp0_stage4;
wire    ap_block_state5_pp0_stage4_iter0;
wire    ap_block_state11_pp0_stage4_iter1;
wire    ap_block_pp0_stage4_11001;
wire    ap_block_pp0_stage5_11001;
wire    ap_block_state1_pp0_stage0_iter0;
wire    ap_block_state7_pp0_stage0_iter1;
wire    ap_block_state13_pp0_stage0_iter2;
wire    ap_block_pp0_stage0_11001;
wire    ap_CS_fsm_pp0_stage2;
wire    ap_block_state3_pp0_stage2_iter0;
wire    ap_block_state9_pp0_stage2_iter1;
wire    ap_block_state15_pp0_stage2_iter2;
wire    ap_block_pp0_stage2_11001;
wire   [0:0] icmp_ln133_fu_326_p2;
reg   [0:0] icmp_ln133_reg_580_pp0_iter1_reg;
wire   [1:0] trunc_ln134_fu_338_p1;
reg   [1:0] trunc_ln134_reg_584;
reg   [1:0] trunc_ln134_reg_584_pp0_iter1_reg;
reg   [1:0] trunc_ln134_reg_584_pp0_iter2_reg;
wire   [5:0] empty_121_fu_362_p2;
reg   [5:0] empty_121_reg_588;
wire   [31:0] grp_fu_296_p2;
reg   [31:0] mul9_i_1_reg_612;
reg   [31:0] mul9_i_2_reg_627;
reg   [31:0] mul9_i_3_reg_632;
reg   [31:0] mul9_i_5_reg_647;
reg   [31:0] mul9_i_6_reg_662;
reg   [31:0] mul9_i_7_reg_667;
reg   [31:0] mul9_i_8_reg_682;
reg   [31:0] mul9_i_9_reg_687;
reg   [31:0] mul9_i_10_reg_702;
reg   [31:0] mul9_i_s_reg_707;
reg   [31:0] mul9_i_s_reg_707_pp0_iter2_reg;
reg    ap_enable_reg_pp0_iter0_reg;
wire    ap_block_pp0_stage2_subdone;
wire   [63:0] zext_ln136_fu_368_p1;
wire    ap_block_pp0_stage0;
wire   [63:0] zext_ln136_41_fu_379_p1;
wire   [63:0] zext_ln136_42_fu_394_p1;
wire    ap_block_pp0_stage1;
wire   [63:0] zext_ln136_43_fu_404_p1;
wire   [63:0] zext_ln136_44_fu_414_p1;
wire    ap_block_pp0_stage2;
wire   [63:0] zext_ln136_45_fu_424_p1;
wire   [63:0] zext_ln136_46_fu_434_p1;
wire    ap_block_pp0_stage3;
wire   [63:0] zext_ln136_47_fu_444_p1;
wire   [63:0] zext_ln136_48_fu_454_p1;
wire    ap_block_pp0_stage4;
wire   [63:0] zext_ln136_49_fu_464_p1;
wire   [63:0] zext_ln136_50_fu_474_p1;
wire    ap_block_pp0_stage5;
wire   [63:0] zext_ln136_51_fu_484_p1;
reg   [2:0] i_fu_98;
wire   [2:0] add_ln133_fu_332_p2;
wire    ap_loop_init;
reg   [2:0] ap_sig_allocacmp_i_59;
reg   [31:0] grp_fu_279_p0;
reg   [31:0] grp_fu_279_p1;
reg   [31:0] grp_fu_285_p0;
reg   [31:0] grp_fu_285_p1;
reg   [31:0] grp_fu_291_p1;
reg   [31:0] grp_fu_296_p1;
wire   [3:0] p_shl4_fu_350_p3;
wire   [5:0] p_shl3_fu_342_p3;
wire   [5:0] p_shl4_cast_fu_358_p1;
wire   [5:0] or_ln136_fu_373_p2;
wire   [5:0] or_ln136_11_fu_389_p2;
wire   [5:0] or_ln136_12_fu_399_p2;
wire   [5:0] add_ln136_fu_409_p2;
wire   [5:0] add_ln136_5_fu_419_p2;
wire   [5:0] add_ln136_6_fu_429_p2;
wire   [5:0] add_ln136_7_fu_439_p2;
wire   [5:0] add_ln136_8_fu_449_p2;
wire   [5:0] add_ln136_9_fu_459_p2;
wire   [5:0] add_ln136_10_fu_469_p2;
wire   [5:0] add_ln136_11_fu_479_p2;
wire    ap_block_pp0_stage2_00001;
wire    ap_block_pp0_stage3_00001;
wire    ap_block_pp0_stage4_00001;
wire    ap_block_pp0_stage5_00001;
wire    ap_block_pp0_stage0_00001;
wire    ap_block_pp0_stage1_00001;
reg    ap_done_reg;
wire    ap_continue_int;
reg    ap_done_int;
reg    ap_loop_exit_ready_pp0_iter1_reg;
reg    ap_condition_exit_pp0_iter1_stage2;
reg    ap_idle_pp0_0to0;
reg   [5:0] ap_NS_fsm;
wire    ap_block_pp0_stage0_subdone;
reg    ap_idle_pp0_1to2;
wire    ap_block_pp0_stage1_subdone;
wire    ap_block_pp0_stage3_subdone;
wire    ap_block_pp0_stage4_subdone;
wire    ap_enable_pp0;
wire    ap_start_int;
wire    ap_ce_reg;

// power-on initialization
initial begin
#0 ap_CS_fsm = 6'd1;
#0 ap_enable_reg_pp0_iter1 = 1'b0;
#0 ap_enable_reg_pp0_iter2 = 1'b0;
#0 ap_enable_reg_pp0_iter0_reg = 1'b0;
#0 ap_done_reg = 1'b0;
end

tracking_fmul_32ns_32ns_32_1_max_dsp_1 #(
    .ID( 1 ),
    .NUM_STAGE( 1 ),
    .din0_WIDTH( 32 ),
    .din1_WIDTH( 32 ),
    .dout_WIDTH( 32 ))
fmul_32ns_32ns_32_1_max_dsp_1_U234(
    .din0(BdynT_1_q1),
    .din1(grp_fu_291_p1),
    .dout(grp_fu_291_p2)
);

tracking_fmul_32ns_32ns_32_1_max_dsp_1 #(
    .ID( 1 ),
    .NUM_STAGE( 1 ),
    .din0_WIDTH( 32 ),
    .din1_WIDTH( 32 ),
    .dout_WIDTH( 32 ))
fmul_32ns_32ns_32_1_max_dsp_1_U235(
    .din0(BdynT_1_q0),
    .din1(grp_fu_296_p1),
    .dout(grp_fu_296_p2)
);

tracking_flow_control_loop_pipe_sequential_init flow_control_loop_pipe_sequential_init_U(
    .ap_clk(ap_clk),
    .ap_rst(ap_rst),
    .ap_start(ap_start),
    .ap_ready(ap_ready),
    .ap_done(ap_done),
    .ap_start_int(ap_start_int),
    .ap_loop_init(ap_loop_init),
    .ap_ready_int(ap_ready_int),
    .ap_loop_exit_ready(ap_condition_exit_pp0_iter0_stage5),
    .ap_loop_exit_done(ap_done_int),
    .ap_continue_int(ap_continue_int),
    .ap_done_int(ap_done_int)
);

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        ap_CS_fsm <= ap_ST_fsm_pp0_stage0;
    end else begin
        ap_CS_fsm <= ap_NS_fsm;
    end
end

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        ap_done_reg <= 1'b0;
    end else begin
        if ((ap_continue_int == 1'b1)) begin
            ap_done_reg <= 1'b0;
        end else if (((ap_loop_exit_ready_pp0_iter1_reg == 1'b1) & (1'b0 == ap_block_pp0_stage2_subdone) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
            ap_done_reg <= 1'b1;
        end
    end
end

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        ap_enable_reg_pp0_iter0_reg <= 1'b0;
    end else begin
        if ((1'b1 == ap_CS_fsm_pp0_stage0)) begin
            ap_enable_reg_pp0_iter0_reg <= ap_start_int;
        end
    end
end

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        ap_enable_reg_pp0_iter1 <= 1'b0;
    end else begin
        if ((1'b1 == ap_condition_exit_pp0_iter0_stage5)) begin
            ap_enable_reg_pp0_iter1 <= 1'b0;
        end else if (((1'b0 == ap_block_pp0_stage5_subdone) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
            ap_enable_reg_pp0_iter1 <= ap_enable_reg_pp0_iter0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        ap_enable_reg_pp0_iter2 <= 1'b0;
    end else begin
        if (((1'b0 == ap_block_pp0_stage2_subdone) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
            ap_enable_reg_pp0_iter2 <= 1'b0;
        end else if (((1'b0 == ap_block_pp0_stage5_subdone) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
            ap_enable_reg_pp0_iter2 <= ap_enable_reg_pp0_iter1;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((ap_idle_pp0_0to0 == 1'b1) & (1'b1 == ap_condition_exit_pp0_iter1_stage2))) begin
        ap_loop_exit_ready_pp0_iter1_reg <= 1'b0;
    end else if (((1'b0 == ap_block_pp0_stage5_11001) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        ap_loop_exit_ready_pp0_iter1_reg <= ap_loop_exit_ready;
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((ap_enable_reg_pp0_iter0 == 1'b1) & (icmp_ln133_fu_326_p2 == 1'd0))) begin
            i_fu_98 <= add_ln133_fu_332_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            i_fu_98 <= 3'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (icmp_ln133_fu_326_p2 == 1'd0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        empty_121_reg_588[5 : 2] <= empty_121_fu_362_p2[5 : 2];
        trunc_ln134_reg_584 <= trunc_ln134_fu_338_p1;
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        icmp_ln133_reg_580 <= icmp_ln133_fu_326_p2;
        icmp_ln133_reg_580_pp0_iter1_reg <= icmp_ln133_reg_580;
        mul9_i_s_reg_707_pp0_iter2_reg <= mul9_i_s_reg_707;
        trunc_ln134_reg_584_pp0_iter1_reg <= trunc_ln134_reg_584;
        trunc_ln134_reg_584_pp0_iter2_reg <= trunc_ln134_reg_584_pp0_iter1_reg;
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        mul9_i_10_reg_702 <= grp_fu_291_p2;
        mul9_i_s_reg_707 <= grp_fu_296_p2;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln133_reg_580 == 1'd0) & (1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        mul9_i_1_reg_612 <= grp_fu_296_p2;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln133_reg_580 == 1'd0) & (1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        mul9_i_2_reg_627 <= grp_fu_291_p2;
        mul9_i_3_reg_632 <= grp_fu_296_p2;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln133_reg_580 == 1'd0) & (1'b0 == ap_block_pp0_stage3_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
        mul9_i_5_reg_647 <= grp_fu_296_p2;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln133_reg_580 == 1'd0) & (1'b0 == ap_block_pp0_stage4_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
        mul9_i_6_reg_662 <= grp_fu_291_p2;
        mul9_i_7_reg_667 <= grp_fu_296_p2;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln133_reg_580 == 1'd0) & (1'b0 == ap_block_pp0_stage5_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        mul9_i_8_reg_682 <= grp_fu_291_p2;
        mul9_i_9_reg_687 <= grp_fu_296_p2;
    end
end

always @ (posedge ap_clk) begin
    if ((((icmp_ln133_reg_580 == 1'd0) & (1'b0 == ap_block_pp0_stage3_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3)) | ((icmp_ln133_reg_580 == 1'd0) & (1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)))) begin
        reg_301 <= grp_fu_291_p2;
    end
end

always @ (*) begin
    if ((ap_enable_reg_pp0_iter0 == 1'b1)) begin
        if (((1'b0 == ap_block_pp0_stage5) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
            BdynT_1_address0 = zext_ln136_51_fu_484_p1;
        end else if (((1'b0 == ap_block_pp0_stage4) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
            BdynT_1_address0 = zext_ln136_49_fu_464_p1;
        end else if (((1'b0 == ap_block_pp0_stage3) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
            BdynT_1_address0 = zext_ln136_47_fu_444_p1;
        end else if (((1'b0 == ap_block_pp0_stage2) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
            BdynT_1_address0 = zext_ln136_45_fu_424_p1;
        end else if (((1'b0 == ap_block_pp0_stage1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
            BdynT_1_address0 = zext_ln136_43_fu_404_p1;
        end else if (((1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            BdynT_1_address0 = zext_ln136_41_fu_379_p1;
        end else begin
            BdynT_1_address0 = 'bx;
        end
    end else begin
        BdynT_1_address0 = 'bx;
    end
end

always @ (*) begin
    if ((ap_enable_reg_pp0_iter0 == 1'b1)) begin
        if (((1'b0 == ap_block_pp0_stage5) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
            BdynT_1_address1 = zext_ln136_50_fu_474_p1;
        end else if (((1'b0 == ap_block_pp0_stage4) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
            BdynT_1_address1 = zext_ln136_48_fu_454_p1;
        end else if (((1'b0 == ap_block_pp0_stage3) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
            BdynT_1_address1 = zext_ln136_46_fu_434_p1;
        end else if (((1'b0 == ap_block_pp0_stage2) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
            BdynT_1_address1 = zext_ln136_44_fu_414_p1;
        end else if (((1'b0 == ap_block_pp0_stage1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
            BdynT_1_address1 = zext_ln136_42_fu_394_p1;
        end else if (((1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            BdynT_1_address1 = zext_ln136_fu_368_p1;
        end else begin
            BdynT_1_address1 = 'bx;
        end
    end else begin
        BdynT_1_address1 = 'bx;
    end
end

always @ (*) begin
    if ((((1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2)) | ((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)) | ((1'b0 == ap_block_pp0_stage5_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5)) | ((1'b0 == ap_block_pp0_stage4_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4)) | ((1'b0 == ap_block_pp0_stage3_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3)) | ((1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)))) begin
        BdynT_1_ce0 = 1'b1;
    end else begin
        BdynT_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if ((((1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2)) | ((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)) | ((1'b0 == ap_block_pp0_stage5_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5)) | ((1'b0 == ap_block_pp0_stage4_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4)) | ((1'b0 == ap_block_pp0_stage3_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3)) | ((1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)))) begin
        BdynT_1_ce1 = 1'b1;
    end else begin
        BdynT_1_ce1 = 1'b0;
    end
end

always @ (*) begin
    if (((icmp_ln133_reg_580 == 1'd1) & (1'b0 == ap_block_pp0_stage5_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        ap_condition_exit_pp0_iter0_stage5 = 1'b1;
    end else begin
        ap_condition_exit_pp0_iter0_stage5 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage2_subdone) & (ap_enable_reg_pp0_iter1 == 1'b1) & (icmp_ln133_reg_580_pp0_iter1_reg == 1'd1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        ap_condition_exit_pp0_iter1_stage2 = 1'b1;
    end else begin
        ap_condition_exit_pp0_iter1_stage2 = 1'b0;
    end
end

always @ (*) begin
    if (((ap_loop_exit_ready_pp0_iter1_reg == 1'b1) & (1'b0 == ap_block_pp0_stage2_subdone) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        ap_done_int = 1'b1;
    end else begin
        ap_done_int = ap_done_reg;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_pp0_stage0)) begin
        ap_enable_reg_pp0_iter0 = ap_start_int;
    end else begin
        ap_enable_reg_pp0_iter0 = ap_enable_reg_pp0_iter0_reg;
    end
end

always @ (*) begin
    if (((ap_start_int == 1'b0) & (ap_idle_pp0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_idle = 1'b1;
    end else begin
        ap_idle = 1'b0;
    end
end

always @ (*) begin
    if (((ap_enable_reg_pp0_iter2 == 1'b0) & (ap_enable_reg_pp0_iter1 == 1'b0) & (ap_enable_reg_pp0_iter0 == 1'b0))) begin
        ap_idle_pp0 = 1'b1;
    end else begin
        ap_idle_pp0 = 1'b0;
    end
end

always @ (*) begin
    if ((ap_enable_reg_pp0_iter0 == 1'b0)) begin
        ap_idle_pp0_0to0 = 1'b1;
    end else begin
        ap_idle_pp0_0to0 = 1'b0;
    end
end

always @ (*) begin
    if (((ap_enable_reg_pp0_iter2 == 1'b0) & (ap_enable_reg_pp0_iter1 == 1'b0))) begin
        ap_idle_pp0_1to2 = 1'b1;
    end else begin
        ap_idle_pp0_1to2 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage5_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        ap_ready_int = 1'b1;
    end else begin
        ap_ready_int = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0) & (ap_loop_init == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_i_59 = 3'd0;
    end else begin
        ap_sig_allocacmp_i_59 = i_fu_98;
    end
end

always @ (*) begin
    if ((((1'b0 == ap_block_pp0_stage5) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5)) | ((1'b0 == ap_block_pp0_stage4) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4)) | ((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3)) | ((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)))) begin
        grp_fu_279_p0 = grp_fu_770_p_dout0;
    end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        grp_fu_279_p0 = reg_301;
    end else begin
        grp_fu_279_p0 = 'bx;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        grp_fu_279_p1 = mul9_i_5_reg_647;
    end else if (((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        grp_fu_279_p1 = reg_301;
    end else if (((1'b0 == ap_block_pp0_stage5) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        grp_fu_279_p1 = mul9_i_3_reg_632;
    end else if (((1'b0 == ap_block_pp0_stage4) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
        grp_fu_279_p1 = mul9_i_2_reg_627;
    end else if (((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
        grp_fu_279_p1 = mul9_i_1_reg_612;
    end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        grp_fu_279_p1 = 32'd0;
    end else begin
        grp_fu_279_p1 = 'bx;
    end
end

always @ (*) begin
    if ((((1'b0 == ap_block_pp0_stage5) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5)) | ((1'b0 == ap_block_pp0_stage4) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4)) | ((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3)) | ((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)))) begin
        grp_fu_285_p0 = grp_fu_774_p_dout0;
    end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        grp_fu_285_p0 = grp_fu_770_p_dout0;
    end else begin
        grp_fu_285_p0 = 'bx;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        grp_fu_285_p1 = mul9_i_s_reg_707_pp0_iter2_reg;
    end else if (((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        grp_fu_285_p1 = mul9_i_10_reg_702;
    end else if (((1'b0 == ap_block_pp0_stage5) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        grp_fu_285_p1 = mul9_i_9_reg_687;
    end else if (((1'b0 == ap_block_pp0_stage4) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
        grp_fu_285_p1 = mul9_i_8_reg_682;
    end else if (((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
        grp_fu_285_p1 = mul9_i_7_reg_667;
    end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        grp_fu_285_p1 = mul9_i_6_reg_662;
    end else begin
        grp_fu_285_p1 = 'bx;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        grp_fu_291_p1 = p_col_p1_load_21;
    end else if (((1'b0 == ap_block_pp0_stage5) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        grp_fu_291_p1 = p_col_p1_load_19;
    end else if (((1'b0 == ap_block_pp0_stage4) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
        grp_fu_291_p1 = p_col_p1_load_17;
    end else if (((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
        grp_fu_291_p1 = p_col_p1_load_15;
    end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        grp_fu_291_p1 = p_col_p1_load_13;
    end else if (((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        grp_fu_291_p1 = p_col_p1_load;
    end else begin
        grp_fu_291_p1 = 'bx;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        grp_fu_296_p1 = p_col_p1_load_22;
    end else if (((1'b0 == ap_block_pp0_stage5) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        grp_fu_296_p1 = p_col_p1_load_20;
    end else if (((1'b0 == ap_block_pp0_stage4) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
        grp_fu_296_p1 = p_col_p1_load_18;
    end else if (((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
        grp_fu_296_p1 = p_col_p1_load_16;
    end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        grp_fu_296_p1 = p_col_p1_load_14;
    end else if (((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        grp_fu_296_p1 = p_col_p1_load_12;
    end else begin
        grp_fu_296_p1 = 'bx;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter2 == 1'b1) & (trunc_ln134_reg_584_pp0_iter2_reg == 2'd0) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        u1_0_ap_vld = 1'b1;
    end else begin
        u1_0_ap_vld = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter2 == 1'b1) & (trunc_ln134_reg_584_pp0_iter2_reg == 2'd1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        u1_1_ap_vld = 1'b1;
    end else begin
        u1_1_ap_vld = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter2 == 1'b1) & (trunc_ln134_reg_584_pp0_iter2_reg == 2'd2) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        u1_2_ap_vld = 1'b1;
    end else begin
        u1_2_ap_vld = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter2 == 1'b1) & (trunc_ln134_reg_584_pp0_iter2_reg == 2'd3) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        u1_3_ap_vld = 1'b1;
    end else begin
        u1_3_ap_vld = 1'b0;
    end
end

always @ (*) begin
    case (ap_CS_fsm)
        ap_ST_fsm_pp0_stage0 : begin
            if ((~((ap_start_int == 1'b0) & (ap_idle_pp0_1to2 == 1'b1)) & (1'b0 == ap_block_pp0_stage0_subdone))) begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage1;
            end else begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage0;
            end
        end
        ap_ST_fsm_pp0_stage1 : begin
            if ((1'b0 == ap_block_pp0_stage1_subdone)) begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage2;
            end else begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage1;
            end
        end
        ap_ST_fsm_pp0_stage2 : begin
            if (((ap_idle_pp0_0to0 == 1'b1) & (1'b1 == ap_condition_exit_pp0_iter1_stage2))) begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage0;
            end else if ((1'b0 == ap_block_pp0_stage2_subdone)) begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage3;
            end else begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage2;
            end
        end
        ap_ST_fsm_pp0_stage3 : begin
            if ((1'b0 == ap_block_pp0_stage3_subdone)) begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage4;
            end else begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage3;
            end
        end
        ap_ST_fsm_pp0_stage4 : begin
            if ((1'b0 == ap_block_pp0_stage4_subdone)) begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage5;
            end else begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage4;
            end
        end
        ap_ST_fsm_pp0_stage5 : begin
            if ((1'b0 == ap_block_pp0_stage5_subdone)) begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage0;
            end else begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage5;
            end
        end
        default : begin
            ap_NS_fsm = 'bx;
        end
    endcase
end

assign add_ln133_fu_332_p2 = (ap_sig_allocacmp_i_59 + 3'd1);

assign add_ln136_10_fu_469_p2 = (empty_121_reg_588 + 6'd10);

assign add_ln136_11_fu_479_p2 = (empty_121_reg_588 + 6'd11);

assign add_ln136_5_fu_419_p2 = (empty_121_reg_588 + 6'd5);

assign add_ln136_6_fu_429_p2 = (empty_121_reg_588 + 6'd6);

assign add_ln136_7_fu_439_p2 = (empty_121_reg_588 + 6'd7);

assign add_ln136_8_fu_449_p2 = (empty_121_reg_588 + 6'd8);

assign add_ln136_9_fu_459_p2 = (empty_121_reg_588 + 6'd9);

assign add_ln136_fu_409_p2 = (empty_121_reg_588 + 6'd4);

assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

assign ap_CS_fsm_pp0_stage1 = ap_CS_fsm[32'd1];

assign ap_CS_fsm_pp0_stage2 = ap_CS_fsm[32'd2];

assign ap_CS_fsm_pp0_stage3 = ap_CS_fsm[32'd3];

assign ap_CS_fsm_pp0_stage4 = ap_CS_fsm[32'd4];

assign ap_CS_fsm_pp0_stage5 = ap_CS_fsm[32'd5];

assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_00001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage1 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage1_00001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage1_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage1_subdone = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage2 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage2_00001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage2_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage2_subdone = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage3 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage3_00001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage3_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage3_subdone = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage4 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage4_00001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage4_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage4_subdone = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage5 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage5_00001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage5_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage5_subdone = ~(1'b1 == 1'b1);

assign ap_block_state10_pp0_stage3_iter1 = ~(1'b1 == 1'b1);

assign ap_block_state11_pp0_stage4_iter1 = ~(1'b1 == 1'b1);

assign ap_block_state12_pp0_stage5_iter1 = ~(1'b1 == 1'b1);

assign ap_block_state13_pp0_stage0_iter2 = ~(1'b1 == 1'b1);

assign ap_block_state14_pp0_stage1_iter2 = ~(1'b1 == 1'b1);

assign ap_block_state15_pp0_stage2_iter2 = ~(1'b1 == 1'b1);

assign ap_block_state1_pp0_stage0_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state2_pp0_stage1_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state3_pp0_stage2_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state4_pp0_stage3_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state5_pp0_stage4_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state6_pp0_stage5_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state7_pp0_stage0_iter1 = ~(1'b1 == 1'b1);

assign ap_block_state8_pp0_stage1_iter1 = ~(1'b1 == 1'b1);

assign ap_block_state9_pp0_stage2_iter1 = ~(1'b1 == 1'b1);

assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage5;

assign empty_121_fu_362_p2 = (p_shl3_fu_342_p3 - p_shl4_cast_fu_358_p1);

assign grp_fu_770_p_ce = 1'b1;

assign grp_fu_770_p_din0 = grp_fu_279_p0;

assign grp_fu_770_p_din1 = grp_fu_279_p1;

assign grp_fu_770_p_opcode = 2'd0;

assign grp_fu_774_p_ce = 1'b1;

assign grp_fu_774_p_din0 = grp_fu_285_p0;

assign grp_fu_774_p_din1 = grp_fu_285_p1;

assign grp_fu_774_p_opcode = 2'd0;

assign icmp_ln133_fu_326_p2 = ((ap_sig_allocacmp_i_59 == 3'd4) ? 1'b1 : 1'b0);

assign or_ln136_11_fu_389_p2 = (empty_121_reg_588 | 6'd2);

assign or_ln136_12_fu_399_p2 = (empty_121_reg_588 | 6'd3);

assign or_ln136_fu_373_p2 = (empty_121_fu_362_p2 | 6'd1);

assign p_shl3_fu_342_p3 = {{trunc_ln134_fu_338_p1}, {4'd0}};

assign p_shl4_cast_fu_358_p1 = p_shl4_fu_350_p3;

assign p_shl4_fu_350_p3 = {{trunc_ln134_fu_338_p1}, {2'd0}};

assign trunc_ln134_fu_338_p1 = ap_sig_allocacmp_i_59[1:0];

assign u1_0 = grp_fu_774_p_dout0;

assign u1_1 = grp_fu_774_p_dout0;

assign u1_2 = grp_fu_774_p_dout0;

assign u1_3 = grp_fu_774_p_dout0;

assign zext_ln136_41_fu_379_p1 = or_ln136_fu_373_p2;

assign zext_ln136_42_fu_394_p1 = or_ln136_11_fu_389_p2;

assign zext_ln136_43_fu_404_p1 = or_ln136_12_fu_399_p2;

assign zext_ln136_44_fu_414_p1 = add_ln136_fu_409_p2;

assign zext_ln136_45_fu_424_p1 = add_ln136_5_fu_419_p2;

assign zext_ln136_46_fu_434_p1 = add_ln136_6_fu_429_p2;

assign zext_ln136_47_fu_444_p1 = add_ln136_7_fu_439_p2;

assign zext_ln136_48_fu_454_p1 = add_ln136_8_fu_449_p2;

assign zext_ln136_49_fu_464_p1 = add_ln136_9_fu_459_p2;

assign zext_ln136_50_fu_474_p1 = add_ln136_10_fu_469_p2;

assign zext_ln136_51_fu_484_p1 = add_ln136_11_fu_479_p2;

assign zext_ln136_fu_368_p1 = empty_121_fu_362_p2;

always @ (posedge ap_clk) begin
    empty_121_reg_588[1:0] <= 2'b00;
end

endmodule //tracking_backward_pass_1_Pipeline_VITIS_LOOP_133_1