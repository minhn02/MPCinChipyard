// ==============================================================
// RTL generated by Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.1 (64-bit)
// Version: 2022.1
// Copyright (C) Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

`timescale 1 ns / 1 ps 

module tracking_forward_pass_2_Pipeline_VITIS_LOOP_133_1 (
        ap_clk,
        ap_rst,
        ap_start,
        ap_done,
        ap_idle,
        ap_ready,
        x_col_load,
        x_col_load_1,
        x_col_load_2,
        x_col_load_3,
        x_col_load_4,
        x_col_load_5,
        x_col_load_6,
        x_col_load_7,
        x_col_load_8,
        x_col_load_9,
        x_col_load_10,
        x_col_load_11,
        Adyn_1_address0,
        Adyn_1_ce0,
        Adyn_1_q0,
        Adyn_1_address1,
        Adyn_1_ce1,
        Adyn_1_q1,
        tiny_x1_address0,
        tiny_x1_ce0,
        tiny_x1_we0,
        tiny_x1_d0
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
input  [31:0] x_col_load;
input  [31:0] x_col_load_1;
input  [31:0] x_col_load_2;
input  [31:0] x_col_load_3;
input  [31:0] x_col_load_4;
input  [31:0] x_col_load_5;
input  [31:0] x_col_load_6;
input  [31:0] x_col_load_7;
input  [31:0] x_col_load_8;
input  [31:0] x_col_load_9;
input  [31:0] x_col_load_10;
input  [31:0] x_col_load_11;
output  [7:0] Adyn_1_address0;
output   Adyn_1_ce0;
input  [31:0] Adyn_1_q0;
output  [7:0] Adyn_1_address1;
output   Adyn_1_ce1;
input  [31:0] Adyn_1_q1;
output  [3:0] tiny_x1_address0;
output   tiny_x1_ce0;
output   tiny_x1_we0;
output  [31:0] tiny_x1_d0;

reg ap_idle;
reg[7:0] Adyn_1_address0;
reg Adyn_1_ce0;
reg[7:0] Adyn_1_address1;
reg Adyn_1_ce1;
reg tiny_x1_ce0;
reg tiny_x1_we0;

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
reg   [0:0] icmp_ln133_reg_563;
reg    ap_condition_exit_pp0_iter0_stage5;
wire    ap_loop_exit_ready;
reg    ap_ready_int;
wire   [31:0] grp_fu_293_p2;
reg   [31:0] reg_303;
wire    ap_CS_fsm_pp0_stage1;
wire    ap_block_state2_pp0_stage1_iter0;
wire    ap_block_state8_pp0_stage1_iter1;
wire    ap_block_state14_pp0_stage1_iter2;
wire    ap_block_pp0_stage1_11001;
wire    ap_CS_fsm_pp0_stage3;
wire    ap_block_state4_pp0_stage3_iter0;
wire    ap_block_state10_pp0_stage3_iter1;
wire    ap_block_pp0_stage3_11001;
wire   [31:0] grp_fu_280_p2;
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
wire   [31:0] grp_fu_286_p2;
reg   [3:0] i_50_reg_558;
reg   [3:0] i_50_reg_558_pp0_iter1_reg;
reg   [3:0] i_50_reg_558_pp0_iter2_reg;
wire   [0:0] icmp_ln133_fu_328_p2;
reg   [0:0] icmp_ln133_reg_563_pp0_iter1_reg;
wire   [7:0] empty_118_fu_360_p2;
reg   [7:0] empty_118_reg_567;
wire   [31:0] grp_fu_298_p2;
reg   [31:0] mul9_i_1_reg_591;
reg   [31:0] mul9_i_2_reg_606;
reg   [31:0] mul9_i_3_reg_611;
reg   [31:0] mul9_i_5_reg_626;
reg   [31:0] mul9_i_6_reg_641;
reg   [31:0] mul9_i_7_reg_646;
reg   [31:0] mul9_i_8_reg_661;
reg   [31:0] mul9_i_9_reg_666;
reg   [31:0] mul9_i_10_reg_681;
reg   [31:0] mul9_i_s_reg_686;
reg   [31:0] mul9_i_s_reg_686_pp0_iter2_reg;
reg    ap_enable_reg_pp0_iter0_reg;
wire    ap_block_pp0_stage2_subdone;
wire   [63:0] zext_ln136_fu_366_p1;
wire    ap_block_pp0_stage0;
wire   [63:0] zext_ln136_13_fu_377_p1;
wire   [63:0] zext_ln136_14_fu_392_p1;
wire    ap_block_pp0_stage1;
wire   [63:0] zext_ln136_15_fu_402_p1;
wire   [63:0] zext_ln136_16_fu_412_p1;
wire    ap_block_pp0_stage2;
wire   [63:0] zext_ln136_17_fu_422_p1;
wire   [63:0] zext_ln136_18_fu_432_p1;
wire    ap_block_pp0_stage3;
wire   [63:0] zext_ln136_19_fu_442_p1;
wire   [63:0] zext_ln136_20_fu_452_p1;
wire    ap_block_pp0_stage4;
wire   [63:0] zext_ln136_21_fu_462_p1;
wire   [63:0] zext_ln136_22_fu_472_p1;
wire    ap_block_pp0_stage5;
wire   [63:0] zext_ln136_23_fu_482_p1;
wire   [63:0] i_57_cast15_fu_487_p1;
reg   [3:0] i_fu_86;
wire   [3:0] add_ln133_fu_334_p2;
wire    ap_loop_init;
reg   [3:0] ap_sig_allocacmp_i_50;
reg   [31:0] grp_fu_280_p0;
reg   [31:0] grp_fu_280_p1;
reg   [31:0] grp_fu_286_p0;
reg   [31:0] grp_fu_286_p1;
reg   [31:0] grp_fu_293_p1;
reg   [31:0] grp_fu_298_p1;
wire   [5:0] p_shl3_fu_348_p3;
wire   [7:0] p_shl2_fu_340_p3;
wire   [7:0] p_shl3_cast_fu_356_p1;
wire   [7:0] or_ln136_fu_371_p2;
wire   [7:0] or_ln136_3_fu_387_p2;
wire   [7:0] or_ln136_4_fu_397_p2;
wire   [7:0] add_ln136_4_fu_407_p2;
wire   [7:0] add_ln136_5_fu_417_p2;
wire   [7:0] add_ln136_6_fu_427_p2;
wire   [7:0] add_ln136_7_fu_437_p2;
wire   [7:0] add_ln136_fu_447_p2;
wire   [7:0] add_ln136_8_fu_457_p2;
wire   [7:0] add_ln136_9_fu_467_p2;
wire   [7:0] add_ln136_10_fu_477_p2;
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

tracking_fadd_32ns_32ns_32_2_full_dsp_1 #(
    .ID( 1 ),
    .NUM_STAGE( 2 ),
    .din0_WIDTH( 32 ),
    .din1_WIDTH( 32 ),
    .dout_WIDTH( 32 ))
fadd_32ns_32ns_32_2_full_dsp_1_U34(
    .clk(ap_clk),
    .reset(ap_rst),
    .din0(grp_fu_280_p0),
    .din1(grp_fu_280_p1),
    .ce(1'b1),
    .dout(grp_fu_280_p2)
);

tracking_fadd_32ns_32ns_32_2_full_dsp_1 #(
    .ID( 1 ),
    .NUM_STAGE( 2 ),
    .din0_WIDTH( 32 ),
    .din1_WIDTH( 32 ),
    .dout_WIDTH( 32 ))
fadd_32ns_32ns_32_2_full_dsp_1_U35(
    .clk(ap_clk),
    .reset(ap_rst),
    .din0(grp_fu_286_p0),
    .din1(grp_fu_286_p1),
    .ce(1'b1),
    .dout(grp_fu_286_p2)
);

tracking_fmul_32ns_32ns_32_1_max_dsp_1 #(
    .ID( 1 ),
    .NUM_STAGE( 1 ),
    .din0_WIDTH( 32 ),
    .din1_WIDTH( 32 ),
    .dout_WIDTH( 32 ))
fmul_32ns_32ns_32_1_max_dsp_1_U36(
    .din0(Adyn_1_q1),
    .din1(grp_fu_293_p1),
    .dout(grp_fu_293_p2)
);

tracking_fmul_32ns_32ns_32_1_max_dsp_1 #(
    .ID( 1 ),
    .NUM_STAGE( 1 ),
    .din0_WIDTH( 32 ),
    .din1_WIDTH( 32 ),
    .dout_WIDTH( 32 ))
fmul_32ns_32ns_32_1_max_dsp_1_U37(
    .din0(Adyn_1_q0),
    .din1(grp_fu_298_p1),
    .dout(grp_fu_298_p2)
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
        if (((ap_enable_reg_pp0_iter0 == 1'b1) & (icmp_ln133_fu_328_p2 == 1'd0))) begin
            i_fu_86 <= add_ln133_fu_334_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            i_fu_86 <= 4'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (icmp_ln133_fu_328_p2 == 1'd0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        empty_118_reg_567[7 : 2] <= empty_118_fu_360_p2[7 : 2];
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        i_50_reg_558 <= ap_sig_allocacmp_i_50;
        i_50_reg_558_pp0_iter1_reg <= i_50_reg_558;
        i_50_reg_558_pp0_iter2_reg <= i_50_reg_558_pp0_iter1_reg;
        icmp_ln133_reg_563 <= icmp_ln133_fu_328_p2;
        icmp_ln133_reg_563_pp0_iter1_reg <= icmp_ln133_reg_563;
        mul9_i_s_reg_686_pp0_iter2_reg <= mul9_i_s_reg_686;
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        mul9_i_10_reg_681 <= grp_fu_293_p2;
        mul9_i_s_reg_686 <= grp_fu_298_p2;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln133_reg_563 == 1'd0) & (1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        mul9_i_1_reg_591 <= grp_fu_298_p2;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln133_reg_563 == 1'd0) & (1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        mul9_i_2_reg_606 <= grp_fu_293_p2;
        mul9_i_3_reg_611 <= grp_fu_298_p2;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln133_reg_563 == 1'd0) & (1'b0 == ap_block_pp0_stage3_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
        mul9_i_5_reg_626 <= grp_fu_298_p2;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln133_reg_563 == 1'd0) & (1'b0 == ap_block_pp0_stage4_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
        mul9_i_6_reg_641 <= grp_fu_293_p2;
        mul9_i_7_reg_646 <= grp_fu_298_p2;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln133_reg_563 == 1'd0) & (1'b0 == ap_block_pp0_stage5_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        mul9_i_8_reg_661 <= grp_fu_293_p2;
        mul9_i_9_reg_666 <= grp_fu_298_p2;
    end
end

always @ (posedge ap_clk) begin
    if ((((icmp_ln133_reg_563 == 1'd0) & (1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((icmp_ln133_reg_563 == 1'd0) & (1'b0 == ap_block_pp0_stage3_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3)))) begin
        reg_303 <= grp_fu_293_p2;
    end
end

always @ (*) begin
    if ((ap_enable_reg_pp0_iter0 == 1'b1)) begin
        if (((1'b0 == ap_block_pp0_stage5) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
            Adyn_1_address0 = zext_ln136_23_fu_482_p1;
        end else if (((1'b0 == ap_block_pp0_stage4) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
            Adyn_1_address0 = zext_ln136_21_fu_462_p1;
        end else if (((1'b0 == ap_block_pp0_stage3) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
            Adyn_1_address0 = zext_ln136_19_fu_442_p1;
        end else if (((1'b0 == ap_block_pp0_stage2) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
            Adyn_1_address0 = zext_ln136_17_fu_422_p1;
        end else if (((1'b0 == ap_block_pp0_stage1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
            Adyn_1_address0 = zext_ln136_15_fu_402_p1;
        end else if (((1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            Adyn_1_address0 = zext_ln136_13_fu_377_p1;
        end else begin
            Adyn_1_address0 = 'bx;
        end
    end else begin
        Adyn_1_address0 = 'bx;
    end
end

always @ (*) begin
    if ((ap_enable_reg_pp0_iter0 == 1'b1)) begin
        if (((1'b0 == ap_block_pp0_stage5) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
            Adyn_1_address1 = zext_ln136_22_fu_472_p1;
        end else if (((1'b0 == ap_block_pp0_stage4) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
            Adyn_1_address1 = zext_ln136_20_fu_452_p1;
        end else if (((1'b0 == ap_block_pp0_stage3) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
            Adyn_1_address1 = zext_ln136_18_fu_432_p1;
        end else if (((1'b0 == ap_block_pp0_stage2) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
            Adyn_1_address1 = zext_ln136_16_fu_412_p1;
        end else if (((1'b0 == ap_block_pp0_stage1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
            Adyn_1_address1 = zext_ln136_14_fu_392_p1;
        end else if (((1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            Adyn_1_address1 = zext_ln136_fu_366_p1;
        end else begin
            Adyn_1_address1 = 'bx;
        end
    end else begin
        Adyn_1_address1 = 'bx;
    end
end

always @ (*) begin
    if ((((1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2)) | ((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)) | ((1'b0 == ap_block_pp0_stage5_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5)) | ((1'b0 == ap_block_pp0_stage4_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4)) | ((1'b0 == ap_block_pp0_stage3_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3)))) begin
        Adyn_1_ce0 = 1'b1;
    end else begin
        Adyn_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if ((((1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2)) | ((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)) | ((1'b0 == ap_block_pp0_stage5_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5)) | ((1'b0 == ap_block_pp0_stage4_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4)) | ((1'b0 == ap_block_pp0_stage3_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3)))) begin
        Adyn_1_ce1 = 1'b1;
    end else begin
        Adyn_1_ce1 = 1'b0;
    end
end

always @ (*) begin
    if (((icmp_ln133_reg_563 == 1'd1) & (1'b0 == ap_block_pp0_stage5_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        ap_condition_exit_pp0_iter0_stage5 = 1'b1;
    end else begin
        ap_condition_exit_pp0_iter0_stage5 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage2_subdone) & (ap_enable_reg_pp0_iter1 == 1'b1) & (icmp_ln133_reg_563_pp0_iter1_reg == 1'd1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
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
        ap_sig_allocacmp_i_50 = 4'd0;
    end else begin
        ap_sig_allocacmp_i_50 = i_fu_86;
    end
end

always @ (*) begin
    if ((((1'b0 == ap_block_pp0_stage5) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5)) | ((1'b0 == ap_block_pp0_stage4) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4)) | ((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3)) | ((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)))) begin
        grp_fu_280_p0 = grp_fu_280_p2;
    end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        grp_fu_280_p0 = reg_303;
    end else begin
        grp_fu_280_p0 = 'bx;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        grp_fu_280_p1 = mul9_i_5_reg_626;
    end else if (((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        grp_fu_280_p1 = reg_303;
    end else if (((1'b0 == ap_block_pp0_stage5) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        grp_fu_280_p1 = mul9_i_3_reg_611;
    end else if (((1'b0 == ap_block_pp0_stage4) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
        grp_fu_280_p1 = mul9_i_2_reg_606;
    end else if (((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
        grp_fu_280_p1 = mul9_i_1_reg_591;
    end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        grp_fu_280_p1 = 32'd0;
    end else begin
        grp_fu_280_p1 = 'bx;
    end
end

always @ (*) begin
    if ((((1'b0 == ap_block_pp0_stage5) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5)) | ((1'b0 == ap_block_pp0_stage4) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4)) | ((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3)) | ((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)))) begin
        grp_fu_286_p0 = grp_fu_286_p2;
    end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        grp_fu_286_p0 = grp_fu_280_p2;
    end else begin
        grp_fu_286_p0 = 'bx;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        grp_fu_286_p1 = mul9_i_s_reg_686_pp0_iter2_reg;
    end else if (((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        grp_fu_286_p1 = mul9_i_10_reg_681;
    end else if (((1'b0 == ap_block_pp0_stage5) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        grp_fu_286_p1 = mul9_i_9_reg_666;
    end else if (((1'b0 == ap_block_pp0_stage4) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
        grp_fu_286_p1 = mul9_i_8_reg_661;
    end else if (((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
        grp_fu_286_p1 = mul9_i_7_reg_646;
    end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        grp_fu_286_p1 = mul9_i_6_reg_641;
    end else begin
        grp_fu_286_p1 = 'bx;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        grp_fu_293_p1 = x_col_load_10;
    end else if (((1'b0 == ap_block_pp0_stage5) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        grp_fu_293_p1 = x_col_load_8;
    end else if (((1'b0 == ap_block_pp0_stage4) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
        grp_fu_293_p1 = x_col_load_6;
    end else if (((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
        grp_fu_293_p1 = x_col_load_4;
    end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        grp_fu_293_p1 = x_col_load_2;
    end else if (((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        grp_fu_293_p1 = x_col_load;
    end else begin
        grp_fu_293_p1 = 'bx;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        grp_fu_298_p1 = x_col_load_11;
    end else if (((1'b0 == ap_block_pp0_stage5) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage5))) begin
        grp_fu_298_p1 = x_col_load_9;
    end else if (((1'b0 == ap_block_pp0_stage4) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage4))) begin
        grp_fu_298_p1 = x_col_load_7;
    end else if (((1'b0 == ap_block_pp0_stage3) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage3))) begin
        grp_fu_298_p1 = x_col_load_5;
    end else if (((1'b0 == ap_block_pp0_stage2) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        grp_fu_298_p1 = x_col_load_3;
    end else if (((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        grp_fu_298_p1 = x_col_load_1;
    end else begin
        grp_fu_298_p1 = 'bx;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        tiny_x1_ce0 = 1'b1;
    end else begin
        tiny_x1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage2_11001) & (ap_enable_reg_pp0_iter2 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage2))) begin
        tiny_x1_we0 = 1'b1;
    end else begin
        tiny_x1_we0 = 1'b0;
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

assign add_ln133_fu_334_p2 = (ap_sig_allocacmp_i_50 + 4'd1);

assign add_ln136_10_fu_477_p2 = (empty_118_reg_567 + 8'd11);

assign add_ln136_4_fu_407_p2 = (empty_118_reg_567 + 8'd4);

assign add_ln136_5_fu_417_p2 = (empty_118_reg_567 + 8'd5);

assign add_ln136_6_fu_427_p2 = (empty_118_reg_567 + 8'd6);

assign add_ln136_7_fu_437_p2 = (empty_118_reg_567 + 8'd7);

assign add_ln136_8_fu_457_p2 = (empty_118_reg_567 + 8'd9);

assign add_ln136_9_fu_467_p2 = (empty_118_reg_567 + 8'd10);

assign add_ln136_fu_447_p2 = (empty_118_reg_567 + 8'd8);

assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

assign ap_CS_fsm_pp0_stage1 = ap_CS_fsm[32'd1];

assign ap_CS_fsm_pp0_stage2 = ap_CS_fsm[32'd2];

assign ap_CS_fsm_pp0_stage3 = ap_CS_fsm[32'd3];

assign ap_CS_fsm_pp0_stage4 = ap_CS_fsm[32'd4];

assign ap_CS_fsm_pp0_stage5 = ap_CS_fsm[32'd5];

assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage1 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage1_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage1_subdone = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage2 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage2_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage2_subdone = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage3 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage3_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage3_subdone = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage4 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage4_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage4_subdone = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage5 = ~(1'b1 == 1'b1);

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

assign empty_118_fu_360_p2 = (p_shl2_fu_340_p3 - p_shl3_cast_fu_356_p1);

assign i_57_cast15_fu_487_p1 = i_50_reg_558_pp0_iter2_reg;

assign icmp_ln133_fu_328_p2 = ((ap_sig_allocacmp_i_50 == 4'd12) ? 1'b1 : 1'b0);

assign or_ln136_3_fu_387_p2 = (empty_118_reg_567 | 8'd2);

assign or_ln136_4_fu_397_p2 = (empty_118_reg_567 | 8'd3);

assign or_ln136_fu_371_p2 = (empty_118_fu_360_p2 | 8'd1);

assign p_shl2_fu_340_p3 = {{ap_sig_allocacmp_i_50}, {4'd0}};

assign p_shl3_cast_fu_356_p1 = p_shl3_fu_348_p3;

assign p_shl3_fu_348_p3 = {{ap_sig_allocacmp_i_50}, {2'd0}};

assign tiny_x1_address0 = i_57_cast15_fu_487_p1;

assign tiny_x1_d0 = grp_fu_286_p2;

assign zext_ln136_13_fu_377_p1 = or_ln136_fu_371_p2;

assign zext_ln136_14_fu_392_p1 = or_ln136_3_fu_387_p2;

assign zext_ln136_15_fu_402_p1 = or_ln136_4_fu_397_p2;

assign zext_ln136_16_fu_412_p1 = add_ln136_4_fu_407_p2;

assign zext_ln136_17_fu_422_p1 = add_ln136_5_fu_417_p2;

assign zext_ln136_18_fu_432_p1 = add_ln136_6_fu_427_p2;

assign zext_ln136_19_fu_442_p1 = add_ln136_7_fu_437_p2;

assign zext_ln136_20_fu_452_p1 = add_ln136_fu_447_p2;

assign zext_ln136_21_fu_462_p1 = add_ln136_8_fu_457_p2;

assign zext_ln136_22_fu_472_p1 = add_ln136_9_fu_467_p2;

assign zext_ln136_23_fu_482_p1 = add_ln136_10_fu_477_p2;

assign zext_ln136_fu_366_p1 = empty_118_fu_360_p2;

always @ (posedge ap_clk) begin
    empty_118_reg_567[1:0] <= 2'b00;
end

endmodule //tracking_forward_pass_2_Pipeline_VITIS_LOOP_133_1
