// ==============================================================
// RTL generated by Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.1 (64-bit)
// Version: 2022.1
// Copyright (C) Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

`timescale 1 ns / 1 ps 

module tracking_tiny_solve_Pipeline_VITIS_LOOP_80_1_VITIS_LOOP_81_2 (
        ap_clk,
        ap_rst,
        ap_start,
        ap_done,
        ap_idle,
        ap_ready,
        s1_1_address0,
        s1_1_ce0,
        s1_1_q0,
        s2_1_address0,
        s2_1_ce0,
        s2_1_we0,
        s2_1_d0
);

parameter    ap_ST_fsm_pp0_stage0 = 1'd1;

input   ap_clk;
input   ap_rst;
input   ap_start;
output   ap_done;
output   ap_idle;
output   ap_ready;
output  [6:0] s1_1_address0;
output   s1_1_ce0;
input  [31:0] s1_1_q0;
output  [6:0] s2_1_address0;
output   s2_1_ce0;
output   s2_1_we0;
output  [31:0] s2_1_d0;

reg ap_idle;
reg s1_1_ce0;
reg s2_1_ce0;
reg s2_1_we0;

(* fsm_encoding = "none" *) reg   [0:0] ap_CS_fsm;
wire    ap_CS_fsm_pp0_stage0;
wire    ap_enable_reg_pp0_iter0;
reg    ap_enable_reg_pp0_iter1;
reg    ap_idle_pp0;
wire    ap_block_state1_pp0_stage0_iter0;
wire    ap_block_state2_pp0_stage0_iter1;
wire    ap_block_pp0_stage0_subdone;
wire   [0:0] icmp_ln80_fu_137_p2;
reg    ap_condition_exit_pp0_iter0_stage0;
wire    ap_loop_exit_ready;
reg    ap_ready_int;
wire   [63:0] zext_ln82_fu_228_p1;
reg   [63:0] zext_ln82_reg_299;
wire    ap_block_pp0_stage0_11001;
wire    ap_block_pp0_stage0;
reg   [3:0] j_fu_48;
wire   [3:0] add_ln81_fu_233_p2;
wire    ap_loop_init;
reg   [3:0] ap_sig_allocacmp_j_load;
reg   [3:0] i_fu_52;
wire   [3:0] select_ln80_2_fu_210_p3;
reg   [3:0] ap_sig_allocacmp_i_31;
reg   [6:0] indvar_flatten9_fu_56;
wire   [6:0] add_ln80_3_fu_143_p2;
reg   [6:0] ap_sig_allocacmp_indvar_flatten9_load;
wire   [2:0] empty_fu_107_p1;
wire   [5:0] p_shl7_fu_119_p3;
wire   [6:0] p_shl6_fu_111_p3;
wire   [6:0] p_shl7_cast_fu_127_p1;
wire   [0:0] icmp_ln81_fu_158_p2;
wire   [3:0] add_ln80_fu_152_p2;
wire   [2:0] empty_93_fu_172_p1;
wire   [5:0] p_shl7_mid1_fu_184_p3;
wire   [6:0] p_shl6_mid1_fu_176_p3;
wire   [6:0] p_shl7_cast_mid1_fu_192_p1;
wire   [6:0] p_mid17_fu_196_p2;
wire   [6:0] empty_91_fu_131_p2;
wire   [3:0] select_ln80_fu_164_p3;
wire   [6:0] j_27_cast_fu_218_p1;
wire   [6:0] select_ln80_1_fu_202_p3;
wire   [6:0] add_ln82_fu_222_p2;
wire   [31:0] data_V_fu_254_p1;
wire   [30:0] trunc_ln368_fu_258_p1;
wire   [31:0] p_Result_s_fu_262_p3;
reg    ap_done_reg;
wire    ap_continue_int;
reg    ap_done_int;
reg   [0:0] ap_NS_fsm;
wire    ap_enable_pp0;
wire    ap_start_int;
wire    ap_ce_reg;

// power-on initialization
initial begin
#0 ap_CS_fsm = 1'd1;
#0 ap_enable_reg_pp0_iter1 = 1'b0;
#0 ap_done_reg = 1'b0;
end

tracking_flow_control_loop_pipe_sequential_init flow_control_loop_pipe_sequential_init_U(
    .ap_clk(ap_clk),
    .ap_rst(ap_rst),
    .ap_start(ap_start),
    .ap_ready(ap_ready),
    .ap_done(ap_done),
    .ap_start_int(ap_start_int),
    .ap_loop_init(ap_loop_init),
    .ap_ready_int(ap_ready_int),
    .ap_loop_exit_ready(ap_condition_exit_pp0_iter0_stage0),
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
        end else if (((ap_loop_exit_ready == 1'b1) & (1'b0 == ap_block_pp0_stage0_subdone) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            ap_done_reg <= 1'b1;
        end
    end
end

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        ap_enable_reg_pp0_iter1 <= 1'b0;
    end else begin
        if ((1'b1 == ap_condition_exit_pp0_iter0_stage0)) begin
            ap_enable_reg_pp0_iter1 <= 1'b0;
        end else if (((1'b0 == ap_block_pp0_stage0_subdone) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            ap_enable_reg_pp0_iter1 <= ap_start_int;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln80_fu_137_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            i_fu_52 <= select_ln80_2_fu_210_p3;
        end else if ((ap_loop_init == 1'b1)) begin
            i_fu_52 <= 4'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln80_fu_137_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            indvar_flatten9_fu_56 <= add_ln80_3_fu_143_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            indvar_flatten9_fu_56 <= 7'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln80_fu_137_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            j_fu_48 <= add_ln81_fu_233_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            j_fu_48 <= 4'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln80_fu_137_p2 == 1'd0) & (1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        zext_ln82_reg_299[6 : 0] <= zext_ln82_fu_228_p1[6 : 0];
    end
end

always @ (*) begin
    if (((icmp_ln80_fu_137_p2 == 1'd1) & (1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_condition_exit_pp0_iter0_stage0 = 1'b1;
    end else begin
        ap_condition_exit_pp0_iter0_stage0 = 1'b0;
    end
end

always @ (*) begin
    if (((ap_loop_exit_ready == 1'b1) & (1'b0 == ap_block_pp0_stage0_subdone) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_done_int = 1'b1;
    end else begin
        ap_done_int = ap_done_reg;
    end
end

always @ (*) begin
    if (((ap_idle_pp0 == 1'b1) & (ap_start_int == 1'b0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_idle = 1'b1;
    end else begin
        ap_idle = 1'b0;
    end
end

always @ (*) begin
    if (((ap_enable_reg_pp0_iter1 == 1'b0) & (ap_enable_reg_pp0_iter0 == 1'b0))) begin
        ap_idle_pp0 = 1'b1;
    end else begin
        ap_idle_pp0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_ready_int = 1'b1;
    end else begin
        ap_ready_int = 1'b0;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_i_31 = 4'd0;
    end else begin
        ap_sig_allocacmp_i_31 = i_fu_52;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_indvar_flatten9_load = 7'd0;
    end else begin
        ap_sig_allocacmp_indvar_flatten9_load = indvar_flatten9_fu_56;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_j_load = 4'd0;
    end else begin
        ap_sig_allocacmp_j_load = j_fu_48;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        s1_1_ce0 = 1'b1;
    end else begin
        s1_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        s2_1_ce0 = 1'b1;
    end else begin
        s2_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        s2_1_we0 = 1'b1;
    end else begin
        s2_1_we0 = 1'b0;
    end
end

always @ (*) begin
    case (ap_CS_fsm)
        ap_ST_fsm_pp0_stage0 : begin
            ap_NS_fsm = ap_ST_fsm_pp0_stage0;
        end
        default : begin
            ap_NS_fsm = 'bx;
        end
    endcase
end

assign add_ln80_3_fu_143_p2 = (ap_sig_allocacmp_indvar_flatten9_load + 7'd1);

assign add_ln80_fu_152_p2 = (ap_sig_allocacmp_i_31 + 4'd1);

assign add_ln81_fu_233_p2 = (select_ln80_fu_164_p3 + 4'd1);

assign add_ln82_fu_222_p2 = (j_27_cast_fu_218_p1 + select_ln80_1_fu_202_p3);

assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

assign ap_block_state1_pp0_stage0_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state2_pp0_stage0_iter1 = ~(1'b1 == 1'b1);

assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

assign ap_enable_reg_pp0_iter0 = ap_start_int;

assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage0;

assign data_V_fu_254_p1 = s1_1_q0;

assign empty_91_fu_131_p2 = (p_shl6_fu_111_p3 - p_shl7_cast_fu_127_p1);

assign empty_93_fu_172_p1 = add_ln80_fu_152_p2[2:0];

assign empty_fu_107_p1 = ap_sig_allocacmp_i_31[2:0];

assign icmp_ln80_fu_137_p2 = ((ap_sig_allocacmp_indvar_flatten9_load == 7'd120) ? 1'b1 : 1'b0);

assign icmp_ln81_fu_158_p2 = ((ap_sig_allocacmp_j_load == 4'd12) ? 1'b1 : 1'b0);

assign j_27_cast_fu_218_p1 = select_ln80_fu_164_p3;

assign p_Result_s_fu_262_p3 = {{1'd0}, {trunc_ln368_fu_258_p1}};

assign p_mid17_fu_196_p2 = (p_shl6_mid1_fu_176_p3 - p_shl7_cast_mid1_fu_192_p1);

assign p_shl6_fu_111_p3 = {{empty_fu_107_p1}, {4'd0}};

assign p_shl6_mid1_fu_176_p3 = {{empty_93_fu_172_p1}, {4'd0}};

assign p_shl7_cast_fu_127_p1 = p_shl7_fu_119_p3;

assign p_shl7_cast_mid1_fu_192_p1 = p_shl7_mid1_fu_184_p3;

assign p_shl7_fu_119_p3 = {{ap_sig_allocacmp_i_31}, {2'd0}};

assign p_shl7_mid1_fu_184_p3 = {{add_ln80_fu_152_p2}, {2'd0}};

assign s1_1_address0 = zext_ln82_fu_228_p1;

assign s2_1_address0 = zext_ln82_reg_299;

assign s2_1_d0 = p_Result_s_fu_262_p3;

assign select_ln80_1_fu_202_p3 = ((icmp_ln81_fu_158_p2[0:0] == 1'b1) ? p_mid17_fu_196_p2 : empty_91_fu_131_p2);

assign select_ln80_2_fu_210_p3 = ((icmp_ln81_fu_158_p2[0:0] == 1'b1) ? add_ln80_fu_152_p2 : ap_sig_allocacmp_i_31);

assign select_ln80_fu_164_p3 = ((icmp_ln81_fu_158_p2[0:0] == 1'b1) ? 4'd0 : ap_sig_allocacmp_j_load);

assign trunc_ln368_fu_258_p1 = data_V_fu_254_p1[30:0];

assign zext_ln82_fu_228_p1 = add_ln82_fu_222_p2;

always @ (posedge ap_clk) begin
    zext_ln82_reg_299[63:7] <= 57'b000000000000000000000000000000000000000000000000000000000;
end

endmodule //tracking_tiny_solve_Pipeline_VITIS_LOOP_80_1_VITIS_LOOP_81_2
