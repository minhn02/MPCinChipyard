// ==============================================================
// RTL generated by Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.1 (64-bit)
// Version: 2022.1
// Copyright (C) Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

`timescale 1 ns / 1 ps 

module tracking_update_slack_Pipeline_VITIS_LOOP_89_1_VITIS_LOOP_90_2 (
        ap_clk,
        ap_rst,
        ap_start,
        ap_done,
        ap_idle,
        ap_ready,
        x_max_1_address0,
        x_max_1_ce0,
        x_max_1_q0,
        s1_1_address0,
        s1_1_ce0,
        s1_1_q0,
        vnew_1_address0,
        vnew_1_ce0,
        vnew_1_we0,
        vnew_1_d0
);

parameter    ap_ST_fsm_pp0_stage0 = 1'd1;

input   ap_clk;
input   ap_rst;
input   ap_start;
output   ap_done;
output   ap_idle;
output   ap_ready;
output  [6:0] x_max_1_address0;
output   x_max_1_ce0;
input  [31:0] x_max_1_q0;
output  [6:0] s1_1_address0;
output   s1_1_ce0;
input  [31:0] s1_1_q0;
output  [6:0] vnew_1_address0;
output   vnew_1_ce0;
output   vnew_1_we0;
output  [31:0] vnew_1_d0;

reg ap_idle;
reg x_max_1_ce0;
reg s1_1_ce0;
reg vnew_1_ce0;
reg vnew_1_we0;

(* fsm_encoding = "none" *) reg   [0:0] ap_CS_fsm;
wire    ap_CS_fsm_pp0_stage0;
wire    ap_enable_reg_pp0_iter0;
reg    ap_enable_reg_pp0_iter1;
reg    ap_idle_pp0;
wire    ap_block_state1_pp0_stage0_iter0;
wire    ap_block_state2_pp0_stage0_iter1;
wire    ap_block_pp0_stage0_subdone;
wire   [0:0] icmp_ln89_fu_164_p2;
reg    ap_condition_exit_pp0_iter0_stage0;
wire    ap_loop_exit_ready;
reg    ap_ready_int;
wire   [63:0] zext_ln91_fu_255_p1;
reg   [63:0] zext_ln91_reg_399;
wire    ap_block_pp0_stage0_11001;
wire    ap_block_pp0_stage0;
reg   [3:0] j_fu_56;
wire   [3:0] add_ln90_fu_261_p2;
wire    ap_loop_init;
reg   [3:0] ap_sig_allocacmp_j_load;
reg   [3:0] i_fu_60;
wire   [3:0] select_ln89_2_fu_237_p3;
reg   [3:0] ap_sig_allocacmp_i_1;
reg   [6:0] indvar_flatten19_fu_64;
wire   [6:0] add_ln89_fu_170_p2;
reg   [6:0] ap_sig_allocacmp_indvar_flatten19_load;
wire   [2:0] empty_fu_134_p1;
wire   [5:0] p_shl5_fu_146_p3;
wire   [6:0] p_shl4_fu_138_p3;
wire   [6:0] p_shl5_cast_fu_154_p1;
wire   [0:0] icmp_ln90_fu_185_p2;
wire   [3:0] add_ln89_1_fu_179_p2;
wire   [2:0] empty_41_fu_199_p1;
wire   [5:0] p_shl5_mid1_fu_211_p3;
wire   [6:0] p_shl4_mid1_fu_203_p3;
wire   [6:0] p_shl5_cast_mid1_fu_219_p1;
wire   [6:0] p_mid117_fu_223_p2;
wire   [6:0] empty_39_fu_158_p2;
wire   [3:0] select_ln89_fu_191_p3;
wire   [6:0] j_5_cast_fu_245_p1;
wire   [6:0] select_ln89_1_fu_229_p3;
wire   [6:0] add_ln91_fu_249_p2;
wire   [31:0] bitcast_ln91_fu_282_p1;
wire   [31:0] bitcast_ln91_1_fu_300_p1;
wire   [7:0] tmp_9_fu_286_p4;
wire   [22:0] trunc_ln91_fu_296_p1;
wire   [0:0] icmp_ln91_1_fu_324_p2;
wire   [0:0] icmp_ln91_fu_318_p2;
wire   [7:0] tmp_s_fu_304_p4;
wire   [22:0] trunc_ln91_1_fu_314_p1;
wire   [0:0] icmp_ln91_3_fu_342_p2;
wire   [0:0] icmp_ln91_2_fu_336_p2;
wire   [0:0] or_ln91_fu_330_p2;
wire   [0:0] or_ln91_1_fu_348_p2;
wire   [0:0] and_ln91_fu_354_p2;
wire   [0:0] tmp_1_fu_107_p2;
wire   [0:0] and_ln91_1_fu_360_p2;
wire    ap_block_pp0_stage0_00001;
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

tracking_fcmp_32ns_32ns_1_1_no_dsp_1 #(
    .ID( 1 ),
    .NUM_STAGE( 1 ),
    .din0_WIDTH( 32 ),
    .din1_WIDTH( 32 ),
    .dout_WIDTH( 1 ))
fcmp_32ns_32ns_1_1_no_dsp_1_U89(
    .din0(x_max_1_q0),
    .din1(s1_1_q0),
    .opcode(5'd4),
    .dout(tmp_1_fu_107_p2)
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
        if (((icmp_ln89_fu_164_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            i_fu_60 <= select_ln89_2_fu_237_p3;
        end else if ((ap_loop_init == 1'b1)) begin
            i_fu_60 <= 4'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln89_fu_164_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            indvar_flatten19_fu_64 <= add_ln89_fu_170_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            indvar_flatten19_fu_64 <= 7'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln89_fu_164_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            j_fu_56 <= add_ln90_fu_261_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            j_fu_56 <= 4'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln89_fu_164_p2 == 1'd0) & (1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        zext_ln91_reg_399[6 : 0] <= zext_ln91_fu_255_p1[6 : 0];
    end
end

always @ (*) begin
    if (((icmp_ln89_fu_164_p2 == 1'd1) & (1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
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
    if (((ap_start_int == 1'b0) & (ap_idle_pp0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
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
        ap_sig_allocacmp_i_1 = 4'd0;
    end else begin
        ap_sig_allocacmp_i_1 = i_fu_60;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_indvar_flatten19_load = 7'd0;
    end else begin
        ap_sig_allocacmp_indvar_flatten19_load = indvar_flatten19_fu_64;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_j_load = 4'd0;
    end else begin
        ap_sig_allocacmp_j_load = j_fu_56;
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
        vnew_1_ce0 = 1'b1;
    end else begin
        vnew_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        vnew_1_we0 = 1'b1;
    end else begin
        vnew_1_we0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        x_max_1_ce0 = 1'b1;
    end else begin
        x_max_1_ce0 = 1'b0;
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

assign add_ln89_1_fu_179_p2 = (ap_sig_allocacmp_i_1 + 4'd1);

assign add_ln89_fu_170_p2 = (ap_sig_allocacmp_indvar_flatten19_load + 7'd1);

assign add_ln90_fu_261_p2 = (select_ln89_fu_191_p3 + 4'd1);

assign add_ln91_fu_249_p2 = (j_5_cast_fu_245_p1 + select_ln89_1_fu_229_p3);

assign and_ln91_1_fu_360_p2 = (tmp_1_fu_107_p2 & and_ln91_fu_354_p2);

assign and_ln91_fu_354_p2 = (or_ln91_fu_330_p2 & or_ln91_1_fu_348_p2);

assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_00001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

assign ap_block_state1_pp0_stage0_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state2_pp0_stage0_iter1 = ~(1'b1 == 1'b1);

assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

assign ap_enable_reg_pp0_iter0 = ap_start_int;

assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage0;

assign bitcast_ln91_1_fu_300_p1 = s1_1_q0;

assign bitcast_ln91_fu_282_p1 = x_max_1_q0;

assign empty_39_fu_158_p2 = (p_shl4_fu_138_p3 - p_shl5_cast_fu_154_p1);

assign empty_41_fu_199_p1 = add_ln89_1_fu_179_p2[2:0];

assign empty_fu_134_p1 = ap_sig_allocacmp_i_1[2:0];

assign icmp_ln89_fu_164_p2 = ((ap_sig_allocacmp_indvar_flatten19_load == 7'd120) ? 1'b1 : 1'b0);

assign icmp_ln90_fu_185_p2 = ((ap_sig_allocacmp_j_load == 4'd12) ? 1'b1 : 1'b0);

assign icmp_ln91_1_fu_324_p2 = ((trunc_ln91_fu_296_p1 == 23'd0) ? 1'b1 : 1'b0);

assign icmp_ln91_2_fu_336_p2 = ((tmp_s_fu_304_p4 != 8'd255) ? 1'b1 : 1'b0);

assign icmp_ln91_3_fu_342_p2 = ((trunc_ln91_1_fu_314_p1 == 23'd0) ? 1'b1 : 1'b0);

assign icmp_ln91_fu_318_p2 = ((tmp_9_fu_286_p4 != 8'd255) ? 1'b1 : 1'b0);

assign j_5_cast_fu_245_p1 = select_ln89_fu_191_p3;

assign or_ln91_1_fu_348_p2 = (icmp_ln91_3_fu_342_p2 | icmp_ln91_2_fu_336_p2);

assign or_ln91_fu_330_p2 = (icmp_ln91_fu_318_p2 | icmp_ln91_1_fu_324_p2);

assign p_mid117_fu_223_p2 = (p_shl4_mid1_fu_203_p3 - p_shl5_cast_mid1_fu_219_p1);

assign p_shl4_fu_138_p3 = {{empty_fu_134_p1}, {4'd0}};

assign p_shl4_mid1_fu_203_p3 = {{empty_41_fu_199_p1}, {4'd0}};

assign p_shl5_cast_fu_154_p1 = p_shl5_fu_146_p3;

assign p_shl5_cast_mid1_fu_219_p1 = p_shl5_mid1_fu_211_p3;

assign p_shl5_fu_146_p3 = {{ap_sig_allocacmp_i_1}, {2'd0}};

assign p_shl5_mid1_fu_211_p3 = {{add_ln89_1_fu_179_p2}, {2'd0}};

assign s1_1_address0 = zext_ln91_fu_255_p1;

assign select_ln89_1_fu_229_p3 = ((icmp_ln90_fu_185_p2[0:0] == 1'b1) ? p_mid117_fu_223_p2 : empty_39_fu_158_p2);

assign select_ln89_2_fu_237_p3 = ((icmp_ln90_fu_185_p2[0:0] == 1'b1) ? add_ln89_1_fu_179_p2 : ap_sig_allocacmp_i_1);

assign select_ln89_fu_191_p3 = ((icmp_ln90_fu_185_p2[0:0] == 1'b1) ? 4'd0 : ap_sig_allocacmp_j_load);

assign tmp_9_fu_286_p4 = {{bitcast_ln91_fu_282_p1[30:23]}};

assign tmp_s_fu_304_p4 = {{bitcast_ln91_1_fu_300_p1[30:23]}};

assign trunc_ln91_1_fu_314_p1 = bitcast_ln91_1_fu_300_p1[22:0];

assign trunc_ln91_fu_296_p1 = bitcast_ln91_fu_282_p1[22:0];

assign vnew_1_address0 = zext_ln91_reg_399;

assign vnew_1_d0 = ((and_ln91_1_fu_360_p2[0:0] == 1'b1) ? x_max_1_q0 : s1_1_q0);

assign x_max_1_address0 = zext_ln91_fu_255_p1;

assign zext_ln91_fu_255_p1 = add_ln91_fu_249_p2;

always @ (posedge ap_clk) begin
    zext_ln91_reg_399[63:7] <= 57'b000000000000000000000000000000000000000000000000000000000;
end

endmodule //tracking_update_slack_Pipeline_VITIS_LOOP_89_1_VITIS_LOOP_90_2