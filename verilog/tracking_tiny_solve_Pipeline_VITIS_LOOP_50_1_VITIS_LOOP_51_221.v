// ==============================================================
// RTL generated by Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.1 (64-bit)
// Version: 2022.1
// Copyright (C) Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

`timescale 1 ns / 1 ps 

module tracking_tiny_solve_Pipeline_VITIS_LOOP_50_1_VITIS_LOOP_51_221 (
        ap_clk,
        ap_rst,
        ap_start,
        ap_done,
        ap_idle,
        ap_ready,
        max_3_out,
        max_3_out_ap_vld,
        m2_1_address0,
        m2_1_ce0,
        m2_1_q0
);

parameter    ap_ST_fsm_pp0_stage0 = 1'd1;

input   ap_clk;
input   ap_rst;
input   ap_start;
output   ap_done;
output   ap_idle;
output   ap_ready;
output  [31:0] max_3_out;
output   max_3_out_ap_vld;
output  [5:0] m2_1_address0;
output   m2_1_ce0;
input  [31:0] m2_1_q0;

reg ap_idle;
reg max_3_out_ap_vld;
reg m2_1_ce0;

(* fsm_encoding = "none" *) reg   [0:0] ap_CS_fsm;
wire    ap_CS_fsm_pp0_stage0;
wire    ap_enable_reg_pp0_iter0;
reg    ap_enable_reg_pp0_iter1;
reg    ap_idle_pp0;
wire    ap_block_state1_pp0_stage0_iter0;
wire    ap_block_state2_pp0_stage0_iter1;
wire    ap_block_pp0_stage0_subdone;
wire   [0:0] icmp_ln50_fu_124_p2;
reg    ap_condition_exit_pp0_iter0_stage0;
wire    ap_loop_exit_ready;
reg    ap_ready_int;
wire    ap_block_pp0_stage0_11001;
wire   [63:0] zext_ln52_fu_188_p1;
wire    ap_block_pp0_stage0;
reg   [31:0] max_fu_60;
wire   [31:0] max_4_fu_302_p3;
wire    ap_loop_init;
reg   [2:0] j_fu_64;
wire   [2:0] add_ln51_fu_193_p2;
reg   [2:0] ap_sig_allocacmp_j_load;
reg   [3:0] i_fu_68;
wire   [3:0] select_ln50_2_fu_162_p3;
reg   [3:0] ap_sig_allocacmp_i_load;
reg   [5:0] indvar_flatten79_fu_72;
wire   [5:0] add_ln50_fu_130_p2;
reg   [5:0] ap_sig_allocacmp_indvar_flatten79_load;
wire    ap_block_pp0_stage0_01001;
wire   [0:0] icmp_ln51_fu_148_p2;
wire   [3:0] add_ln50_2_fu_142_p2;
wire   [2:0] select_ln50_fu_154_p3;
wire   [5:0] j_34_cast_fu_178_p1;
wire   [5:0] p_mid3_fu_170_p3;
wire   [5:0] add_ln52_fu_182_p2;
wire   [31:0] bitcast_ln52_fu_218_p1;
wire   [31:0] bitcast_ln52_2_fu_236_p1;
wire   [7:0] tmp_s_fu_222_p4;
wire   [22:0] trunc_ln52_fu_232_p1;
wire   [0:0] icmp_ln52_4_fu_260_p2;
wire   [0:0] icmp_ln52_fu_254_p2;
wire   [7:0] tmp_10_fu_240_p4;
wire   [22:0] trunc_ln52_2_fu_250_p1;
wire   [0:0] icmp_ln52_6_fu_278_p2;
wire   [0:0] icmp_ln52_5_fu_272_p2;
wire   [0:0] or_ln52_fu_266_p2;
wire   [0:0] or_ln52_2_fu_284_p2;
wire   [0:0] and_ln52_fu_290_p2;
wire   [0:0] tmp_11_fu_96_p2;
wire   [0:0] and_ln52_2_fu_296_p2;
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
fcmp_32ns_32ns_1_1_no_dsp_1_U386(
    .din0(m2_1_q0),
    .din1(max_fu_60),
    .opcode(5'd2),
    .dout(tmp_11_fu_96_p2)
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
        if (((icmp_ln50_fu_124_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            i_fu_68 <= select_ln50_2_fu_162_p3;
        end else if ((ap_loop_init == 1'b1)) begin
            i_fu_68 <= 4'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln50_fu_124_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            indvar_flatten79_fu_72 <= add_ln50_fu_130_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            indvar_flatten79_fu_72 <= 6'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln50_fu_124_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            j_fu_64 <= add_ln51_fu_193_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            j_fu_64 <= 3'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if ((ap_loop_init == 1'b1)) begin
            max_fu_60 <= 32'd8388608;
        end else if ((ap_enable_reg_pp0_iter1 == 1'b1)) begin
            max_fu_60 <= max_4_fu_302_p3;
        end
    end
end

always @ (*) begin
    if (((icmp_ln50_fu_124_p2 == 1'd1) & (1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
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
        ap_sig_allocacmp_i_load = 4'd0;
    end else begin
        ap_sig_allocacmp_i_load = i_fu_68;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_indvar_flatten79_load = 6'd0;
    end else begin
        ap_sig_allocacmp_indvar_flatten79_load = indvar_flatten79_fu_72;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_j_load = 3'd0;
    end else begin
        ap_sig_allocacmp_j_load = j_fu_64;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        m2_1_ce0 = 1'b1;
    end else begin
        m2_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if (((icmp_ln50_fu_124_p2 == 1'd1) & (1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        max_3_out_ap_vld = 1'b1;
    end else begin
        max_3_out_ap_vld = 1'b0;
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

assign add_ln50_2_fu_142_p2 = (ap_sig_allocacmp_i_load + 4'd1);

assign add_ln50_fu_130_p2 = (ap_sig_allocacmp_indvar_flatten79_load + 6'd1);

assign add_ln51_fu_193_p2 = (select_ln50_fu_154_p3 + 3'd1);

assign add_ln52_fu_182_p2 = (j_34_cast_fu_178_p1 + p_mid3_fu_170_p3);

assign and_ln52_2_fu_296_p2 = (tmp_11_fu_96_p2 & and_ln52_fu_290_p2);

assign and_ln52_fu_290_p2 = (or_ln52_fu_266_p2 & or_ln52_2_fu_284_p2);

assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_00001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_01001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

assign ap_block_state1_pp0_stage0_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state2_pp0_stage0_iter1 = ~(1'b1 == 1'b1);

assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

assign ap_enable_reg_pp0_iter0 = ap_start_int;

assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage0;

assign bitcast_ln52_2_fu_236_p1 = max_fu_60;

assign bitcast_ln52_fu_218_p1 = m2_1_q0;

assign icmp_ln50_fu_124_p2 = ((ap_sig_allocacmp_indvar_flatten79_load == 6'd36) ? 1'b1 : 1'b0);

assign icmp_ln51_fu_148_p2 = ((ap_sig_allocacmp_j_load == 3'd4) ? 1'b1 : 1'b0);

assign icmp_ln52_4_fu_260_p2 = ((trunc_ln52_fu_232_p1 == 23'd0) ? 1'b1 : 1'b0);

assign icmp_ln52_5_fu_272_p2 = ((tmp_10_fu_240_p4 != 8'd255) ? 1'b1 : 1'b0);

assign icmp_ln52_6_fu_278_p2 = ((trunc_ln52_2_fu_250_p1 == 23'd0) ? 1'b1 : 1'b0);

assign icmp_ln52_fu_254_p2 = ((tmp_s_fu_222_p4 != 8'd255) ? 1'b1 : 1'b0);

assign j_34_cast_fu_178_p1 = select_ln50_fu_154_p3;

assign m2_1_address0 = zext_ln52_fu_188_p1;

assign max_3_out = max_fu_60;

assign max_4_fu_302_p3 = ((and_ln52_2_fu_296_p2[0:0] == 1'b1) ? m2_1_q0 : max_fu_60);

assign or_ln52_2_fu_284_p2 = (icmp_ln52_6_fu_278_p2 | icmp_ln52_5_fu_272_p2);

assign or_ln52_fu_266_p2 = (icmp_ln52_fu_254_p2 | icmp_ln52_4_fu_260_p2);

assign p_mid3_fu_170_p3 = {{select_ln50_2_fu_162_p3}, {2'd0}};

assign select_ln50_2_fu_162_p3 = ((icmp_ln51_fu_148_p2[0:0] == 1'b1) ? add_ln50_2_fu_142_p2 : ap_sig_allocacmp_i_load);

assign select_ln50_fu_154_p3 = ((icmp_ln51_fu_148_p2[0:0] == 1'b1) ? 3'd0 : ap_sig_allocacmp_j_load);

assign tmp_10_fu_240_p4 = {{bitcast_ln52_2_fu_236_p1[30:23]}};

assign tmp_s_fu_222_p4 = {{bitcast_ln52_fu_218_p1[30:23]}};

assign trunc_ln52_2_fu_250_p1 = bitcast_ln52_2_fu_236_p1[22:0];

assign trunc_ln52_fu_232_p1 = bitcast_ln52_fu_218_p1[22:0];

assign zext_ln52_fu_188_p1 = add_ln52_fu_182_p2;

endmodule //tracking_tiny_solve_Pipeline_VITIS_LOOP_50_1_VITIS_LOOP_51_221
