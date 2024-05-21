// ==============================================================
// RTL generated by Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.1 (64-bit)
// Version: 2022.1
// Copyright (C) Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

`timescale 1 ns / 1 ps 

module tracking_update_linear_cost_4_Pipeline_VITIS_LOOP_86_14 (
        ap_clk,
        ap_rst,
        ap_start,
        ap_done,
        ap_idle,
        ap_ready,
        g_col_h1_address0,
        g_col_h1_ce0,
        g_col_h1_we0,
        g_col_h1_d0,
        g_col_h1_address1,
        g_col_h1_ce1,
        g_col_h1_we1,
        g_col_h1_d1,
        g_1_address0,
        g_1_ce0,
        g_1_q0,
        g_1_address1,
        g_1_ce1,
        g_1_q1
);

parameter    ap_ST_fsm_pp0_stage0 = 2'd1;
parameter    ap_ST_fsm_pp0_stage1 = 2'd2;

input   ap_clk;
input   ap_rst;
input   ap_start;
output   ap_done;
output   ap_idle;
output   ap_ready;
output  [3:0] g_col_h1_address0;
output   g_col_h1_ce0;
output   g_col_h1_we0;
output  [31:0] g_col_h1_d0;
output  [3:0] g_col_h1_address1;
output   g_col_h1_ce1;
output   g_col_h1_we1;
output  [31:0] g_col_h1_d1;
output  [6:0] g_1_address0;
output   g_1_ce0;
input  [31:0] g_1_q0;
output  [6:0] g_1_address1;
output   g_1_ce1;
input  [31:0] g_1_q1;

reg ap_idle;
reg[3:0] g_col_h1_address0;
reg g_col_h1_ce0;
reg g_col_h1_we0;
reg[3:0] g_col_h1_address1;
reg g_col_h1_ce1;
reg g_col_h1_we1;
reg[6:0] g_1_address0;
reg g_1_ce0;
reg[6:0] g_1_address1;
reg g_1_ce1;

(* fsm_encoding = "none" *) reg   [1:0] ap_CS_fsm;
wire    ap_CS_fsm_pp0_stage0;
reg    ap_enable_reg_pp0_iter0;
reg    ap_enable_reg_pp0_iter1;
reg    ap_idle_pp0;
wire    ap_block_state1_pp0_stage0_iter0;
wire    ap_block_state3_pp0_stage0_iter1;
wire    ap_block_pp0_stage0_subdone;
wire   [0:0] icmp_ln86_fu_136_p2;
reg    ap_condition_exit_pp0_iter0_stage0;
wire    ap_loop_exit_ready;
reg    ap_ready_int;
wire    ap_CS_fsm_pp0_stage1;
wire    ap_block_state2_pp0_stage1_iter0;
wire    ap_block_pp0_stage1_subdone;
reg   [3:0] i_reg_261;
wire    ap_block_pp0_stage0_11001;
reg   [0:0] icmp_ln86_reg_269;
wire   [3:0] or_ln86_4_fu_201_p2;
reg   [3:0] or_ln86_4_reg_283;
wire    ap_block_pp0_stage1_11001;
reg    ap_enable_reg_pp0_iter0_reg;
wire   [63:0] zext_ln88_fu_156_p1;
wire    ap_block_pp0_stage0;
wire   [63:0] zext_ln88_29_fu_171_p1;
wire   [63:0] i_6_cast9_fu_187_p1;
wire    ap_block_pp0_stage1;
wire   [63:0] zext_ln88_9_fu_196_p1;
wire   [63:0] zext_ln88_30_fu_220_p1;
wire   [63:0] zext_ln88_31_fu_235_p1;
wire   [63:0] zext_ln88_11_fu_240_p1;
wire   [63:0] zext_ln88_14_fu_249_p1;
reg   [3:0] i_6_fu_40;
wire   [3:0] add_ln86_fu_176_p2;
wire    ap_loop_init;
reg   [3:0] ap_sig_allocacmp_i;
wire   [5:0] i_6_cast24_fu_142_p1;
wire   [5:0] add_ln88_4_fu_146_p2;
wire  signed [6:0] sext_ln88_fu_152_p1;
wire   [5:0] add_ln88_5_fu_161_p2;
wire  signed [6:0] sext_ln88_7_fu_167_p1;
wire   [3:0] or_ln86_3_fu_191_p2;
wire   [5:0] zext_ln88_12_fu_206_p1;
wire   [5:0] add_ln88_6_fu_210_p2;
wire  signed [6:0] sext_ln88_8_fu_216_p1;
wire   [5:0] add_ln88_7_fu_225_p2;
wire  signed [6:0] sext_ln88_9_fu_231_p1;
wire   [3:0] or_ln86_5_fu_244_p2;
reg    ap_done_reg;
wire    ap_continue_int;
reg    ap_done_int;
reg   [1:0] ap_NS_fsm;
reg    ap_idle_pp0_1to1;
wire    ap_enable_pp0;
wire    ap_start_int;
wire    ap_ce_reg;

// power-on initialization
initial begin
#0 ap_CS_fsm = 2'd1;
#0 ap_enable_reg_pp0_iter1 = 1'b0;
#0 ap_enable_reg_pp0_iter0_reg = 1'b0;
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
        ap_enable_reg_pp0_iter0_reg <= 1'b0;
    end else begin
        if ((1'b1 == ap_condition_exit_pp0_iter0_stage0)) begin
            ap_enable_reg_pp0_iter0_reg <= 1'b0;
        end else if ((1'b1 == ap_CS_fsm_pp0_stage0)) begin
            ap_enable_reg_pp0_iter0_reg <= ap_start_int;
        end
    end
end

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        ap_enable_reg_pp0_iter1 <= 1'b0;
    end else begin
        if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            ap_enable_reg_pp0_iter1 <= 1'b0;
        end else if (((1'b0 == ap_block_pp0_stage1_subdone) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
            ap_enable_reg_pp0_iter1 <= ap_enable_reg_pp0_iter0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln86_fu_136_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            i_6_fu_40 <= add_ln86_fu_176_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            i_6_fu_40 <= 4'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        i_reg_261 <= ap_sig_allocacmp_i;
        icmp_ln86_reg_269 <= icmp_ln86_fu_136_p2;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln86_reg_269 == 1'd0) & (1'b0 == ap_block_pp0_stage1_11001) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        or_ln86_4_reg_283[0] <= or_ln86_4_fu_201_p2[0];
or_ln86_4_reg_283[3 : 2] <= or_ln86_4_fu_201_p2[3 : 2];
    end
end

always @ (*) begin
    if (((icmp_ln86_fu_136_p2 == 1'd1) & (1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
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
    if ((1'b1 == ap_CS_fsm_pp0_stage0)) begin
        ap_enable_reg_pp0_iter0 = ap_start_int;
    end else begin
        ap_enable_reg_pp0_iter0 = ap_enable_reg_pp0_iter0_reg;
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
    if ((ap_enable_reg_pp0_iter1 == 1'b0)) begin
        ap_idle_pp0_1to1 = 1'b1;
    end else begin
        ap_idle_pp0_1to1 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage1_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        ap_ready_int = 1'b1;
    end else begin
        ap_ready_int = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0) & (ap_loop_init == 1'b1))) begin
        ap_sig_allocacmp_i = 4'd0;
    end else begin
        ap_sig_allocacmp_i = i_6_fu_40;
    end
end

always @ (*) begin
    if ((ap_enable_reg_pp0_iter0 == 1'b1)) begin
        if (((1'b0 == ap_block_pp0_stage1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
            g_1_address0 = zext_ln88_31_fu_235_p1;
        end else if (((1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            g_1_address0 = zext_ln88_29_fu_171_p1;
        end else begin
            g_1_address0 = 'bx;
        end
    end else begin
        g_1_address0 = 'bx;
    end
end

always @ (*) begin
    if ((ap_enable_reg_pp0_iter0 == 1'b1)) begin
        if (((1'b0 == ap_block_pp0_stage1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
            g_1_address1 = zext_ln88_30_fu_220_p1;
        end else if (((1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
            g_1_address1 = zext_ln88_fu_156_p1;
        end else begin
            g_1_address1 = 'bx;
        end
    end else begin
        g_1_address1 = 'bx;
    end
end

always @ (*) begin
    if ((((1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)))) begin
        g_1_ce0 = 1'b1;
    end else begin
        g_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if ((((1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)))) begin
        g_1_ce1 = 1'b1;
    end else begin
        g_1_ce1 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        g_col_h1_address0 = zext_ln88_14_fu_249_p1;
    end else if (((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        g_col_h1_address0 = zext_ln88_9_fu_196_p1;
    end else begin
        g_col_h1_address0 = 'bx;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        g_col_h1_address1 = zext_ln88_11_fu_240_p1;
    end else if (((1'b0 == ap_block_pp0_stage1) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1))) begin
        g_col_h1_address1 = i_6_cast9_fu_187_p1;
    end else begin
        g_col_h1_address1 = 'bx;
    end
end

always @ (*) begin
    if ((((1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)))) begin
        g_col_h1_ce0 = 1'b1;
    end else begin
        g_col_h1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if ((((1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)))) begin
        g_col_h1_ce1 = 1'b1;
    end else begin
        g_col_h1_ce1 = 1'b0;
    end
end

always @ (*) begin
    if ((((icmp_ln86_reg_269 == 1'd0) & (1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)))) begin
        g_col_h1_we0 = 1'b1;
    end else begin
        g_col_h1_we0 = 1'b0;
    end
end

always @ (*) begin
    if ((((icmp_ln86_reg_269 == 1'd0) & (1'b0 == ap_block_pp0_stage1_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage1)) | ((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0)))) begin
        g_col_h1_we1 = 1'b1;
    end else begin
        g_col_h1_we1 = 1'b0;
    end
end

always @ (*) begin
    case (ap_CS_fsm)
        ap_ST_fsm_pp0_stage0 : begin
            if ((1'b1 == ap_condition_exit_pp0_iter0_stage0)) begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage0;
            end else if ((~((ap_start_int == 1'b0) & (ap_idle_pp0_1to1 == 1'b1)) & (1'b0 == ap_block_pp0_stage0_subdone))) begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage1;
            end else begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage0;
            end
        end
        ap_ST_fsm_pp0_stage1 : begin
            if ((1'b0 == ap_block_pp0_stage1_subdone)) begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage0;
            end else begin
                ap_NS_fsm = ap_ST_fsm_pp0_stage1;
            end
        end
        default : begin
            ap_NS_fsm = 'bx;
        end
    endcase
end

assign add_ln86_fu_176_p2 = (ap_sig_allocacmp_i + 4'd4);

assign add_ln88_4_fu_146_p2 = ($signed(i_6_cast24_fu_142_p1) + $signed(6'd44));

assign add_ln88_5_fu_161_p2 = ($signed(i_6_cast24_fu_142_p1) + $signed(6'd45));

assign add_ln88_6_fu_210_p2 = ($signed(zext_ln88_12_fu_206_p1) + $signed(6'd44));

assign add_ln88_7_fu_225_p2 = ($signed(zext_ln88_12_fu_206_p1) + $signed(6'd45));

assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

assign ap_CS_fsm_pp0_stage1 = ap_CS_fsm[32'd1];

assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage1 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage1_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage1_subdone = ~(1'b1 == 1'b1);

assign ap_block_state1_pp0_stage0_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state2_pp0_stage1_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state3_pp0_stage0_iter1 = ~(1'b1 == 1'b1);

assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage0;

assign g_col_h1_d0 = g_1_q0;

assign g_col_h1_d1 = g_1_q1;

assign i_6_cast24_fu_142_p1 = ap_sig_allocacmp_i;

assign i_6_cast9_fu_187_p1 = i_reg_261;

assign icmp_ln86_fu_136_p2 = ((ap_sig_allocacmp_i == 4'd12) ? 1'b1 : 1'b0);

assign or_ln86_3_fu_191_p2 = (i_reg_261 | 4'd1);

assign or_ln86_4_fu_201_p2 = (i_reg_261 | 4'd2);

assign or_ln86_5_fu_244_p2 = (i_reg_261 | 4'd3);

assign sext_ln88_7_fu_167_p1 = $signed(add_ln88_5_fu_161_p2);

assign sext_ln88_8_fu_216_p1 = $signed(add_ln88_6_fu_210_p2);

assign sext_ln88_9_fu_231_p1 = $signed(add_ln88_7_fu_225_p2);

assign sext_ln88_fu_152_p1 = $signed(add_ln88_4_fu_146_p2);

assign zext_ln88_11_fu_240_p1 = or_ln86_4_reg_283;

assign zext_ln88_12_fu_206_p1 = or_ln86_4_fu_201_p2;

assign zext_ln88_14_fu_249_p1 = or_ln86_5_fu_244_p2;

assign zext_ln88_29_fu_171_p1 = $unsigned(sext_ln88_7_fu_167_p1);

assign zext_ln88_30_fu_220_p1 = $unsigned(sext_ln88_8_fu_216_p1);

assign zext_ln88_31_fu_235_p1 = $unsigned(sext_ln88_9_fu_231_p1);

assign zext_ln88_9_fu_196_p1 = or_ln86_3_fu_191_p2;

assign zext_ln88_fu_156_p1 = $unsigned(sext_ln88_fu_152_p1);

always @ (posedge ap_clk) begin
    or_ln86_4_reg_283[1] <= 1'b1;
end

endmodule //tracking_update_linear_cost_4_Pipeline_VITIS_LOOP_86_14
