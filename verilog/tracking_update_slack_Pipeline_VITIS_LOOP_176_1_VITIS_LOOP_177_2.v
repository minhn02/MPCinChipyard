// ==============================================================
// RTL generated by Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.1 (64-bit)
// Version: 2022.1
// Copyright (C) Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

`timescale 1 ns / 1 ps 

module tracking_update_slack_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2 (
        ap_clk,
        ap_rst,
        ap_start,
        ap_done,
        ap_idle,
        ap_ready,
        x_1_address0,
        x_1_ce0,
        x_1_q0,
        g_1_address0,
        g_1_ce0,
        g_1_q0,
        vnew_1_address0,
        vnew_1_ce0,
        vnew_1_we0,
        vnew_1_d0,
        grp_fu_261_p_din0,
        grp_fu_261_p_din1,
        grp_fu_261_p_opcode,
        grp_fu_261_p_dout0,
        grp_fu_261_p_ce
);

parameter    ap_ST_fsm_pp0_stage0 = 1'd1;

input   ap_clk;
input   ap_rst;
input   ap_start;
output   ap_done;
output   ap_idle;
output   ap_ready;
output  [6:0] x_1_address0;
output   x_1_ce0;
input  [31:0] x_1_q0;
output  [6:0] g_1_address0;
output   g_1_ce0;
input  [31:0] g_1_q0;
output  [6:0] vnew_1_address0;
output   vnew_1_ce0;
output   vnew_1_we0;
output  [31:0] vnew_1_d0;
output  [31:0] grp_fu_261_p_din0;
output  [31:0] grp_fu_261_p_din1;
output  [1:0] grp_fu_261_p_opcode;
input  [31:0] grp_fu_261_p_dout0;
output   grp_fu_261_p_ce;

reg ap_idle;
reg x_1_ce0;
reg g_1_ce0;
reg vnew_1_ce0;
reg vnew_1_we0;

(* fsm_encoding = "none" *) reg   [0:0] ap_CS_fsm;
wire    ap_CS_fsm_pp0_stage0;
wire    ap_enable_reg_pp0_iter0;
reg    ap_enable_reg_pp0_iter1;
reg    ap_enable_reg_pp0_iter2;
reg    ap_idle_pp0;
wire    ap_block_state1_pp0_stage0_iter0;
wire    ap_block_state2_pp0_stage0_iter1;
wire    ap_block_state3_pp0_stage0_iter2;
wire    ap_block_pp0_stage0_subdone;
wire   [0:0] icmp_ln176_fu_155_p2;
reg    ap_condition_exit_pp0_iter0_stage0;
wire    ap_loop_exit_ready;
reg    ap_ready_int;
wire    ap_block_pp0_stage0_11001;
wire   [63:0] zext_ln178_fu_246_p1;
reg   [63:0] zext_ln178_reg_298;
reg   [63:0] zext_ln178_reg_298_pp0_iter1_reg;
wire    ap_block_pp0_stage0;
reg   [3:0] j_fu_46;
wire   [3:0] add_ln177_fu_252_p2;
wire    ap_loop_init;
reg   [3:0] ap_sig_allocacmp_j_load;
reg   [3:0] i_fu_50;
wire   [3:0] select_ln176_2_fu_228_p3;
reg   [3:0] ap_sig_allocacmp_i_2;
reg   [6:0] indvar_flatten_fu_54;
wire   [6:0] add_ln176_fu_161_p2;
reg   [6:0] ap_sig_allocacmp_indvar_flatten_load;
wire   [2:0] empty_fu_125_p1;
wire   [5:0] p_shl1_fu_137_p3;
wire   [6:0] p_shl_fu_129_p3;
wire   [6:0] p_shl1_cast_fu_145_p1;
wire   [0:0] icmp_ln177_fu_176_p2;
wire   [3:0] add_ln176_1_fu_170_p2;
wire   [2:0] empty_44_fu_190_p1;
wire   [5:0] p_shl1_mid1_fu_202_p3;
wire   [6:0] p_shl_mid1_fu_194_p3;
wire   [6:0] p_shl1_cast_mid1_fu_210_p1;
wire   [6:0] p_mid1_fu_214_p2;
wire   [6:0] empty_42_fu_149_p2;
wire   [3:0] select_ln176_fu_182_p3;
wire   [6:0] j_3_cast_fu_236_p1;
wire   [6:0] select_ln176_1_fu_220_p3;
wire   [6:0] add_ln178_fu_240_p2;
reg    ap_done_reg;
wire    ap_continue_int;
reg    ap_done_int;
reg    ap_loop_exit_ready_pp0_iter1_reg;
reg   [0:0] ap_NS_fsm;
wire    ap_enable_pp0;
wire    ap_start_int;
wire    ap_block_pp0_stage0_00001;
wire    ap_ce_reg;

// power-on initialization
initial begin
#0 ap_CS_fsm = 1'd1;
#0 ap_enable_reg_pp0_iter1 = 1'b0;
#0 ap_enable_reg_pp0_iter2 = 1'b0;
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
        end else if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_loop_exit_ready_pp0_iter1_reg == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
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
    if (ap_rst == 1'b1) begin
        ap_enable_reg_pp0_iter2 <= 1'b0;
    end else begin
        if ((1'b0 == ap_block_pp0_stage0_subdone)) begin
            ap_enable_reg_pp0_iter2 <= ap_enable_reg_pp0_iter1;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln176_fu_155_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            i_fu_50 <= select_ln176_2_fu_228_p3;
        end else if ((ap_loop_init == 1'b1)) begin
            i_fu_50 <= 4'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln176_fu_155_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            indvar_flatten_fu_54 <= add_ln176_fu_161_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            indvar_flatten_fu_54 <= 7'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln176_fu_155_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            j_fu_46 <= add_ln177_fu_252_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            j_fu_46 <= 4'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_loop_exit_ready_pp0_iter1_reg <= ap_loop_exit_ready;
        zext_ln178_reg_298_pp0_iter1_reg[6 : 0] <= zext_ln178_reg_298[6 : 0];
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln176_fu_155_p2 == 1'd0) & (1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        zext_ln178_reg_298[6 : 0] <= zext_ln178_fu_246_p1[6 : 0];
    end
end

always @ (*) begin
    if (((icmp_ln176_fu_155_p2 == 1'd1) & (1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_condition_exit_pp0_iter0_stage0 = 1'b1;
    end else begin
        ap_condition_exit_pp0_iter0_stage0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_subdone) & (ap_loop_exit_ready_pp0_iter1_reg == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
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
    if (((ap_enable_reg_pp0_iter2 == 1'b0) & (ap_enable_reg_pp0_iter1 == 1'b0) & (ap_enable_reg_pp0_iter0 == 1'b0))) begin
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
        ap_sig_allocacmp_i_2 = 4'd0;
    end else begin
        ap_sig_allocacmp_i_2 = i_fu_50;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_indvar_flatten_load = 7'd0;
    end else begin
        ap_sig_allocacmp_indvar_flatten_load = indvar_flatten_fu_54;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_j_load = 4'd0;
    end else begin
        ap_sig_allocacmp_j_load = j_fu_46;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        g_1_ce0 = 1'b1;
    end else begin
        g_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter2 == 1'b1))) begin
        vnew_1_ce0 = 1'b1;
    end else begin
        vnew_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter2 == 1'b1))) begin
        vnew_1_we0 = 1'b1;
    end else begin
        vnew_1_we0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        x_1_ce0 = 1'b1;
    end else begin
        x_1_ce0 = 1'b0;
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

assign add_ln176_1_fu_170_p2 = (ap_sig_allocacmp_i_2 + 4'd1);

assign add_ln176_fu_161_p2 = (ap_sig_allocacmp_indvar_flatten_load + 7'd1);

assign add_ln177_fu_252_p2 = (select_ln176_fu_182_p3 + 4'd1);

assign add_ln178_fu_240_p2 = (j_3_cast_fu_236_p1 + select_ln176_1_fu_220_p3);

assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_00001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

assign ap_block_state1_pp0_stage0_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state2_pp0_stage0_iter1 = ~(1'b1 == 1'b1);

assign ap_block_state3_pp0_stage0_iter2 = ~(1'b1 == 1'b1);

assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

assign ap_enable_reg_pp0_iter0 = ap_start_int;

assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage0;

assign empty_42_fu_149_p2 = (p_shl_fu_129_p3 - p_shl1_cast_fu_145_p1);

assign empty_44_fu_190_p1 = add_ln176_1_fu_170_p2[2:0];

assign empty_fu_125_p1 = ap_sig_allocacmp_i_2[2:0];

assign g_1_address0 = zext_ln178_fu_246_p1;

assign grp_fu_261_p_ce = 1'b1;

assign grp_fu_261_p_din0 = x_1_q0;

assign grp_fu_261_p_din1 = g_1_q0;

assign grp_fu_261_p_opcode = 2'd0;

assign icmp_ln176_fu_155_p2 = ((ap_sig_allocacmp_indvar_flatten_load == 7'd120) ? 1'b1 : 1'b0);

assign icmp_ln177_fu_176_p2 = ((ap_sig_allocacmp_j_load == 4'd12) ? 1'b1 : 1'b0);

assign j_3_cast_fu_236_p1 = select_ln176_fu_182_p3;

assign p_mid1_fu_214_p2 = (p_shl_mid1_fu_194_p3 - p_shl1_cast_mid1_fu_210_p1);

assign p_shl1_cast_fu_145_p1 = p_shl1_fu_137_p3;

assign p_shl1_cast_mid1_fu_210_p1 = p_shl1_mid1_fu_202_p3;

assign p_shl1_fu_137_p3 = {{ap_sig_allocacmp_i_2}, {2'd0}};

assign p_shl1_mid1_fu_202_p3 = {{add_ln176_1_fu_170_p2}, {2'd0}};

assign p_shl_fu_129_p3 = {{empty_fu_125_p1}, {4'd0}};

assign p_shl_mid1_fu_194_p3 = {{empty_44_fu_190_p1}, {4'd0}};

assign select_ln176_1_fu_220_p3 = ((icmp_ln177_fu_176_p2[0:0] == 1'b1) ? p_mid1_fu_214_p2 : empty_42_fu_149_p2);

assign select_ln176_2_fu_228_p3 = ((icmp_ln177_fu_176_p2[0:0] == 1'b1) ? add_ln176_1_fu_170_p2 : ap_sig_allocacmp_i_2);

assign select_ln176_fu_182_p3 = ((icmp_ln177_fu_176_p2[0:0] == 1'b1) ? 4'd0 : ap_sig_allocacmp_j_load);

assign vnew_1_address0 = zext_ln178_reg_298_pp0_iter1_reg;

assign vnew_1_d0 = grp_fu_261_p_dout0;

assign x_1_address0 = zext_ln178_fu_246_p1;

assign zext_ln178_fu_246_p1 = add_ln178_fu_240_p2;

always @ (posedge ap_clk) begin
    zext_ln178_reg_298[63:7] <= 57'b000000000000000000000000000000000000000000000000000000000;
    zext_ln178_reg_298_pp0_iter1_reg[63:7] <= 57'b000000000000000000000000000000000000000000000000000000000;
end

endmodule //tracking_update_slack_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2
