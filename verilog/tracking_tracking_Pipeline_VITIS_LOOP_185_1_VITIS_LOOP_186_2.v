// ==============================================================
// RTL generated by Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.1 (64-bit)
// Version: 2022.1
// Copyright (C) Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

`timescale 1 ns / 1 ps 

module tracking_tracking_Pipeline_VITIS_LOOP_185_1_VITIS_LOOP_186_2 (
        ap_clk,
        ap_rst,
        ap_start,
        ap_done,
        ap_idle,
        ap_ready,
        Kinf_1_address0,
        Kinf_1_ce0,
        Kinf_1_q0,
        KinfT_1_address0,
        KinfT_1_ce0,
        KinfT_1_we0,
        KinfT_1_d0
);

parameter    ap_ST_fsm_pp0_stage0 = 1'd1;

input   ap_clk;
input   ap_rst;
input   ap_start;
output   ap_done;
output   ap_idle;
output   ap_ready;
output  [5:0] Kinf_1_address0;
output   Kinf_1_ce0;
input  [31:0] Kinf_1_q0;
output  [5:0] KinfT_1_address0;
output   KinfT_1_ce0;
output   KinfT_1_we0;
output  [31:0] KinfT_1_d0;

reg ap_idle;
reg Kinf_1_ce0;
reg KinfT_1_ce0;
reg KinfT_1_we0;

(* fsm_encoding = "none" *) reg   [0:0] ap_CS_fsm;
wire    ap_CS_fsm_pp0_stage0;
wire    ap_enable_reg_pp0_iter0;
reg    ap_enable_reg_pp0_iter1;
reg    ap_idle_pp0;
wire    ap_block_state1_pp0_stage0_iter0;
wire    ap_block_state2_pp0_stage0_iter1;
wire    ap_block_pp0_stage0_subdone;
wire   [0:0] icmp_ln185_fu_140_p2;
reg    ap_condition_exit_pp0_iter0_stage0;
wire    ap_loop_exit_ready;
reg    ap_ready_int;
wire    ap_block_pp0_stage0_11001;
wire   [5:0] add_ln187_1_fu_248_p2;
reg   [5:0] add_ln187_1_reg_308;
wire   [63:0] zext_ln187_fu_235_p1;
wire    ap_block_pp0_stage0;
wire   [63:0] zext_ln187_1_fu_275_p1;
reg   [3:0] j_fu_50;
wire   [3:0] add_ln186_fu_254_p2;
wire    ap_loop_init;
reg   [3:0] ap_sig_allocacmp_j_load;
reg   [2:0] i_fu_54;
wire   [2:0] select_ln185_1_fu_175_p3;
reg   [2:0] ap_sig_allocacmp_i_25;
reg   [5:0] indvar_flatten_fu_58;
wire   [5:0] add_ln185_3_fu_146_p2;
reg   [5:0] ap_sig_allocacmp_indvar_flatten_load;
wire   [1:0] empty_fu_110_p1;
wire   [3:0] p_shl1_fu_122_p3;
wire   [5:0] p_shl_fu_114_p3;
wire   [5:0] p_shl1_cast_fu_130_p1;
wire   [0:0] icmp_ln186_fu_161_p2;
wire   [2:0] add_ln185_fu_155_p2;
wire   [1:0] empty_82_fu_187_p1;
wire   [3:0] p_shl1_mid1_fu_199_p3;
wire   [5:0] p_shl_mid1_fu_191_p3;
wire   [5:0] p_shl1_cast_mid1_fu_207_p1;
wire   [5:0] p_mid1_fu_211_p2;
wire   [5:0] empty_80_fu_134_p2;
wire   [3:0] select_ln185_fu_167_p3;
wire   [5:0] j_cast_fu_225_p1;
wire   [5:0] select_ln185_2_fu_217_p3;
wire   [5:0] add_ln187_fu_229_p2;
wire   [5:0] shl_ln_fu_240_p3;
wire   [5:0] zext_ln185_fu_183_p1;
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
        if (((icmp_ln185_fu_140_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            i_fu_54 <= select_ln185_1_fu_175_p3;
        end else if ((ap_loop_init == 1'b1)) begin
            i_fu_54 <= 3'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln185_fu_140_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            indvar_flatten_fu_58 <= add_ln185_3_fu_146_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            indvar_flatten_fu_58 <= 6'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln185_fu_140_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            j_fu_50 <= add_ln186_fu_254_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            j_fu_50 <= 4'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln185_fu_140_p2 == 1'd0) & (1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        add_ln187_1_reg_308 <= add_ln187_1_fu_248_p2;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        KinfT_1_ce0 = 1'b1;
    end else begin
        KinfT_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        KinfT_1_we0 = 1'b1;
    end else begin
        KinfT_1_we0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        Kinf_1_ce0 = 1'b1;
    end else begin
        Kinf_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if (((icmp_ln185_fu_140_p2 == 1'd1) & (1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
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
        ap_sig_allocacmp_i_25 = 3'd0;
    end else begin
        ap_sig_allocacmp_i_25 = i_fu_54;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_indvar_flatten_load = 6'd0;
    end else begin
        ap_sig_allocacmp_indvar_flatten_load = indvar_flatten_fu_58;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_j_load = 4'd0;
    end else begin
        ap_sig_allocacmp_j_load = j_fu_50;
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

assign KinfT_1_address0 = zext_ln187_1_fu_275_p1;

assign KinfT_1_d0 = Kinf_1_q0;

assign Kinf_1_address0 = zext_ln187_fu_235_p1;

assign add_ln185_3_fu_146_p2 = (ap_sig_allocacmp_indvar_flatten_load + 6'd1);

assign add_ln185_fu_155_p2 = (ap_sig_allocacmp_i_25 + 3'd1);

assign add_ln186_fu_254_p2 = (select_ln185_fu_167_p3 + 4'd1);

assign add_ln187_1_fu_248_p2 = (shl_ln_fu_240_p3 + zext_ln185_fu_183_p1);

assign add_ln187_fu_229_p2 = (j_cast_fu_225_p1 + select_ln185_2_fu_217_p3);

assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

assign ap_block_state1_pp0_stage0_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state2_pp0_stage0_iter1 = ~(1'b1 == 1'b1);

assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

assign ap_enable_reg_pp0_iter0 = ap_start_int;

assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage0;

assign empty_80_fu_134_p2 = (p_shl_fu_114_p3 - p_shl1_cast_fu_130_p1);

assign empty_82_fu_187_p1 = add_ln185_fu_155_p2[1:0];

assign empty_fu_110_p1 = ap_sig_allocacmp_i_25[1:0];

assign icmp_ln185_fu_140_p2 = ((ap_sig_allocacmp_indvar_flatten_load == 6'd48) ? 1'b1 : 1'b0);

assign icmp_ln186_fu_161_p2 = ((ap_sig_allocacmp_j_load == 4'd12) ? 1'b1 : 1'b0);

assign j_cast_fu_225_p1 = select_ln185_fu_167_p3;

assign p_mid1_fu_211_p2 = (p_shl_mid1_fu_191_p3 - p_shl1_cast_mid1_fu_207_p1);

assign p_shl1_cast_fu_130_p1 = p_shl1_fu_122_p3;

assign p_shl1_cast_mid1_fu_207_p1 = p_shl1_mid1_fu_199_p3;

assign p_shl1_fu_122_p3 = {{empty_fu_110_p1}, {2'd0}};

assign p_shl1_mid1_fu_199_p3 = {{empty_82_fu_187_p1}, {2'd0}};

assign p_shl_fu_114_p3 = {{empty_fu_110_p1}, {4'd0}};

assign p_shl_mid1_fu_191_p3 = {{empty_82_fu_187_p1}, {4'd0}};

assign select_ln185_1_fu_175_p3 = ((icmp_ln186_fu_161_p2[0:0] == 1'b1) ? add_ln185_fu_155_p2 : ap_sig_allocacmp_i_25);

assign select_ln185_2_fu_217_p3 = ((icmp_ln186_fu_161_p2[0:0] == 1'b1) ? p_mid1_fu_211_p2 : empty_80_fu_134_p2);

assign select_ln185_fu_167_p3 = ((icmp_ln186_fu_161_p2[0:0] == 1'b1) ? 4'd0 : ap_sig_allocacmp_j_load);

assign shl_ln_fu_240_p3 = {{select_ln185_fu_167_p3}, {2'd0}};

assign zext_ln185_fu_183_p1 = select_ln185_1_fu_175_p3;

assign zext_ln187_1_fu_275_p1 = add_ln187_1_reg_308;

assign zext_ln187_fu_235_p1 = add_ln187_fu_229_p2;

endmodule //tracking_tracking_Pipeline_VITIS_LOOP_185_1_VITIS_LOOP_186_2
