// ==============================================================
// RTL generated by Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.1 (64-bit)
// Version: 2022.1
// Copyright (C) Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

`timescale 1 ns / 1 ps 

module tracking_tracking_Pipeline_VITIS_LOOP_210_1_VITIS_LOOP_211_2 (
        ap_clk,
        ap_rst,
        ap_start,
        ap_done,
        ap_idle,
        ap_ready,
        sub_ln71,
        Xref_1_address0,
        Xref_1_ce0,
        Xref_1_we0,
        Xref_1_d0
);

parameter    ap_ST_fsm_pp0_stage0 = 1'd1;

input   ap_clk;
input   ap_rst;
input   ap_start;
output   ap_done;
output   ap_idle;
output   ap_ready;
input  [11:0] sub_ln71;
output  [6:0] Xref_1_address0;
output   Xref_1_ce0;
output   Xref_1_we0;
output  [31:0] Xref_1_d0;

reg ap_idle;
reg Xref_1_ce0;
reg Xref_1_we0;

(* fsm_encoding = "none" *) reg   [0:0] ap_CS_fsm;
wire    ap_CS_fsm_pp0_stage0;
wire    ap_enable_reg_pp0_iter0;
reg    ap_enable_reg_pp0_iter1;
reg    ap_idle_pp0;
wire    ap_block_state1_pp0_stage0_iter0;
wire    ap_block_state2_pp0_stage0_iter1;
wire    ap_block_pp0_stage0_subdone;
wire   [0:0] icmp_ln210_fu_164_p2;
reg    ap_condition_exit_pp0_iter0_stage0;
wire    ap_loop_exit_ready;
reg    ap_ready_int;
wire   [11:0] Xref_data_address0;
reg    Xref_data_ce0;
wire   [31:0] Xref_data_q0;
wire    ap_block_pp0_stage0_11001;
wire   [6:0] add_ln212_1_fu_292_p2;
reg   [6:0] add_ln212_1_reg_382;
wire   [63:0] zext_ln212_fu_287_p1;
wire    ap_block_pp0_stage0;
wire   [63:0] zext_ln212_1_fu_335_p1;
reg   [3:0] j_fu_50;
wire   [3:0] add_ln211_fu_298_p2;
wire    ap_loop_init;
reg   [3:0] ap_sig_allocacmp_j_load;
reg   [6:0] k_1_fu_54;
wire   [6:0] add_ln211_1_fu_304_p2;
reg   [6:0] ap_sig_allocacmp_k_1_load;
reg   [6:0] k_fu_58;
wire   [6:0] select_ln210_4_fu_265_p3;
reg   [6:0] ap_sig_allocacmp_k_load;
reg   [3:0] i_fu_62;
wire   [3:0] select_ln210_3_fu_257_p3;
reg   [3:0] ap_sig_allocacmp_i_24;
reg   [6:0] indvar_flatten31_fu_66;
wire   [6:0] add_ln210_2_fu_170_p2;
reg   [6:0] ap_sig_allocacmp_indvar_flatten31_load;
wire   [2:0] empty_fu_134_p1;
wire   [5:0] p_shl5_fu_146_p3;
wire   [6:0] p_shl4_fu_138_p3;
wire   [6:0] p_shl15_cast_fu_154_p1;
wire   [0:0] icmp_ln211_fu_197_p2;
wire   [6:0] add_ln210_1_fu_191_p2;
wire   [3:0] add_ln210_fu_185_p2;
wire   [2:0] empty_78_fu_219_p1;
wire   [5:0] p_shl15_mid1_fu_231_p3;
wire   [6:0] p_shl14_mid1_fu_223_p3;
wire   [6:0] p_shl15_cast_mid1_fu_239_p1;
wire   [6:0] p_mid129_fu_243_p2;
wire   [6:0] empty_76_fu_158_p2;
wire   [3:0] select_ln210_1_fu_211_p3;
wire   [6:0] select_ln210_fu_203_p3;
wire   [11:0] k_1_cast_fu_277_p1;
wire   [11:0] add_ln212_fu_281_p2;
wire   [6:0] j_20_cast_fu_273_p1;
wire   [6:0] select_ln210_2_fu_249_p3;
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

tracking_tracking_Pipeline_VITIS_LOOP_210_1_VITIS_LOOP_211_2_Xref_data_ROM_AUTO_1R #(
    .DataWidth( 32 ),
    .AddressRange( 3612 ),
    .AddressWidth( 12 ))
Xref_data_U(
    .clk(ap_clk),
    .reset(ap_rst),
    .address0(Xref_data_address0),
    .ce0(Xref_data_ce0),
    .q0(Xref_data_q0)
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
        if (((icmp_ln210_fu_164_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            i_fu_62 <= select_ln210_3_fu_257_p3;
        end else if ((ap_loop_init == 1'b1)) begin
            i_fu_62 <= 4'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln210_fu_164_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            indvar_flatten31_fu_66 <= add_ln210_2_fu_170_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            indvar_flatten31_fu_66 <= 7'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln210_fu_164_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            j_fu_50 <= add_ln211_fu_298_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            j_fu_50 <= 4'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln210_fu_164_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            k_1_fu_54 <= add_ln211_1_fu_304_p2;
        end else if ((ap_loop_init == 1'b1)) begin
            k_1_fu_54 <= 7'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        if (((icmp_ln210_fu_164_p2 == 1'd0) & (ap_enable_reg_pp0_iter0 == 1'b1))) begin
            k_fu_58 <= select_ln210_4_fu_265_p3;
        end else if ((ap_loop_init == 1'b1)) begin
            k_fu_58 <= 7'd0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln210_fu_164_p2 == 1'd0) & (1'b0 == ap_block_pp0_stage0_11001) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        add_ln212_1_reg_382 <= add_ln212_1_fu_292_p2;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        Xref_1_ce0 = 1'b1;
    end else begin
        Xref_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter1 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        Xref_1_we0 = 1'b1;
    end else begin
        Xref_1_we0 = 1'b0;
    end
end

always @ (*) begin
    if (((1'b0 == ap_block_pp0_stage0_11001) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        Xref_data_ce0 = 1'b1;
    end else begin
        Xref_data_ce0 = 1'b0;
    end
end

always @ (*) begin
    if (((icmp_ln210_fu_164_p2 == 1'd1) & (1'b0 == ap_block_pp0_stage0_subdone) & (ap_enable_reg_pp0_iter0 == 1'b1) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
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
        ap_sig_allocacmp_i_24 = 4'd0;
    end else begin
        ap_sig_allocacmp_i_24 = i_fu_62;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_indvar_flatten31_load = 7'd0;
    end else begin
        ap_sig_allocacmp_indvar_flatten31_load = indvar_flatten31_fu_66;
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
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_k_1_load = 7'd0;
    end else begin
        ap_sig_allocacmp_k_1_load = k_1_fu_54;
    end
end

always @ (*) begin
    if (((ap_loop_init == 1'b1) & (1'b0 == ap_block_pp0_stage0) & (1'b1 == ap_CS_fsm_pp0_stage0))) begin
        ap_sig_allocacmp_k_load = 7'd0;
    end else begin
        ap_sig_allocacmp_k_load = k_fu_58;
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

assign Xref_1_address0 = zext_ln212_1_fu_335_p1;

assign Xref_1_d0 = Xref_data_q0;

assign Xref_data_address0 = zext_ln212_fu_287_p1;

assign add_ln210_1_fu_191_p2 = (ap_sig_allocacmp_k_load + 7'd12);

assign add_ln210_2_fu_170_p2 = (ap_sig_allocacmp_indvar_flatten31_load + 7'd1);

assign add_ln210_fu_185_p2 = (ap_sig_allocacmp_i_24 + 4'd1);

assign add_ln211_1_fu_304_p2 = (select_ln210_fu_203_p3 + 7'd1);

assign add_ln211_fu_298_p2 = (select_ln210_1_fu_211_p3 + 4'd1);

assign add_ln212_1_fu_292_p2 = (j_20_cast_fu_273_p1 + select_ln210_2_fu_249_p3);

assign add_ln212_fu_281_p2 = (k_1_cast_fu_277_p1 + sub_ln71);

assign ap_CS_fsm_pp0_stage0 = ap_CS_fsm[32'd0];

assign ap_block_pp0_stage0 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_11001 = ~(1'b1 == 1'b1);

assign ap_block_pp0_stage0_subdone = ~(1'b1 == 1'b1);

assign ap_block_state1_pp0_stage0_iter0 = ~(1'b1 == 1'b1);

assign ap_block_state2_pp0_stage0_iter1 = ~(1'b1 == 1'b1);

assign ap_enable_pp0 = (ap_idle_pp0 ^ 1'b1);

assign ap_enable_reg_pp0_iter0 = ap_start_int;

assign ap_loop_exit_ready = ap_condition_exit_pp0_iter0_stage0;

assign empty_76_fu_158_p2 = (p_shl4_fu_138_p3 - p_shl15_cast_fu_154_p1);

assign empty_78_fu_219_p1 = add_ln210_fu_185_p2[2:0];

assign empty_fu_134_p1 = ap_sig_allocacmp_i_24[2:0];

assign icmp_ln210_fu_164_p2 = ((ap_sig_allocacmp_indvar_flatten31_load == 7'd120) ? 1'b1 : 1'b0);

assign icmp_ln211_fu_197_p2 = ((ap_sig_allocacmp_j_load == 4'd12) ? 1'b1 : 1'b0);

assign j_20_cast_fu_273_p1 = select_ln210_1_fu_211_p3;

assign k_1_cast_fu_277_p1 = select_ln210_fu_203_p3;

assign p_mid129_fu_243_p2 = (p_shl14_mid1_fu_223_p3 - p_shl15_cast_mid1_fu_239_p1);

assign p_shl14_mid1_fu_223_p3 = {{empty_78_fu_219_p1}, {4'd0}};

assign p_shl15_cast_fu_154_p1 = p_shl5_fu_146_p3;

assign p_shl15_cast_mid1_fu_239_p1 = p_shl15_mid1_fu_231_p3;

assign p_shl15_mid1_fu_231_p3 = {{add_ln210_fu_185_p2}, {2'd0}};

assign p_shl4_fu_138_p3 = {{empty_fu_134_p1}, {4'd0}};

assign p_shl5_fu_146_p3 = {{ap_sig_allocacmp_i_24}, {2'd0}};

assign select_ln210_1_fu_211_p3 = ((icmp_ln211_fu_197_p2[0:0] == 1'b1) ? 4'd0 : ap_sig_allocacmp_j_load);

assign select_ln210_2_fu_249_p3 = ((icmp_ln211_fu_197_p2[0:0] == 1'b1) ? p_mid129_fu_243_p2 : empty_76_fu_158_p2);

assign select_ln210_3_fu_257_p3 = ((icmp_ln211_fu_197_p2[0:0] == 1'b1) ? add_ln210_fu_185_p2 : ap_sig_allocacmp_i_24);

assign select_ln210_4_fu_265_p3 = ((icmp_ln211_fu_197_p2[0:0] == 1'b1) ? add_ln210_1_fu_191_p2 : ap_sig_allocacmp_k_load);

assign select_ln210_fu_203_p3 = ((icmp_ln211_fu_197_p2[0:0] == 1'b1) ? add_ln210_1_fu_191_p2 : ap_sig_allocacmp_k_1_load);

assign zext_ln212_1_fu_335_p1 = add_ln212_1_reg_382;

assign zext_ln212_fu_287_p1 = add_ln212_fu_281_p2;

endmodule //tracking_tracking_Pipeline_VITIS_LOOP_210_1_VITIS_LOOP_211_2
