// ==============================================================
// RTL generated by Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2022.1 (64-bit)
// Version: 2022.1
// Copyright (C) Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

`timescale 1 ns / 1 ps 

module tracking_update_dual (
        ap_clk,
        ap_rst,
        ap_start,
        ap_done,
        ap_idle,
        ap_ready,
        y_1_address0,
        y_1_ce0,
        y_1_we0,
        y_1_d0,
        y_1_q0,
        u_1_address0,
        u_1_ce0,
        u_1_q0,
        m1_1_address0,
        m1_1_ce0,
        m1_1_we0,
        m1_1_d0,
        m1_1_q0,
        g_1_address0,
        g_1_ce0,
        g_1_we0,
        g_1_d0,
        g_1_q0,
        x_1_address0,
        x_1_ce0,
        x_1_q0,
        s1_1_address0,
        s1_1_ce0,
        s1_1_we0,
        s1_1_d0,
        s1_1_q0,
        vnew_1_address0,
        vnew_1_ce0,
        vnew_1_q0,
        znew_1_address0,
        znew_1_ce0,
        znew_1_q0
);

parameter    ap_ST_fsm_state1 = 12'd1;
parameter    ap_ST_fsm_state2 = 12'd2;
parameter    ap_ST_fsm_state3 = 12'd4;
parameter    ap_ST_fsm_state4 = 12'd8;
parameter    ap_ST_fsm_state5 = 12'd16;
parameter    ap_ST_fsm_state6 = 12'd32;
parameter    ap_ST_fsm_state7 = 12'd64;
parameter    ap_ST_fsm_state8 = 12'd128;
parameter    ap_ST_fsm_state9 = 12'd256;
parameter    ap_ST_fsm_state10 = 12'd512;
parameter    ap_ST_fsm_state11 = 12'd1024;
parameter    ap_ST_fsm_state12 = 12'd2048;

input   ap_clk;
input   ap_rst;
input   ap_start;
output   ap_done;
output   ap_idle;
output   ap_ready;
output  [5:0] y_1_address0;
output   y_1_ce0;
output   y_1_we0;
output  [31:0] y_1_d0;
input  [31:0] y_1_q0;
output  [5:0] u_1_address0;
output   u_1_ce0;
input  [31:0] u_1_q0;
output  [5:0] m1_1_address0;
output   m1_1_ce0;
output   m1_1_we0;
output  [31:0] m1_1_d0;
input  [31:0] m1_1_q0;
output  [6:0] g_1_address0;
output   g_1_ce0;
output   g_1_we0;
output  [31:0] g_1_d0;
input  [31:0] g_1_q0;
output  [6:0] x_1_address0;
output   x_1_ce0;
input  [31:0] x_1_q0;
output  [6:0] s1_1_address0;
output   s1_1_ce0;
output   s1_1_we0;
output  [31:0] s1_1_d0;
input  [31:0] s1_1_q0;
output  [6:0] vnew_1_address0;
output   vnew_1_ce0;
input  [31:0] vnew_1_q0;
output  [5:0] znew_1_address0;
output   znew_1_ce0;
input  [31:0] znew_1_q0;

reg ap_done;
reg ap_idle;
reg ap_ready;
reg[5:0] y_1_address0;
reg y_1_ce0;
reg y_1_we0;
reg u_1_ce0;
reg[5:0] m1_1_address0;
reg m1_1_ce0;
reg m1_1_we0;
reg[6:0] g_1_address0;
reg g_1_ce0;
reg g_1_we0;
reg[6:0] s1_1_address0;
reg s1_1_ce0;
reg s1_1_we0;
reg znew_1_ce0;

(* fsm_encoding = "none" *) reg   [11:0] ap_CS_fsm;
wire    ap_CS_fsm_state1;
wire   [3:0] add_ln176_fu_198_p2;
reg   [3:0] add_ln176_reg_312;
wire    ap_CS_fsm_state2;
wire   [5:0] tmp_fu_204_p3;
reg   [5:0] tmp_reg_317;
wire   [0:0] icmp_ln176_fu_192_p2;
wire   [2:0] add_ln177_fu_227_p2;
reg   [2:0] add_ln177_reg_332;
wire    ap_CS_fsm_state3;
wire   [63:0] zext_ln178_fu_238_p1;
reg   [63:0] zext_ln178_reg_337;
wire   [0:0] icmp_ln177_fu_221_p2;
wire    ap_CS_fsm_state4;
wire   [3:0] add_ln167_fu_257_p2;
reg   [3:0] add_ln167_reg_365;
wire    ap_CS_fsm_state6;
wire   [5:0] tmp_s_fu_263_p3;
reg   [5:0] tmp_s_reg_370;
wire   [0:0] icmp_ln167_fu_251_p2;
wire   [2:0] add_ln168_fu_281_p2;
reg   [2:0] add_ln168_reg_378;
wire    ap_CS_fsm_state7;
wire   [63:0] zext_ln169_fu_292_p1;
reg   [63:0] zext_ln169_reg_383;
wire   [0:0] icmp_ln168_fu_275_p2;
wire    ap_CS_fsm_state8;
wire    grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_start;
wire    grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_done;
wire    grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_idle;
wire    grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_ready;
wire   [6:0] grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_g_1_address0;
wire    grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_g_1_ce0;
wire   [6:0] grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_x_1_address0;
wire    grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_x_1_ce0;
wire   [6:0] grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_s1_1_address0;
wire    grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_s1_1_ce0;
wire    grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_s1_1_we0;
wire   [31:0] grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_s1_1_d0;
wire   [31:0] grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_grp_fu_174_p_din0;
wire   [31:0] grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_grp_fu_174_p_din1;
wire   [0:0] grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_grp_fu_174_p_opcode;
wire    grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_grp_fu_174_p_ce;
wire    grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_start;
wire    grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_done;
wire    grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_idle;
wire    grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_ready;
wire   [6:0] grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_s1_1_address0;
wire    grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_s1_1_ce0;
wire   [6:0] grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_vnew_1_address0;
wire    grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_vnew_1_ce0;
wire   [6:0] grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_g_1_address0;
wire    grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_g_1_ce0;
wire    grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_g_1_we0;
wire   [31:0] grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_g_1_d0;
wire   [31:0] grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_grp_fu_174_p_din0;
wire   [31:0] grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_grp_fu_174_p_din1;
wire   [0:0] grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_grp_fu_174_p_opcode;
wire    grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_grp_fu_174_p_ce;
reg   [2:0] j_reg_132;
wire    ap_CS_fsm_state5;
reg   [2:0] j_9_reg_143;
wire    ap_CS_fsm_state9;
reg    grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_start_reg;
wire    ap_CS_fsm_state10;
reg    grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_start_reg;
wire    ap_CS_fsm_state11;
wire    ap_CS_fsm_state12;
reg   [3:0] i_fu_56;
reg   [3:0] i_20_fu_60;
wire   [31:0] grp_fu_174_p2;
reg   [31:0] grp_fu_174_p0;
reg   [31:0] grp_fu_174_p1;
wire   [5:0] zext_ln177_fu_217_p1;
wire   [5:0] add_ln178_fu_233_p2;
wire   [5:0] zext_ln168_fu_271_p1;
wire   [5:0] add_ln169_fu_287_p2;
reg   [1:0] grp_fu_174_opcode;
reg    grp_fu_174_ce;
reg   [11:0] ap_NS_fsm;
reg    ap_ST_fsm_state1_blk;
wire    ap_ST_fsm_state2_blk;
wire    ap_ST_fsm_state3_blk;
wire    ap_ST_fsm_state4_blk;
wire    ap_ST_fsm_state5_blk;
wire    ap_ST_fsm_state6_blk;
wire    ap_ST_fsm_state7_blk;
wire    ap_ST_fsm_state8_blk;
wire    ap_ST_fsm_state9_blk;
reg    ap_ST_fsm_state10_blk;
wire    ap_ST_fsm_state11_blk;
reg    ap_ST_fsm_state12_blk;
wire    ap_ce_reg;

// power-on initialization
initial begin
#0 ap_CS_fsm = 12'd1;
#0 grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_start_reg = 1'b0;
#0 grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_start_reg = 1'b0;
end

tracking_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2 grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154(
    .ap_clk(ap_clk),
    .ap_rst(ap_rst),
    .ap_start(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_start),
    .ap_done(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_done),
    .ap_idle(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_idle),
    .ap_ready(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_ready),
    .g_1_address0(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_g_1_address0),
    .g_1_ce0(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_g_1_ce0),
    .g_1_q0(g_1_q0),
    .x_1_address0(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_x_1_address0),
    .x_1_ce0(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_x_1_ce0),
    .x_1_q0(x_1_q0),
    .s1_1_address0(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_s1_1_address0),
    .s1_1_ce0(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_s1_1_ce0),
    .s1_1_we0(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_s1_1_we0),
    .s1_1_d0(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_s1_1_d0),
    .grp_fu_174_p_din0(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_grp_fu_174_p_din0),
    .grp_fu_174_p_din1(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_grp_fu_174_p_din1),
    .grp_fu_174_p_opcode(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_grp_fu_174_p_opcode),
    .grp_fu_174_p_dout0(grp_fu_174_p2),
    .grp_fu_174_p_ce(grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_grp_fu_174_p_ce)
);

tracking_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2 grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164(
    .ap_clk(ap_clk),
    .ap_rst(ap_rst),
    .ap_start(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_start),
    .ap_done(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_done),
    .ap_idle(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_idle),
    .ap_ready(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_ready),
    .s1_1_address0(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_s1_1_address0),
    .s1_1_ce0(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_s1_1_ce0),
    .s1_1_q0(s1_1_q0),
    .vnew_1_address0(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_vnew_1_address0),
    .vnew_1_ce0(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_vnew_1_ce0),
    .vnew_1_q0(vnew_1_q0),
    .g_1_address0(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_g_1_address0),
    .g_1_ce0(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_g_1_ce0),
    .g_1_we0(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_g_1_we0),
    .g_1_d0(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_g_1_d0),
    .grp_fu_174_p_din0(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_grp_fu_174_p_din0),
    .grp_fu_174_p_din1(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_grp_fu_174_p_din1),
    .grp_fu_174_p_opcode(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_grp_fu_174_p_opcode),
    .grp_fu_174_p_dout0(grp_fu_174_p2),
    .grp_fu_174_p_ce(grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_grp_fu_174_p_ce)
);

tracking_faddfsub_32ns_32ns_32_2_full_dsp_1 #(
    .ID( 1 ),
    .NUM_STAGE( 2 ),
    .din0_WIDTH( 32 ),
    .din1_WIDTH( 32 ),
    .dout_WIDTH( 32 ))
faddfsub_32ns_32ns_32_2_full_dsp_1_U117(
    .clk(ap_clk),
    .reset(ap_rst),
    .din0(grp_fu_174_p0),
    .din1(grp_fu_174_p1),
    .opcode(grp_fu_174_opcode),
    .ce(grp_fu_174_ce),
    .dout(grp_fu_174_p2)
);

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        ap_CS_fsm <= ap_ST_fsm_state1;
    end else begin
        ap_CS_fsm <= ap_NS_fsm;
    end
end

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_start_reg <= 1'b0;
    end else begin
        if ((1'b1 == ap_CS_fsm_state11)) begin
            grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_start_reg <= 1'b1;
        end else if ((grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_ready == 1'b1)) begin
            grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_start_reg <= 1'b0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_start_reg <= 1'b0;
    end else begin
        if (((1'b1 == ap_CS_fsm_state6) & (icmp_ln167_fu_251_p2 == 1'd1))) begin
            grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_start_reg <= 1'b1;
        end else if ((grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_ready == 1'b1)) begin
            grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_start_reg <= 1'b0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln176_fu_192_p2 == 1'd1) & (1'b1 == ap_CS_fsm_state2))) begin
        i_20_fu_60 <= 4'd0;
    end else if (((1'b1 == ap_CS_fsm_state7) & (icmp_ln168_fu_275_p2 == 1'd1))) begin
        i_20_fu_60 <= add_ln167_reg_365;
    end
end

always @ (posedge ap_clk) begin
    if (((1'b1 == ap_CS_fsm_state1) & (ap_start == 1'b1))) begin
        i_fu_56 <= 4'd0;
    end else if (((1'b1 == ap_CS_fsm_state3) & (icmp_ln177_fu_221_p2 == 1'd1))) begin
        i_fu_56 <= add_ln176_reg_312;
    end
end

always @ (posedge ap_clk) begin
    if (((1'b1 == ap_CS_fsm_state6) & (icmp_ln167_fu_251_p2 == 1'd0))) begin
        j_9_reg_143 <= 3'd0;
    end else if ((1'b1 == ap_CS_fsm_state9)) begin
        j_9_reg_143 <= add_ln168_reg_378;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln176_fu_192_p2 == 1'd0) & (1'b1 == ap_CS_fsm_state2))) begin
        j_reg_132 <= 3'd0;
    end else if ((1'b1 == ap_CS_fsm_state5)) begin
        j_reg_132 <= add_ln177_reg_332;
    end
end

always @ (posedge ap_clk) begin
    if ((1'b1 == ap_CS_fsm_state6)) begin
        add_ln167_reg_365 <= add_ln167_fu_257_p2;
    end
end

always @ (posedge ap_clk) begin
    if ((1'b1 == ap_CS_fsm_state7)) begin
        add_ln168_reg_378 <= add_ln168_fu_281_p2;
    end
end

always @ (posedge ap_clk) begin
    if ((1'b1 == ap_CS_fsm_state2)) begin
        add_ln176_reg_312 <= add_ln176_fu_198_p2;
    end
end

always @ (posedge ap_clk) begin
    if ((1'b1 == ap_CS_fsm_state3)) begin
        add_ln177_reg_332 <= add_ln177_fu_227_p2;
    end
end

always @ (posedge ap_clk) begin
    if (((icmp_ln176_fu_192_p2 == 1'd0) & (1'b1 == ap_CS_fsm_state2))) begin
        tmp_reg_317[5 : 2] <= tmp_fu_204_p3[5 : 2];
    end
end

always @ (posedge ap_clk) begin
    if (((1'b1 == ap_CS_fsm_state6) & (icmp_ln167_fu_251_p2 == 1'd0))) begin
        tmp_s_reg_370[5 : 2] <= tmp_s_fu_263_p3[5 : 2];
    end
end

always @ (posedge ap_clk) begin
    if (((1'b1 == ap_CS_fsm_state7) & (icmp_ln168_fu_275_p2 == 1'd0))) begin
        zext_ln169_reg_383[5 : 0] <= zext_ln169_fu_292_p1[5 : 0];
    end
end

always @ (posedge ap_clk) begin
    if (((1'b1 == ap_CS_fsm_state3) & (icmp_ln177_fu_221_p2 == 1'd0))) begin
        zext_ln178_reg_337[5 : 0] <= zext_ln178_fu_238_p1[5 : 0];
    end
end

always @ (*) begin
    if ((grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_done == 1'b0)) begin
        ap_ST_fsm_state10_blk = 1'b1;
    end else begin
        ap_ST_fsm_state10_blk = 1'b0;
    end
end

assign ap_ST_fsm_state11_blk = 1'b0;

always @ (*) begin
    if ((grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_done == 1'b0)) begin
        ap_ST_fsm_state12_blk = 1'b1;
    end else begin
        ap_ST_fsm_state12_blk = 1'b0;
    end
end

always @ (*) begin
    if ((ap_start == 1'b0)) begin
        ap_ST_fsm_state1_blk = 1'b1;
    end else begin
        ap_ST_fsm_state1_blk = 1'b0;
    end
end

assign ap_ST_fsm_state2_blk = 1'b0;

assign ap_ST_fsm_state3_blk = 1'b0;

assign ap_ST_fsm_state4_blk = 1'b0;

assign ap_ST_fsm_state5_blk = 1'b0;

assign ap_ST_fsm_state6_blk = 1'b0;

assign ap_ST_fsm_state7_blk = 1'b0;

assign ap_ST_fsm_state8_blk = 1'b0;

assign ap_ST_fsm_state9_blk = 1'b0;

always @ (*) begin
    if ((((1'b1 == ap_CS_fsm_state1) & (ap_start == 1'b0)) | ((1'b1 == ap_CS_fsm_state12) & (grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_done == 1'b1)))) begin
        ap_done = 1'b1;
    end else begin
        ap_done = 1'b0;
    end
end

always @ (*) begin
    if (((1'b1 == ap_CS_fsm_state1) & (ap_start == 1'b0))) begin
        ap_idle = 1'b1;
    end else begin
        ap_idle = 1'b0;
    end
end

always @ (*) begin
    if (((1'b1 == ap_CS_fsm_state12) & (grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_done == 1'b1))) begin
        ap_ready = 1'b1;
    end else begin
        ap_ready = 1'b0;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state12)) begin
        g_1_address0 = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_g_1_address0;
    end else if ((1'b1 == ap_CS_fsm_state10)) begin
        g_1_address0 = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_g_1_address0;
    end else begin
        g_1_address0 = 'bx;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state12)) begin
        g_1_ce0 = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_g_1_ce0;
    end else if ((1'b1 == ap_CS_fsm_state10)) begin
        g_1_ce0 = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_g_1_ce0;
    end else begin
        g_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state12)) begin
        g_1_we0 = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_g_1_we0;
    end else begin
        g_1_we0 = 1'b0;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state12)) begin
        grp_fu_174_ce = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_grp_fu_174_p_ce;
    end else if ((1'b1 == ap_CS_fsm_state10)) begin
        grp_fu_174_ce = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_grp_fu_174_p_ce;
    end else begin
        grp_fu_174_ce = 1'b1;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state12)) begin
        grp_fu_174_opcode = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_grp_fu_174_p_opcode;
    end else if ((1'b1 == ap_CS_fsm_state10)) begin
        grp_fu_174_opcode = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_grp_fu_174_p_opcode;
    end else if ((1'b1 == ap_CS_fsm_state8)) begin
        grp_fu_174_opcode = 2'd1;
    end else if ((1'b1 == ap_CS_fsm_state4)) begin
        grp_fu_174_opcode = 2'd0;
    end else begin
        grp_fu_174_opcode = 'bx;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state12)) begin
        grp_fu_174_p0 = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_grp_fu_174_p_din0;
    end else if ((1'b1 == ap_CS_fsm_state10)) begin
        grp_fu_174_p0 = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_grp_fu_174_p_din0;
    end else if ((1'b1 == ap_CS_fsm_state8)) begin
        grp_fu_174_p0 = m1_1_q0;
    end else if ((1'b1 == ap_CS_fsm_state4)) begin
        grp_fu_174_p0 = y_1_q0;
    end else begin
        grp_fu_174_p0 = 'bx;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state12)) begin
        grp_fu_174_p1 = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_grp_fu_174_p_din1;
    end else if ((1'b1 == ap_CS_fsm_state10)) begin
        grp_fu_174_p1 = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_grp_fu_174_p_din1;
    end else if ((1'b1 == ap_CS_fsm_state8)) begin
        grp_fu_174_p1 = znew_1_q0;
    end else if ((1'b1 == ap_CS_fsm_state4)) begin
        grp_fu_174_p1 = u_1_q0;
    end else begin
        grp_fu_174_p1 = 'bx;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state7)) begin
        m1_1_address0 = zext_ln169_fu_292_p1;
    end else if ((1'b1 == ap_CS_fsm_state5)) begin
        m1_1_address0 = zext_ln178_reg_337;
    end else begin
        m1_1_address0 = 'bx;
    end
end

always @ (*) begin
    if (((1'b1 == ap_CS_fsm_state5) | (1'b1 == ap_CS_fsm_state7))) begin
        m1_1_ce0 = 1'b1;
    end else begin
        m1_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state5)) begin
        m1_1_we0 = 1'b1;
    end else begin
        m1_1_we0 = 1'b0;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state12)) begin
        s1_1_address0 = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_s1_1_address0;
    end else if ((1'b1 == ap_CS_fsm_state10)) begin
        s1_1_address0 = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_s1_1_address0;
    end else begin
        s1_1_address0 = 'bx;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state12)) begin
        s1_1_ce0 = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_s1_1_ce0;
    end else if ((1'b1 == ap_CS_fsm_state10)) begin
        s1_1_ce0 = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_s1_1_ce0;
    end else begin
        s1_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state10)) begin
        s1_1_we0 = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_s1_1_we0;
    end else begin
        s1_1_we0 = 1'b0;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state3)) begin
        u_1_ce0 = 1'b1;
    end else begin
        u_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state9)) begin
        y_1_address0 = zext_ln169_reg_383;
    end else if ((1'b1 == ap_CS_fsm_state3)) begin
        y_1_address0 = zext_ln178_fu_238_p1;
    end else begin
        y_1_address0 = 'bx;
    end
end

always @ (*) begin
    if (((1'b1 == ap_CS_fsm_state9) | (1'b1 == ap_CS_fsm_state3))) begin
        y_1_ce0 = 1'b1;
    end else begin
        y_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state9)) begin
        y_1_we0 = 1'b1;
    end else begin
        y_1_we0 = 1'b0;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state7)) begin
        znew_1_ce0 = 1'b1;
    end else begin
        znew_1_ce0 = 1'b0;
    end
end

always @ (*) begin
    case (ap_CS_fsm)
        ap_ST_fsm_state1 : begin
            if (((1'b1 == ap_CS_fsm_state1) & (ap_start == 1'b1))) begin
                ap_NS_fsm = ap_ST_fsm_state2;
            end else begin
                ap_NS_fsm = ap_ST_fsm_state1;
            end
        end
        ap_ST_fsm_state2 : begin
            if (((icmp_ln176_fu_192_p2 == 1'd1) & (1'b1 == ap_CS_fsm_state2))) begin
                ap_NS_fsm = ap_ST_fsm_state6;
            end else begin
                ap_NS_fsm = ap_ST_fsm_state3;
            end
        end
        ap_ST_fsm_state3 : begin
            if (((1'b1 == ap_CS_fsm_state3) & (icmp_ln177_fu_221_p2 == 1'd1))) begin
                ap_NS_fsm = ap_ST_fsm_state2;
            end else begin
                ap_NS_fsm = ap_ST_fsm_state4;
            end
        end
        ap_ST_fsm_state4 : begin
            ap_NS_fsm = ap_ST_fsm_state5;
        end
        ap_ST_fsm_state5 : begin
            ap_NS_fsm = ap_ST_fsm_state3;
        end
        ap_ST_fsm_state6 : begin
            if (((1'b1 == ap_CS_fsm_state6) & (icmp_ln167_fu_251_p2 == 1'd0))) begin
                ap_NS_fsm = ap_ST_fsm_state7;
            end else begin
                ap_NS_fsm = ap_ST_fsm_state10;
            end
        end
        ap_ST_fsm_state7 : begin
            if (((1'b1 == ap_CS_fsm_state7) & (icmp_ln168_fu_275_p2 == 1'd1))) begin
                ap_NS_fsm = ap_ST_fsm_state6;
            end else begin
                ap_NS_fsm = ap_ST_fsm_state8;
            end
        end
        ap_ST_fsm_state8 : begin
            ap_NS_fsm = ap_ST_fsm_state9;
        end
        ap_ST_fsm_state9 : begin
            ap_NS_fsm = ap_ST_fsm_state7;
        end
        ap_ST_fsm_state10 : begin
            if (((1'b1 == ap_CS_fsm_state10) & (grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_done == 1'b1))) begin
                ap_NS_fsm = ap_ST_fsm_state11;
            end else begin
                ap_NS_fsm = ap_ST_fsm_state10;
            end
        end
        ap_ST_fsm_state11 : begin
            ap_NS_fsm = ap_ST_fsm_state12;
        end
        ap_ST_fsm_state12 : begin
            if (((1'b1 == ap_CS_fsm_state12) & (grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_done == 1'b1))) begin
                ap_NS_fsm = ap_ST_fsm_state1;
            end else begin
                ap_NS_fsm = ap_ST_fsm_state12;
            end
        end
        default : begin
            ap_NS_fsm = 'bx;
        end
    endcase
end

assign add_ln167_fu_257_p2 = (i_20_fu_60 + 4'd1);

assign add_ln168_fu_281_p2 = (j_9_reg_143 + 3'd1);

assign add_ln169_fu_287_p2 = (zext_ln168_fu_271_p1 + tmp_s_reg_370);

assign add_ln176_fu_198_p2 = (i_fu_56 + 4'd1);

assign add_ln177_fu_227_p2 = (j_reg_132 + 3'd1);

assign add_ln178_fu_233_p2 = (zext_ln177_fu_217_p1 + tmp_reg_317);

assign ap_CS_fsm_state1 = ap_CS_fsm[32'd0];

assign ap_CS_fsm_state10 = ap_CS_fsm[32'd9];

assign ap_CS_fsm_state11 = ap_CS_fsm[32'd10];

assign ap_CS_fsm_state12 = ap_CS_fsm[32'd11];

assign ap_CS_fsm_state2 = ap_CS_fsm[32'd1];

assign ap_CS_fsm_state3 = ap_CS_fsm[32'd2];

assign ap_CS_fsm_state4 = ap_CS_fsm[32'd3];

assign ap_CS_fsm_state5 = ap_CS_fsm[32'd4];

assign ap_CS_fsm_state6 = ap_CS_fsm[32'd5];

assign ap_CS_fsm_state7 = ap_CS_fsm[32'd6];

assign ap_CS_fsm_state8 = ap_CS_fsm[32'd7];

assign ap_CS_fsm_state9 = ap_CS_fsm[32'd8];

assign g_1_d0 = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_g_1_d0;

assign grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_start = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_ap_start_reg;

assign grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_start = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_ap_start_reg;

assign icmp_ln167_fu_251_p2 = ((i_20_fu_60 == 4'd9) ? 1'b1 : 1'b0);

assign icmp_ln168_fu_275_p2 = ((j_9_reg_143 == 3'd4) ? 1'b1 : 1'b0);

assign icmp_ln176_fu_192_p2 = ((i_fu_56 == 4'd9) ? 1'b1 : 1'b0);

assign icmp_ln177_fu_221_p2 = ((j_reg_132 == 3'd4) ? 1'b1 : 1'b0);

assign m1_1_d0 = grp_fu_174_p2;

assign s1_1_d0 = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_s1_1_d0;

assign tmp_fu_204_p3 = {{i_fu_56}, {2'd0}};

assign tmp_s_fu_263_p3 = {{i_20_fu_60}, {2'd0}};

assign u_1_address0 = zext_ln178_fu_238_p1;

assign vnew_1_address0 = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_vnew_1_address0;

assign vnew_1_ce0 = grp_update_dual_Pipeline_VITIS_LOOP_167_1_VITIS_LOOP_168_2_fu_164_vnew_1_ce0;

assign x_1_address0 = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_x_1_address0;

assign x_1_ce0 = grp_update_dual_Pipeline_VITIS_LOOP_176_1_VITIS_LOOP_177_2_fu_154_x_1_ce0;

assign y_1_d0 = grp_fu_174_p2;

assign zext_ln168_fu_271_p1 = j_9_reg_143;

assign zext_ln169_fu_292_p1 = add_ln169_fu_287_p2;

assign zext_ln177_fu_217_p1 = j_reg_132;

assign zext_ln178_fu_238_p1 = add_ln178_fu_233_p2;

assign znew_1_address0 = zext_ln169_fu_292_p1;

always @ (posedge ap_clk) begin
    tmp_reg_317[1:0] <= 2'b00;
    zext_ln178_reg_337[63:6] <= 58'b0000000000000000000000000000000000000000000000000000000000;
    tmp_s_reg_370[1:0] <= 2'b00;
    zext_ln169_reg_383[63:6] <= 58'b0000000000000000000000000000000000000000000000000000000000;
end

endmodule //tracking_update_dual