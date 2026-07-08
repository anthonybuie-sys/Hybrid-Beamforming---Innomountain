`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// pe_altmin_matrix_memory.sv
//
// Functional-first local matrix storage for the PE-AltMin datapath.
// This module groups memories for F_opt, F_RF, A, F_DD, and M behind one storage interface.
// Matrix builders, the SVD / Procrustes stage, and the phase projector will connect to this storage layer as the design moves away from flattened buses.
// The implementation intentionally favors simple functional access before later banking and latency optimization.
//
// Memory ownership by stage:
//   - F_opt: loaded by an upstream wrapper, read by A/M builders.
//   - F_RF: loaded as seed or written by phase projection, read by A builder.
//   - A: written by A builder, read by Procrustes stage.
//   - F_DD: written by Procrustes stage, read by M builder.
//   - M: written by M builder, read by phase projection.
// -----------------------------------------------------------------------------

module pe_altmin_matrix_memory #(
    parameter int unsigned NT  = 1024,
    parameter int unsigned NS  = 8,
    parameter int unsigned NRF = 8,
    parameter int unsigned W   = 24
)(
    input  logic clk,

    input  logic f_opt_we,
    input  logic [(NT*NS <= 1 ? 1 : $clog2(NT*NS))-1:0] f_opt_wr_addr,
    input  logic signed [W-1:0] f_opt_wr_re,
    input  logic signed [W-1:0] f_opt_wr_im,
    input  logic [(NT*NS <= 1 ? 1 : $clog2(NT*NS))-1:0] f_opt_rd0_addr,
    input  logic [(NT*NS <= 1 ? 1 : $clog2(NT*NS))-1:0] f_opt_rd1_addr,
    output logic signed [W-1:0] f_opt_rd0_re,
    output logic signed [W-1:0] f_opt_rd0_im,
    output logic signed [W-1:0] f_opt_rd1_re,
    output logic signed [W-1:0] f_opt_rd1_im,

    input  logic f_rf_we,
    input  logic [(NT*NRF <= 1 ? 1 : $clog2(NT*NRF))-1:0] f_rf_wr_addr,
    input  logic signed [W-1:0] f_rf_wr_re,
    input  logic signed [W-1:0] f_rf_wr_im,
    input  logic [(NT*NRF <= 1 ? 1 : $clog2(NT*NRF))-1:0] f_rf_rd0_addr,
    input  logic [(NT*NRF <= 1 ? 1 : $clog2(NT*NRF))-1:0] f_rf_rd1_addr,
    output logic signed [W-1:0] f_rf_rd0_re,
    output logic signed [W-1:0] f_rf_rd0_im,
    output logic signed [W-1:0] f_rf_rd1_re,
    output logic signed [W-1:0] f_rf_rd1_im,

    input  logic a_we,
    input  logic [(NS*NRF <= 1 ? 1 : $clog2(NS*NRF))-1:0] a_wr_addr,
    input  logic signed [W-1:0] a_wr_re,
    input  logic signed [W-1:0] a_wr_im,
    input  logic [(NS*NRF <= 1 ? 1 : $clog2(NS*NRF))-1:0] a_rd0_addr,
    input  logic [(NS*NRF <= 1 ? 1 : $clog2(NS*NRF))-1:0] a_rd1_addr,
    output logic signed [W-1:0] a_rd0_re,
    output logic signed [W-1:0] a_rd0_im,
    output logic signed [W-1:0] a_rd1_re,
    output logic signed [W-1:0] a_rd1_im,

    input  logic f_dd_we,
    input  logic [(NRF*NS <= 1 ? 1 : $clog2(NRF*NS))-1:0] f_dd_wr_addr,
    input  logic signed [W-1:0] f_dd_wr_re,
    input  logic signed [W-1:0] f_dd_wr_im,
    input  logic [(NRF*NS <= 1 ? 1 : $clog2(NRF*NS))-1:0] f_dd_rd0_addr,
    input  logic [(NRF*NS <= 1 ? 1 : $clog2(NRF*NS))-1:0] f_dd_rd1_addr,
    output logic signed [W-1:0] f_dd_rd0_re,
    output logic signed [W-1:0] f_dd_rd0_im,
    output logic signed [W-1:0] f_dd_rd1_re,
    output logic signed [W-1:0] f_dd_rd1_im,

    input  logic m_we,
    input  logic [(NT*NRF <= 1 ? 1 : $clog2(NT*NRF))-1:0] m_wr_addr,
    input  logic signed [W-1:0] m_wr_re,
    input  logic signed [W-1:0] m_wr_im,
    input  logic [(NT*NRF <= 1 ? 1 : $clog2(NT*NRF))-1:0] m_rd0_addr,
    input  logic [(NT*NRF <= 1 ? 1 : $clog2(NT*NRF))-1:0] m_rd1_addr,
    output logic signed [W-1:0] m_rd0_re,
    output logic signed [W-1:0] m_rd0_im,
    output logic signed [W-1:0] m_rd1_re,
    output logic signed [W-1:0] m_rd1_im
);

    // Fully digital reference precoder from the partial SVD path.
    complex_1w2r_ram #(.DEPTH(NT*NS), .W(W)) u_f_opt_mem (
        .clk(clk), .wr_en(f_opt_we), .wr_addr(f_opt_wr_addr),
        .wr_re(f_opt_wr_re), .wr_im(f_opt_wr_im),
        .rd0_addr(f_opt_rd0_addr), .rd1_addr(f_opt_rd1_addr),
        .rd0_re(f_opt_rd0_re), .rd0_im(f_opt_rd0_im),
        .rd1_re(f_opt_rd1_re), .rd1_im(f_opt_rd1_im)
    );

    // Analog RF precoder seed/current estimate. Phase projection writes the
    // updated coefficients back into this memory.
    complex_1w2r_ram #(.DEPTH(NT*NRF), .W(W)) u_f_rf_mem (
        .clk(clk), .wr_en(f_rf_we), .wr_addr(f_rf_wr_addr),
        .wr_re(f_rf_wr_re), .wr_im(f_rf_wr_im),
        .rd0_addr(f_rf_rd0_addr), .rd1_addr(f_rf_rd1_addr),
        .rd0_re(f_rf_rd0_re), .rd0_im(f_rf_rd0_im),
        .rd1_re(f_rf_rd1_re), .rd1_im(f_rf_rd1_im)
    );

    // Small Procrustes input matrix A = F_opt^H * F_RF.
    complex_1w2r_ram #(.DEPTH(NS*NRF), .W(W)) u_a_mem (
        .clk(clk), .wr_en(a_we), .wr_addr(a_wr_addr),
        .wr_re(a_wr_re), .wr_im(a_wr_im),
        .rd0_addr(a_rd0_addr), .rd1_addr(a_rd1_addr),
        .rd0_re(a_rd0_re), .rd0_im(a_rd0_im),
        .rd1_re(a_rd1_re), .rd1_im(a_rd1_im)
    );

    // Procrustes output F_DD = V*U^H, consumed by the M builder.
    complex_1w2r_ram #(.DEPTH(NRF*NS), .W(W)) u_f_dd_mem (
        .clk(clk), .wr_en(f_dd_we), .wr_addr(f_dd_wr_addr),
        .wr_re(f_dd_wr_re), .wr_im(f_dd_wr_im),
        .rd0_addr(f_dd_rd0_addr), .rd1_addr(f_dd_rd1_addr),
        .rd0_re(f_dd_rd0_re), .rd0_im(f_dd_rd0_im),
        .rd1_re(f_dd_rd1_re), .rd1_im(f_dd_rd1_im)
    );

    // Phase projection input M = F_opt * F_DD^H.
    complex_1w2r_ram #(.DEPTH(NT*NRF), .W(W)) u_m_mem (
        .clk(clk), .wr_en(m_we), .wr_addr(m_wr_addr),
        .wr_re(m_wr_re), .wr_im(m_wr_im),
        .rd0_addr(m_rd0_addr), .rd1_addr(m_rd1_addr),
        .rd0_re(m_rd0_re), .rd0_im(m_rd0_im),
        .rd1_re(m_rd1_re), .rd1_im(m_rd1_im)
    );

endmodule
