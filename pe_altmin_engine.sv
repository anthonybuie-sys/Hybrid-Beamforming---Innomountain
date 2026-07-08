`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// pe_altmin_engine.sv
//
// Phase Extraction Alternating Minimization wrapper.
// This wrapper adapts the top-level coefficient-update path to pe_altmin_frf_engine.
// It passes F_opt and the previous active F_RF into the Radio Frequency precoder update engine and returns the updated F_RF.
// The wrapper also carries the warm-start control from coefficient memory validity into the PE-AltMin engine.
//
// Integration notes:
//   - This module is the top-level coefficient path's view of PE-AltMin.
//   - pe_altmin_frf_engine contains the detailed iteration scheduler.
//   - Warm start is asserted after coeff_ram has a valid previous F_RF.
// -----------------------------------------------------------------------------

module pe_altmin_engine #(
    parameter int unsigned NT=64, NS=8, NRF=8, W=24, FRAC=14
)(
    input  logic clk, input logic rst, input logic start, input logic warm_start,
    input  logic [NT*NS*W-1:0] f_opt_re_flat, input logic [NT*NS*W-1:0] f_opt_im_flat,
    input  logic [NT*NRF*W-1:0] f_rf_prev_re_flat, input logic [NT*NRF*W-1:0] f_rf_prev_im_flat,
    output logic [NT*NRF*W-1:0] f_rf_re_flat, output logic [NT*NRF*W-1:0] f_rf_im_flat,
    output logic done
);
    logic [7:0]  iter_count_unused;
    logic [31:0] cycle_count_unused;
    logic [2:0]  stage_id_unused;
    logic        busy_unused;

    // Keep the wrapper thin so the top-level architecture does not depend on
    // internal PE-AltMin status signals such as stage_id and cycle_count.
    pe_altmin_frf_engine #(
        .NT(NT), .NS(NS), .NRF(NRF), .W(W),
        .FRAC(FRAC),
        .MAC_LANES(32), .CORDIC_LANES(32),
        .COLD_ITERS(20), .WARM_ITERS(3)
    ) u_frf_engine (
        .clk(clk), .rst(rst), .start(start), .warm_start(warm_start),
        .f_opt_re_flat(f_opt_re_flat), .f_opt_im_flat(f_opt_im_flat),
        .f_rf_prev_re_flat(f_rf_prev_re_flat), .f_rf_prev_im_flat(f_rf_prev_im_flat),
        .f_rf_re_flat(f_rf_re_flat), .f_rf_im_flat(f_rf_im_flat),
        .iter_count(iter_count_unused),
        .cycle_count(cycle_count_unused),
        .stage_id(stage_id_unused),
        .busy(busy_unused),
        .done(done)
    );
endmodule
