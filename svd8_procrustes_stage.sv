`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// svd8_procrustes_stage.sv
//
// 8x8 Singular Value Decomposition Procrustes Stage.
// This block consumes the A matrix and produces F_DD = V * U^H for the next matrix product.
// It sits between frf_a_builder and frf_m_builder in each PE-AltMin iteration.
// The current body is a latency-bounded placeholder behind the final functional interface.
// -----------------------------------------------------------------------------

module svd8_procrustes_stage #(
    parameter int unsigned NS       = 8,
    parameter int unsigned NRF      = 8,
    parameter int unsigned W        = 24,
    parameter int unsigned SVD8_LAT = 300
)(
    input  logic clk,
    input  logic rst,
    input  logic start,
    input  logic [NS*NRF*W-1:0] a_re_flat,
    input  logic [NS*NRF*W-1:0] a_im_flat,
    output logic [NRF*NS*W-1:0] f_dd_re_flat,
    output logic [NRF*NS*W-1:0] f_dd_im_flat,
    output logic busy,
    output logic done
);

    pe_altmin_latency_stage #(.LATENCY(SVD8_LAT)) u_timer (
        .clk(clk), .rst(rst), .start(start), .busy(busy), .done(done)
    );

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            f_dd_re_flat <= '0;
            f_dd_im_flat <= '0;
        end else if (start) begin
            f_dd_re_flat <= '0;
            f_dd_im_flat <= '0;
        end
    end

endmodule
