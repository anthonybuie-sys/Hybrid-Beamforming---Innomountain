`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// dft_mmse_estimator.sv
//
// Discrete Fourier Transform Minimum Mean Square Error Channel Estimator.
// This block receives the Least Squares channel estimate and produces the filtered channel estimate used by the partial SVD engine.
// It is scheduled after srs_ls_estimator and before partial_svd_engine in the coefficient-update path.
// The current body is a placeholder pass-through until the DFT-MMSE datapath is implemented.
// -----------------------------------------------------------------------------

module dft_mmse_estimator #(
    parameter int unsigned NT=64, NR=16, W=24
)(
    input  logic clk, input logic rst, input logic start,
    input  logic [NR*NT*W-1:0] h_ls_re_flat, input logic [NR*NT*W-1:0] h_ls_im_flat,
    output logic [NR*NT*W-1:0] h_est_re_flat, output logic [NR*NT*W-1:0] h_est_im_flat,
    output logic done
);
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin done <= 1'b0; h_est_re_flat <= 0; h_est_im_flat <= 0; end
        else begin done <= start; if (start) begin h_est_re_flat <= h_ls_re_flat; h_est_im_flat <= h_ls_im_flat; end end
    end
endmodule
