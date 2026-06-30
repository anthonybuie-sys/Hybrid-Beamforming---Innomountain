`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// srs_ls_estimator.sv
//
// Sounding Reference Signal Least Squares Channel Estimator.
// The top-level coefficient-update finite state machine starts this block first.
// It consumes received sounding reference signal samples and produces the initial channel estimate for the DFT-MMSE estimator.
// The current body is a functional placeholder that forwards input samples into the LS estimate registers.
// -----------------------------------------------------------------------------

module srs_ls_estimator #(
    parameter int unsigned NT=64, NR=16, W=24
)(
    input  logic clk, input logic rst, input logic start,
    input  logic [NR*NT*W-1:0] y_re_flat, input logic [NR*NT*W-1:0] y_im_flat,
    output logic [NR*NT*W-1:0] h_ls_re_flat, output logic [NR*NT*W-1:0] h_ls_im_flat,
    output logic done
);
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin done <= 1'b0; h_ls_re_flat <= 0; h_ls_im_flat <= 0; end
        else begin done <= start; if (start) begin h_ls_re_flat <= y_re_flat; h_ls_im_flat <= y_im_flat; end end
    end
endmodule
