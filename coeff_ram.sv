`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// coeff_ram.sv
//
// Coefficient Random Access Memory.
// This block stores the active F_RF and F_BB coefficients after an update pass completes.
// The runtime digital and analog precoder apply modules read these active coefficients.
// The update path writes this memory in the WRITE state of the top-level finite state machine.
// -----------------------------------------------------------------------------

module coeff_ram #(
    parameter int unsigned NT=64, NS=8, NRF=8, W=24
)(
    input  logic clk, input logic rst, input logic we,
    input  logic [NT*NRF*W-1:0] f_rf_wr_re_flat, input logic [NT*NRF*W-1:0] f_rf_wr_im_flat,
    input  logic [NRF*NS*W-1:0] f_bb_wr_re_flat, input logic [NRF*NS*W-1:0] f_bb_wr_im_flat,
    output logic [NT*NRF*W-1:0] f_rf_rd_re_flat, output logic [NT*NRF*W-1:0] f_rf_rd_im_flat,
    output logic [NRF*NS*W-1:0] f_bb_rd_re_flat, output logic [NRF*NS*W-1:0] f_bb_rd_im_flat
);
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            f_rf_rd_re_flat <= 0; f_rf_rd_im_flat <= 0;
            f_bb_rd_re_flat <= 0; f_bb_rd_im_flat <= 0;
        end else if (we) begin
            f_rf_rd_re_flat <= f_rf_wr_re_flat; f_rf_rd_im_flat <= f_rf_wr_im_flat;
            f_bb_rd_re_flat <= f_bb_wr_re_flat; f_bb_rd_im_flat <= f_bb_wr_im_flat;
        end
    end
endmodule
