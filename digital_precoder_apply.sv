`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// digital_precoder_apply.sv
//
// Digital Baseband Precoder Apply Module.
// This runtime-path block applies active F_BB coefficients to incoming data streams before Radio Frequency precoding.
// It is separate from the coefficient-update path and uses coefficients stored in coeff_ram.
// The current body is a lightweight placeholder for the eventual digital matrix multiply datapath.
//
// Interface notes:
//   - valid qualifies the input stream vector.
//   - f_bb_*_flat should be held stable by coeff_ram during runtime use.
//   - Final implementation will compute u = F_BB * s.
// -----------------------------------------------------------------------------

module digital_precoder_apply #(
    parameter int unsigned NS=8, NRF=8, W=24
)(
    input  logic valid,
    input  logic [NS*W-1:0] s_re_flat, input logic [NS*W-1:0] s_im_flat,
    input  logic [NRF*NS*W-1:0] f_bb_re_flat, input logic [NRF*NS*W-1:0] f_bb_im_flat,
    output logic [NRF*W-1:0] u_re_flat, output logic [NRF*W-1:0] u_im_flat
);
    // Placeholder behavior forwards the lowest NRF streams to keep the runtime
    // path connected while the baseband matrix multiply is under design.
    assign u_re_flat = valid ? s_re_flat[NRF*W-1:0] : {NRF*W{1'b0}};
    assign u_im_flat = valid ? s_im_flat[NRF*W-1:0] : {NRF*W{1'b0}};
endmodule
