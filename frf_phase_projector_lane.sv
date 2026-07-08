`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// frf_phase_projector_lane.sv
//
// Radio Frequency Precoder Phase Projection Lane.
//
// This lane maps one complex M entry onto a unit-magnitude F_RF entry.
// The current implementation is a compact 8-sector lookup approximation:
// horizontal, vertical, and diagonal unit vectors are selected from the signs
// and relative magnitudes of M. This is the functional-first phase projection
// baseline; a finer lookup table or Coordinate Rotation Digital Computer lane
// can replace it later without changing the parent scheduler interface.
//
// Selection rule:
//   - If one component dominates by at least 2:1, choose the axis direction.
//   - Otherwise choose the corresponding diagonal direction.
//   - Zero input maps to +1 + j0 so the output is always well-defined.
// -----------------------------------------------------------------------------

module frf_phase_projector_lane #(
    parameter int unsigned W    = 24,
    parameter int unsigned FRAC = 14
)(
    input  logic signed [W-1:0] m_re,
    input  logic signed [W-1:0] m_im,
    output logic signed [W-1:0] f_rf_re,
    output logic signed [W-1:0] f_rf_im
);

    // Fixed-point +1.0 in the configured Q format, saturated if FRAC would
    // exceed the signed output range.
    function automatic logic signed [W-1:0] unit_value;
        logic signed [W-1:0] tmp;
        begin
            tmp = '0;
            if (FRAC >= W-1)
                tmp = {1'b0, {(W-1){1'b1}}};
            else
                tmp = ({{(W-1){1'b0}}, 1'b1} << FRAC);
            unit_value = tmp;
        end
    endfunction

    // Fixed-point 1/sqrt(2), used for diagonal unit-vector sectors.
    function automatic logic signed [W-1:0] diag_value;
        logic signed [W-1:0] tmp;
        begin
            // 0.70710678 in Q15 is approximately 23170.
            tmp = (unit_value() * 16'sd23170) >>> 15;
            diag_value = tmp;
        end
    endfunction

    // Magnitude comparison is unsigned; this helper handles two's-complement
    // inputs without widening the datapath.
    function automatic logic [W-1:0] abs_signed(input logic signed [W-1:0] value);
        begin
            if (value[W-1])
                abs_signed = $unsigned((~value) + {{(W-1){1'b0}}, 1'b1});
            else
                abs_signed = $unsigned(value);
        end
    endfunction

    logic [W-1:0] abs_re;
    logic [W-1:0] abs_im;
    logic signed [W-1:0] one_q;
    logic signed [W-1:0] diag_q;

    // Precompute common magnitudes and lookup constants for the sector decision.
    always @* begin
        abs_re = abs_signed(m_re);
        abs_im = abs_signed(m_im);
        one_q  = unit_value();
        diag_q = diag_value();
    end

    // Eight-sector phase approximation. This keeps the functional baseline
    // deterministic while leaving a clean replacement point for finer phase math.
    always @* begin
        f_rf_re = '0;
        f_rf_im = '0;

        if ((m_re == '0) && (m_im == '0)) begin
            f_rf_re = one_q;
            f_rf_im = '0;
        end else if ({1'b0, abs_im} <= {abs_re, 1'b0}) begin
            f_rf_re = m_re[W-1] ? -one_q : one_q;
            f_rf_im = '0;
        end else if ({1'b0, abs_re} <= {abs_im, 1'b0}) begin
            f_rf_re = '0;
            f_rf_im = m_im[W-1] ? -one_q : one_q;
        end else begin
            f_rf_re = m_re[W-1] ? -diag_q : diag_q;
            f_rf_im = m_im[W-1] ? -diag_q : diag_q;
        end
    end

endmodule
