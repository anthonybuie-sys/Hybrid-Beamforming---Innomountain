`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// frf_phase_projector_lane.sv
//
// Radio Frequency Precoder Phase Projection Lane.
//
// This lane maps one complex M entry onto a unit-magnitude F_RF entry. The
// current implementation is an axis-aligned phase projection used to make the
// hardware path concrete and synthesis-friendly. A true Coordinate Rotation
// Digital Computer or lookup-table sine/cosine lane can replace this module
// without changing the parent scheduler interface.
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

    function automatic logic [W-1:0] abs_signed(input logic signed [W-1:0] value);
        begin
            if (value[W-1])
                abs_signed = $unsigned((~value) + {{(W-1){1'b0}}, 1'b1});
            else
                abs_signed = $unsigned(value);
        end
    endfunction

    always @* begin
        f_rf_re = '0;
        f_rf_im = '0;

        if ((m_re == '0) && (m_im == '0)) begin
            f_rf_re = unit_value();
            f_rf_im = '0;
        end else if (abs_signed(m_re) >= abs_signed(m_im)) begin
            f_rf_re = m_re[W-1] ? -unit_value() : unit_value();
            f_rf_im = '0;
        end else begin
            f_rf_re = '0;
            f_rf_im = m_im[W-1] ? -unit_value() : unit_value();
        end
    end

endmodule
