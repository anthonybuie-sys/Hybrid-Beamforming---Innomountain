`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// svd8_procrustes_stage.sv
//
// Eight-by-Eight Singular Value Decomposition Procrustes Stage.
// This block consumes the A matrix and produces F_DD = V * U^H for the next matrix product.
// It sits between frf_a_builder and frf_m_builder in each PE-AltMin iteration.
//
// Functional-first baseline:
//   The exact SVD/Procrustes datapath is still a future numerical block. For
//   now, this module emits a semi-unitary identity-style F_DD matrix after a
//   programmable latency. That keeps the PE-AltMin datapath active while the
//   true small-matrix SVD implementation is designed.
//
// Replacement target:
//   Compute SVD(A)=U*S*V^H and output F_DD=V*U^H. The interface is already
//   shaped so the true numerical block can replace this baseline later.
// -----------------------------------------------------------------------------

module svd8_procrustes_stage #(
    parameter int unsigned NS       = 8,
    parameter int unsigned NRF      = 8,
    parameter int unsigned W        = 24,
    parameter int unsigned FRAC     = W/2,
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

    typedef enum logic [1:0] {
        ST_IDLE,
        ST_WAIT,
        ST_WRITE
    } state_t;

    state_t state;
    logic [31:0] count;
    logic [31:0] rf_idx;
    logic [31:0] ns_idx;

    // Fixed-point +1.0 used on the diagonal of the baseline F_DD matrix.
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

    // Baseline scheduler:
    //   1. Wait SVD8_LAT cycles to preserve expected controller timing.
    //   2. Write an identity-style F_DD matrix one element per cycle.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= ST_IDLE;
            count        <= 32'd0;
            rf_idx       <= 32'd0;
            ns_idx       <= 32'd0;
            f_dd_re_flat <= '0;
            f_dd_im_flat <= '0;
            busy         <= 1'b0;
            done         <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        state        <= ST_WAIT;
                        count        <= 32'd0;
                        rf_idx       <= 32'd0;
                        ns_idx       <= 32'd0;
                        f_dd_re_flat <= '0;
                        f_dd_im_flat <= '0;
                        busy         <= 1'b1;
                    end
                end

                ST_WAIT: begin
                    if ((SVD8_LAT <= 1) || (count + 32'd1 >= SVD8_LAT[31:0])) begin
                        state  <= ST_WRITE;
                        count  <= 32'd0;
                    end else begin
                        count <= count + 32'd1;
                    end
                end

                ST_WRITE: begin
                    // Semi-unitary baseline: diagonal entries are +1, all other
                    // entries are zero.
                    f_dd_re_flat[((rf_idx * NS) + ns_idx)*W +: W] <=
                        (rf_idx == ns_idx) ? unit_value() : '0;
                    f_dd_im_flat[((rf_idx * NS) + ns_idx)*W +: W] <= '0;

                    if (ns_idx + 32'd1 < NS[31:0]) begin
                        ns_idx <= ns_idx + 32'd1;
                    end else begin
                        ns_idx <= 32'd0;
                        if (rf_idx + 32'd1 < NRF[31:0]) begin
                            rf_idx <= rf_idx + 32'd1;
                        end else begin
                            state <= ST_IDLE;
                            busy  <= 1'b0;
                            done  <= 1'b1;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
