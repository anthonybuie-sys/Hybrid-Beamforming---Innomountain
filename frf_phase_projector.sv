`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// frf_phase_projector.sv
//
// Radio Frequency Precoder Phase Projection Engine.
// This block walks through the M matrix and dispatches entries to frf_phase_projector_lane instances.
// Each lane maps one complex M entry into one unit-magnitude F_RF coefficient.
// The completed F_RF output is captured by pe_altmin_frf_engine and used in the next iteration or returned to the top-level update path.
// -----------------------------------------------------------------------------

module frf_phase_projector #(
    parameter int unsigned NT           = 1024,
    parameter int unsigned NRF          = 8,
    parameter int unsigned W            = 24,
    parameter int unsigned FRAC         = W/2,
    parameter int unsigned CORDIC_LANES = 8,
    parameter int unsigned CORDIC_LAT   = 24
)(
    input  logic clk,
    input  logic rst,
    input  logic start,
    input  logic [NT*NRF*W-1:0] m_re_flat,
    input  logic [NT*NRF*W-1:0] m_im_flat,
    output logic [NT*NRF*W-1:0] f_rf_re_flat,
    output logic [NT*NRF*W-1:0] f_rf_im_flat,
    output logic busy,
    output logic done
);

    localparam int unsigned ELEMENTS = NT * NRF;

    typedef enum logic [1:0] {
        ST_IDLE,
        ST_RUN,
        ST_DRAIN
    } state_t;

    state_t state;

    logic [31:0] elem_base;
    logic [31:0] drain_count;

    logic signed [W-1:0] lane_m_re   [0:CORDIC_LANES-1];
    logic signed [W-1:0] lane_m_im   [0:CORDIC_LANES-1];
    logic signed [W-1:0] lane_out_re [0:CORDIC_LANES-1];
    logic signed [W-1:0] lane_out_im [0:CORDIC_LANES-1];

    genvar lane_g;
    generate
        for (lane_g = 0; lane_g < CORDIC_LANES; lane_g++) begin : gen_phase_lanes
            frf_phase_projector_lane #(
                .W(W),
                .FRAC(FRAC)
            ) u_phase_lane (
                .m_re(lane_m_re[lane_g]),
                .m_im(lane_m_im[lane_g]),
                .f_rf_re(lane_out_re[lane_g]),
                .f_rf_im(lane_out_im[lane_g])
            );
        end
    endgenerate

    always @* begin
        for (int unsigned lane = 0; lane < CORDIC_LANES; lane++) begin
            logic [31:0] elem_idx;

            elem_idx = elem_base + lane[31:0];

            if ((state == ST_RUN) && (elem_idx < ELEMENTS[31:0])) begin
                lane_m_re[lane] = m_re_flat[elem_idx*W +: W];
                lane_m_im[lane] = m_im_flat[elem_idx*W +: W];
            end else begin
                lane_m_re[lane] = '0;
                lane_m_im[lane] = '0;
            end
        end
    end

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= ST_IDLE;
            elem_base    <= 32'd0;
            drain_count  <= 32'd0;
            f_rf_re_flat <= '0;
            f_rf_im_flat <= '0;
            busy         <= 1'b0;
            done         <= 1'b0;
        end else if (start) begin
            state        <= ST_RUN;
            elem_base    <= 32'd0;
            drain_count  <= 32'd0;
            f_rf_re_flat <= '0;
            f_rf_im_flat <= '0;
            busy         <= 1'b1;
            done         <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                end

                ST_RUN: begin
                    for (int unsigned lane = 0; lane < CORDIC_LANES; lane++) begin
                        logic [31:0] elem_idx;

                        elem_idx = elem_base + lane[31:0];

                        if (elem_idx < ELEMENTS[31:0]) begin
                            f_rf_re_flat[elem_idx*W +: W] <= lane_out_re[lane];
                            f_rf_im_flat[elem_idx*W +: W] <= lane_out_im[lane];
                        end
                    end

                    if (elem_base + CORDIC_LANES[31:0] >= ELEMENTS[31:0]) begin
                        state <= ST_DRAIN;
                    end else begin
                        elem_base <= elem_base + CORDIC_LANES[31:0];
                    end
                end

                ST_DRAIN: begin
                    if ((CORDIC_LAT <= 1) || (drain_count + 32'd1 >= CORDIC_LAT[31:0])) begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        drain_count <= drain_count + 32'd1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
