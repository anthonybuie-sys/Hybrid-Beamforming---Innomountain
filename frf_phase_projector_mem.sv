`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// frf_phase_projector_mem.sv
//
// Memory-backed Radio Frequency Precoder Phase Projection Engine.
//
// Interaction:
//   - Reads M matrix entries from local matrix storage.
//   - Uses frf_phase_projector_lane to project each complex M entry.
//   - Writes updated F_RF entries back to local matrix storage.
//
// This is the functional-first memory-interface version of the phase projector.
// It uses one phase lane and sequential scheduling. Wider lane scheduling and
// memory banking are later latency-optimization steps.
//
// Memory addressing:
//   - M read address and F_RF write address use the same flattened NT*NRF index.
//   - The phase lane maps the current M sample to the replacement F_RF sample.
// -----------------------------------------------------------------------------

module frf_phase_projector_mem #(
    parameter int unsigned NT       = 1024,
    parameter int unsigned NRF      = 8,
    parameter int unsigned W        = 24,
    parameter int unsigned FRAC     = W/2,
    parameter int unsigned CORDIC_LAT = 24,
    parameter int unsigned M_AW     = (NT*NRF <= 1) ? 1 : $clog2(NT*NRF),
    parameter int unsigned F_RF_AW  = (NT*NRF <= 1) ? 1 : $clog2(NT*NRF)
)(
    input  logic clk,
    input  logic rst,
    input  logic start,

    output logic [M_AW-1:0] m_rd_addr,
    input  logic signed [W-1:0] m_rd_re,
    input  logic signed [W-1:0] m_rd_im,

    output logic f_rf_we,
    output logic [F_RF_AW-1:0] f_rf_wr_addr,
    output logic signed [W-1:0] f_rf_wr_re,
    output logic signed [W-1:0] f_rf_wr_im,

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
    logic [31:0] elem_idx;
    logic [31:0] drain_count;

    logic signed [W-1:0] lane_re;
    logic signed [W-1:0] lane_im;

    // Read the current M entry. With the current local RAM model, read data is
    // combinational and can feed the phase lane in the same cycle.
    assign m_rd_addr = elem_idx[M_AW-1:0];

    // Single-lane phase projection for the functional-first memory path.
    frf_phase_projector_lane #(
        .W(W),
        .FRAC(FRAC)
    ) u_phase_lane (
        .m_re(m_rd_re),
        .m_im(m_rd_im),
        .f_rf_re(lane_re),
        .f_rf_im(lane_im)
    );

    // Sequential scheduler walks every M entry, writes the projected value into
    // F_RF memory, then waits CORDIC_LAT cycles for future pipeline compatibility.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= ST_IDLE;
            elem_idx     <= 32'd0;
            drain_count  <= 32'd0;
            f_rf_we      <= 1'b0;
            f_rf_wr_addr <= '0;
            f_rf_wr_re   <= '0;
            f_rf_wr_im   <= '0;
            busy         <= 1'b0;
            done         <= 1'b0;
        end else begin
            f_rf_we <= 1'b0;
            done    <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        state       <= ST_RUN;
                        elem_idx    <= 32'd0;
                        drain_count <= 32'd0;
                        busy        <= 1'b1;
                    end
                end

                ST_RUN: begin
                    // Commit the projected unit-vector coefficient for the
                    // current flattened antenna/RF-chain index.
                    f_rf_we      <= 1'b1;
                    f_rf_wr_addr <= elem_idx[F_RF_AW-1:0];
                    f_rf_wr_re   <= lane_re;
                    f_rf_wr_im   <= lane_im;

                    if (elem_idx + 32'd1 >= ELEMENTS[31:0]) begin
                        state <= ST_DRAIN;
                    end else begin
                        elem_idx <= elem_idx + 32'd1;
                    end
                end

                ST_DRAIN: begin
                    if ((CORDIC_LAT <= 1) || (drain_count + 32'd1 >= CORDIC_LAT[31:0])) begin
                        state <= ST_IDLE;
                        busy  <= 1'b0;
                        done  <= 1'b1;
                    end else begin
                        drain_count <= drain_count + 32'd1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
