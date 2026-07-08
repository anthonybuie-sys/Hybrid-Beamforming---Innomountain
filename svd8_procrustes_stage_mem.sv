`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// svd8_procrustes_stage_mem.sv
//
// Memory-backed Procrustes Stage for the PE-AltMin datapath.
//
// Interaction:
//   - Reads A matrix addresses from local matrix storage.
//   - Writes F_DD entries back to local matrix storage.
//   - Sits between frf_a_builder_mem and frf_m_builder_mem.
//
// Functional-first baseline:
//   Emits a semi-unitary identity-style F_DD matrix after SVD8_LAT cycles. The
//   final Jacobi SVD / CORDIC-rotation Procrustes implementation can replace
//   this body while preserving the memory interface and controller handshake.
//
// Replacement target:
//   Read A, compute SVD(A)=U*S*V^H, then write F_DD=V*U^H into local memory.
// -----------------------------------------------------------------------------

module svd8_procrustes_stage_mem #(
    parameter int unsigned NS        = 8,
    parameter int unsigned NRF       = 8,
    parameter int unsigned W         = 24,
    parameter int unsigned FRAC      = W/2,
    parameter int unsigned SVD8_LAT  = 300,
    parameter int unsigned A_AW      = (NS*NRF <= 1) ? 1 : $clog2(NS*NRF),
    parameter int unsigned F_DD_AW   = (NRF*NS <= 1) ? 1 : $clog2(NRF*NS)
)(
    input  logic clk,
    input  logic rst,
    input  logic start,

    output logic [A_AW-1:0] a_rd_addr,
    input  logic signed [W-1:0] a_rd_re,
    input  logic signed [W-1:0] a_rd_im,

    output logic f_dd_we,
    output logic [F_DD_AW-1:0] f_dd_wr_addr,
    output logic signed [W-1:0] f_dd_wr_re,
    output logic signed [W-1:0] f_dd_wr_im,

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

    // Address A using the NS-by-NRF layout produced by frf_a_builder_mem. The
    // baseline does not consume a_rd_* yet, but the interface is already active.
    assign a_rd_addr = (ns_idx * NRF) + rf_idx;

    // Baseline scheduler waits SVD8_LAT cycles, then writes an identity-style
    // F_DD matrix into local memory one element per cycle.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= ST_IDLE;
            count        <= 32'd0;
            rf_idx       <= 32'd0;
            ns_idx       <= 32'd0;
            f_dd_we      <= 1'b0;
            f_dd_wr_addr <= '0;
            f_dd_wr_re   <= '0;
            f_dd_wr_im   <= '0;
            busy         <= 1'b0;
            done         <= 1'b0;
        end else begin
            f_dd_we <= 1'b0;
            done    <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        state  <= ST_WAIT;
                        count  <= 32'd0;
                        rf_idx <= 32'd0;
                        ns_idx <= 32'd0;
                        busy   <= 1'b1;
                    end
                end

                ST_WAIT: begin
                    if ((SVD8_LAT <= 1) || (count + 32'd1 >= SVD8_LAT[31:0])) begin
                        state <= ST_WRITE;
                    end else begin
                        count <= count + 32'd1;
                    end
                end

                ST_WRITE: begin
                    // Semi-unitary baseline: diagonal entries are +1, all other
                    // entries are zero.
                    f_dd_we      <= 1'b1;
                    f_dd_wr_addr <= (rf_idx * NS) + ns_idx;
                    f_dd_wr_re   <= (rf_idx == ns_idx) ? unit_value() : '0;
                    f_dd_wr_im   <= '0;

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
