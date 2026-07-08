`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// frf_m_builder_mem.sv
//
// Memory-backed Radio Frequency Precoder M Matrix Builder.
// This functional-first version computes M = F_opt * F_DD^H using local matrix
// memory read ports instead of large flattened input buses.
//
// Interaction:
//   - Reads F_opt and F_DD entries from pe_altmin_matrix_memory.
//   - Writes completed M entries back into the M matrix memory.
//   - Feeds the memory-backed phase projection engine path.
//
// This module is intentionally sequential. It prioritizes correctness and a
// clean memory interface before later multiply-accumulate lane optimization.
//
// Memory addressing:
//   - F_opt address = nt_idx*NS + s_idx.
//   - F_DD address  = rf_idx*NS + s_idx.
//   - M write addr  = nt_idx*NRF + rf_idx.
// -----------------------------------------------------------------------------

module frf_m_builder_mem #(
    parameter int unsigned NT         = 1024,
    parameter int unsigned NS         = 8,
    parameter int unsigned NRF        = 8,
    parameter int unsigned W          = 24,
    parameter int unsigned FRAC       = W/2,
    parameter int unsigned F_OPT_AW   = (NT*NS <= 1) ? 1 : $clog2(NT*NS),
    parameter int unsigned F_DD_AW    = (NRF*NS <= 1) ? 1 : $clog2(NRF*NS),
    parameter int unsigned M_AW       = (NT*NRF <= 1) ? 1 : $clog2(NT*NRF)
)(
    input  logic clk,
    input  logic rst,
    input  logic start,

    output logic [F_OPT_AW-1:0] f_opt_rd_addr,
    input  logic signed [W-1:0] f_opt_rd_re,
    input  logic signed [W-1:0] f_opt_rd_im,

    output logic [F_DD_AW-1:0] f_dd_rd_addr,
    input  logic signed [W-1:0] f_dd_rd_re,
    input  logic signed [W-1:0] f_dd_rd_im,

    output logic m_we,
    output logic [M_AW-1:0] m_wr_addr,
    output logic signed [W-1:0] m_wr_re,
    output logic signed [W-1:0] m_wr_im,

    output logic busy,
    output logic done
);

    // Guard bits cover the NS-term complex reduction before fixed-point
    // rounding and saturation.
    localparam int unsigned ACC_W = (2*W) + $clog2(NS + 1) + 2;

    typedef enum logic [1:0] {
        ST_IDLE,
        ST_RUN
    } state_t;

    state_t state;

    logic [31:0] s_idx;
    logic [31:0] nt_idx;
    logic [31:0] rf_idx;

    logic signed [ACC_W-1:0] acc_re;
    logic signed [ACC_W-1:0] acc_im;
    logic signed [(2*W)-1:0] prod_re;
    logic signed [(2*W)-1:0] prod_im;
    logic signed [ACC_W-1:0] next_acc_re;
    logic signed [ACC_W-1:0] next_acc_im;

    // Shared output conversion for completed M matrix elements.
    function automatic logic signed [W-1:0] sat_round(input logic signed [ACC_W-1:0] value);
        logic signed [ACC_W-1:0] rounded;
        logic signed [ACC_W-1:0] shifted;
        logic signed [W-1:0] out_max;
        logic signed [W-1:0] out_min;
        logic signed [ACC_W-1:0] ext_max;
        logic signed [ACC_W-1:0] ext_min;
        begin
            out_max = {1'b0, {(W-1){1'b1}}};
            out_min = {1'b1, {(W-1){1'b0}}};
            ext_max = {{(ACC_W-W){out_max[W-1]}}, out_max};
            ext_min = {{(ACC_W-W){out_min[W-1]}}, out_min};

            if (FRAC == 0)
                rounded = value;
            else if (value[ACC_W-1])
                rounded = value - ({{(ACC_W-1){1'b0}}, 1'b1} << (FRAC - 1));
            else
                rounded = value + ({{(ACC_W-1){1'b0}}, 1'b1} << (FRAC - 1));

            shifted = rounded >>> FRAC;

            if (shifted > ext_max)
                sat_round = out_max;
            else if (shifted < ext_min)
                sat_round = out_min;
            else
                sat_round = shifted[W-1:0];
        end
    endfunction

    // Present memory read addresses for the current stream-reduction term.
    assign f_opt_rd_addr = (nt_idx * NS) + s_idx;
    assign f_dd_rd_addr  = (rf_idx * NS) + s_idx;

    // Complex product: F_opt * conj(F_DD).
    assign prod_re = (f_opt_rd_re * f_dd_rd_re) + (f_opt_rd_im * f_dd_rd_im);
    assign prod_im = (f_opt_rd_im * f_dd_rd_re) - (f_opt_rd_re * f_dd_rd_im);

    assign next_acc_re = acc_re + {{(ACC_W-(2*W)){prod_re[(2*W)-1]}}, prod_re};
    assign next_acc_im = acc_im + {{(ACC_W-(2*W)){prod_im[(2*W)-1]}}, prod_im};

    // Sequential functional scheduler. It computes one multiply-accumulate term
    // per cycle and writes one M element after the NS reduction completes.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= ST_IDLE;
            s_idx     <= 32'd0;
            nt_idx    <= 32'd0;
            rf_idx    <= 32'd0;
            acc_re    <= '0;
            acc_im    <= '0;
            m_we      <= 1'b0;
            m_wr_addr <= '0;
            m_wr_re   <= '0;
            m_wr_im   <= '0;
            busy      <= 1'b0;
            done      <= 1'b0;
        end else begin
            m_we <= 1'b0;
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        state  <= ST_RUN;
                        busy   <= 1'b1;
                        s_idx  <= 32'd0;
                        nt_idx <= 32'd0;
                        rf_idx <= 32'd0;
                        acc_re <= '0;
                        acc_im <= '0;
                    end
                end

                ST_RUN: begin
                    if (s_idx + 32'd1 >= NS[31:0]) begin
                        // Completed the stream reduction for one M element;
                        // write the rounded value into M memory.
                        m_we      <= 1'b1;
                        m_wr_addr <= (nt_idx * NRF) + rf_idx;
                        m_wr_re   <= sat_round(next_acc_re);
                        m_wr_im   <= sat_round(next_acc_im);
                        acc_re    <= '0;
                        acc_im    <= '0;
                        s_idx     <= 32'd0;

                        if (rf_idx + 32'd1 < NRF[31:0]) begin
                            rf_idx <= rf_idx + 32'd1;
                        end else begin
                            rf_idx <= 32'd0;
                            if (nt_idx + 32'd1 < NT[31:0]) begin
                                nt_idx <= nt_idx + 32'd1;
                            end else begin
                                state <= ST_IDLE;
                                busy  <= 1'b0;
                                done  <= 1'b1;
                            end
                        end
                    end else begin
                        // Accumulate another data-stream contribution.
                        acc_re <= next_acc_re;
                        acc_im <= next_acc_im;
                        s_idx  <= s_idx + 32'd1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
