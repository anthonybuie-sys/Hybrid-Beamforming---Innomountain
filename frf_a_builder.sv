`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// frf_a_builder.sv
//
// Radio Frequency Precoder A Matrix Builder.
// This block computes A = F_opt^H * F_RF for the PE-AltMin loop.
// It receives F_opt from the partial SVD path and the current F_RF seed from the PE-AltMin controller.
// Its A matrix output feeds the small SVD / Procrustes stage.
//
// Matrix layout:
//   - F_opt is flattened as NT rows by NS columns.
//   - F_RF is flattened as NT rows by NRF columns.
//   - A is flattened as NS rows by NRF columns.
//
// Arithmetic:
//   - Computes conj(F_opt[n, s]) * F_RF[n, r].
//   - Accumulates across transmit antenna index n.
//   - Rounds/saturates from expanded accumulator width back to W bits.
// -----------------------------------------------------------------------------

module frf_a_builder #(
    parameter int unsigned NT        = 1024,
    parameter int unsigned NS        = 8,
    parameter int unsigned NRF       = 8,
    parameter int unsigned W         = 24,
    parameter int unsigned FRAC      = W/2,
    parameter int unsigned MAC_LANES = 8,
    parameter int unsigned MAC_PIPE  = 8
)(
    input  logic clk,
    input  logic rst,
    input  logic start,
    input  logic [NT*NS*W-1:0]  f_opt_re_flat,
    input  logic [NT*NS*W-1:0]  f_opt_im_flat,
    input  logic [NT*NRF*W-1:0] f_rf_re_flat,
    input  logic [NT*NRF*W-1:0] f_rf_im_flat,
    output logic [NS*NRF*W-1:0] a_re_flat,
    output logic [NS*NRF*W-1:0] a_im_flat,
    output logic busy,
    output logic done
);

    // Product width is 2*W. Extra guard bits cover the NT-term reduction and
    // leave margin for signed accumulation before rounding.
    localparam int unsigned ACC_W = (2*W) + $clog2(NT + 1) + 2;

    typedef enum logic [1:0] {
        ST_IDLE,
        ST_RUN,
        ST_DRAIN
    } state_t;

    state_t state;

    logic [31:0] n_base;
    logic [31:0] ns_idx;
    logic [31:0] rf_idx;
    logic [31:0] drain_count;

    logic signed [ACC_W-1:0] acc_re;
    logic signed [ACC_W-1:0] acc_im;
    logic signed [ACC_W-1:0] lane_sum_re;
    logic signed [ACC_W-1:0] lane_sum_im;
    logic signed [ACC_W-1:0] next_acc_re;
    logic signed [ACC_W-1:0] next_acc_im;

    // Convert the accumulator back to the external fixed-point format. This is
    // the common rounding/saturation point for each completed A element.
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

            if (FRAC == 0) begin
                rounded = value;
            end else if (value[ACC_W-1]) begin
                rounded = value - ({{(ACC_W-1){1'b0}}, 1'b1} << (FRAC - 1));
            end else begin
                rounded = value + ({{(ACC_W-1){1'b0}}, 1'b1} << (FRAC - 1));
            end

            shifted = rounded >>> FRAC;

            if (shifted > ext_max)
                sat_round = out_max;
            else if (shifted < ext_min)
                sat_round = out_min;
            else
                sat_round = shifted[W-1:0];
        end
    endfunction

    // Lane combiner: each lane handles one transmit-antenna term in the current
    // reduction chunk. MAC_LANES controls functional parallelism.
    always @* begin
        lane_sum_re = '0;
        lane_sum_im = '0;

        for (int unsigned lane = 0; lane < MAC_LANES; lane++) begin
            logic [31:0] n_idx;
            logic [31:0] f_opt_index;
            logic [31:0] f_rf_index;
            logic signed [W-1:0] opt_re;
            logic signed [W-1:0] opt_im;
            logic signed [W-1:0] rf_re;
            logic signed [W-1:0] rf_im;
            logic signed [(2*W)-1:0] prod_re;
            logic signed [(2*W)-1:0] prod_im;

            n_idx = n_base + lane[31:0];

            if ((state == ST_RUN) && (n_idx < NT)) begin
                f_opt_index = (n_idx * NS) + ns_idx;
                f_rf_index  = (n_idx * NRF) + rf_idx;

                opt_re = f_opt_re_flat[f_opt_index*W +: W];
                opt_im = f_opt_im_flat[f_opt_index*W +: W];
                rf_re  = f_rf_re_flat[f_rf_index*W +: W];
                rf_im  = f_rf_im_flat[f_rf_index*W +: W];

                // Complex product: conj(F_opt) * F_RF.
                prod_re = (opt_re * rf_re) + (opt_im * rf_im);
                prod_im = (opt_re * rf_im) - (opt_im * rf_re);

                lane_sum_re += {{(ACC_W-(2*W)){prod_re[(2*W)-1]}}, prod_re};
                lane_sum_im += {{(ACC_W-(2*W)){prod_im[(2*W)-1]}}, prod_im};
            end
        end
    end

    assign next_acc_re = acc_re + lane_sum_re;
    assign next_acc_im = acc_im + lane_sum_im;

    // Scheduler:
    //   rf_idx selects the RF chain column of F_RF/A.
    //   ns_idx selects the data-stream row of A.
    //   n_base walks the transmit-antenna reduction in MAC_LANES chunks.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= ST_IDLE;
            n_base    <= 32'd0;
            ns_idx    <= 32'd0;
            rf_idx    <= 32'd0;
            drain_count <= 32'd0;
            acc_re    <= '0;
            acc_im    <= '0;
            a_re_flat <= '0;
            a_im_flat <= '0;
            busy      <= 1'b0;
            done      <= 1'b0;
        end else if (start) begin
            state     <= ST_RUN;
            n_base    <= 32'd0;
            ns_idx    <= 32'd0;
            rf_idx    <= 32'd0;
            drain_count <= 32'd0;
            acc_re    <= '0;
            acc_im    <= '0;
            a_re_flat <= '0;
            a_im_flat <= '0;
            busy      <= 1'b1;
            done      <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                end

                ST_RUN: begin
                    if (n_base + MAC_LANES[31:0] >= NT[31:0]) begin
                        // Last reduction chunk for this A element. Commit the
                        // rounded result and advance to the next output element.
                        a_re_flat[((ns_idx * NRF) + rf_idx)*W +: W] <= sat_round(next_acc_re);
                        a_im_flat[((ns_idx * NRF) + rf_idx)*W +: W] <= sat_round(next_acc_im);
                        acc_re <= '0;
                        acc_im <= '0;
                        n_base <= 32'd0;

                        if (rf_idx + 32'd1 < NRF[31:0]) begin
                            rf_idx <= rf_idx + 32'd1;
                        end else begin
                            rf_idx <= 32'd0;
                            if (ns_idx + 32'd1 < NS[31:0]) begin
                                ns_idx <= ns_idx + 32'd1;
                            end else begin
                                state <= ST_DRAIN;
                            end
                        end
                    end else begin
                        // More transmit-antenna terms remain for this A element.
                        acc_re <= next_acc_re;
                        acc_im <= next_acc_im;
                        n_base <= n_base + MAC_LANES[31:0];
                    end
                end

                ST_DRAIN: begin
                    if ((MAC_PIPE <= 1) || (drain_count + 32'd1 >= MAC_PIPE[31:0])) begin
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
