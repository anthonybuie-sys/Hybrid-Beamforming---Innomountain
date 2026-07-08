`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// fbb_solver.sv
//
// Baseband Precoder Solver.
// This block runs after the PE-AltMin Radio Frequency precoder update is complete.
// It receives F_opt and the new F_RF, then produces F_BB for coefficient memory.
//
// Functional-first baseline:
//   The final solver should compute a least-squares baseband precoder. This
//   version implements a matched-filter approximation, F_BB ~= F_RF^H * F_opt,
//   using fixed-point complex multiply-accumulate scheduling. It gives the
//   block a real datapath while keeping the final inverse/normalization design
//   isolated for later refinement.
//
// Matrix layout:
//   - F_opt is flattened as NT rows by NS columns.
//   - F_RF is flattened as NT rows by NRF columns.
//   - F_BB is flattened as NRF rows by NS columns.
//
// Arithmetic:
//   - Computes conj(F_RF[n, r]) * F_opt[n, s].
//   - Accumulates across transmit antenna index n.
//   - Applies a simple NT normalization shift as part of the baseline.
// -----------------------------------------------------------------------------

module fbb_solver #(
    parameter int unsigned NT=64, NS=8, NRF=8, W=24, FRAC=W/2
)(
    input  logic clk, input logic rst, input logic start,
    input  logic [NT*NS*W-1:0] f_opt_re_flat, input logic [NT*NS*W-1:0] f_opt_im_flat,
    input  logic [NT*NRF*W-1:0] f_rf_re_flat, input logic [NT*NRF*W-1:0] f_rf_im_flat,
    output logic [NRF*NS*W-1:0] f_bb_re_flat, output logic [NRF*NS*W-1:0] f_bb_im_flat,
    output logic done
);

    // Product width is 2*W. Extra guard bits cover the NT-term reduction.
    localparam int unsigned ACC_W = (2*W) + $clog2(NT + 1) + 2;

    // Baseline normalization approximates division by NT when NT is a power of
    // two. The final least-squares solve will replace this simplification.
    localparam int unsigned NORM_SHIFT = (NT <= 1) ? 0 : $clog2(NT);
    localparam int unsigned TOTAL_SHIFT = FRAC + NORM_SHIFT;

    typedef enum logic [1:0] {
        ST_IDLE,
        ST_RUN
    } state_t;

    state_t state;

    logic [31:0] n_idx;
    logic [31:0] rf_idx;
    logic [31:0] ns_idx;

    logic signed [W-1:0] opt_re;
    logic signed [W-1:0] opt_im;
    logic signed [W-1:0] rf_re;
    logic signed [W-1:0] rf_im;
    logic signed [(2*W)-1:0] prod_re;
    logic signed [(2*W)-1:0] prod_im;
    logic signed [ACC_W-1:0] acc_re;
    logic signed [ACC_W-1:0] acc_im;
    logic signed [ACC_W-1:0] next_acc_re;
    logic signed [ACC_W-1:0] next_acc_im;

    // Convert the accumulated matched-filter output back to W bits.
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

            if (TOTAL_SHIFT == 0)
                rounded = value;
            else if (value[ACC_W-1])
                rounded = value - ({{(ACC_W-1){1'b0}}, 1'b1} << (TOTAL_SHIFT - 1));
            else
                rounded = value + ({{(ACC_W-1){1'b0}}, 1'b1} << (TOTAL_SHIFT - 1));

            shifted = rounded >>> TOTAL_SHIFT;

            if (shifted > ext_max)
                sat_round = out_max;
            else if (shifted < ext_min)
                sat_round = out_min;
            else
                sat_round = shifted[W-1:0];
        end
    endfunction

    // Select the current row/column term and form one complex product.
    always @* begin
        opt_re = f_opt_re_flat[((n_idx * NS) + ns_idx)*W +: W];
        opt_im = f_opt_im_flat[((n_idx * NS) + ns_idx)*W +: W];
        rf_re  = f_rf_re_flat[((n_idx * NRF) + rf_idx)*W +: W];
        rf_im  = f_rf_im_flat[((n_idx * NRF) + rf_idx)*W +: W];

        // Complex product: conj(F_RF) * F_opt.
        prod_re = (rf_re * opt_re) + (rf_im * opt_im);
        prod_im = (rf_re * opt_im) - (rf_im * opt_re);
    end

    assign next_acc_re = acc_re + {{(ACC_W-(2*W)){prod_re[(2*W)-1]}}, prod_re};
    assign next_acc_im = acc_im + {{(ACC_W-(2*W)){prod_im[(2*W)-1]}}, prod_im};

    // Sequential scheduler computes one F_BB element at a time. It walks all
    // transmit antennas for the current RF-chain/data-stream pair, then commits
    // the rounded result.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= ST_IDLE;
            n_idx        <= 32'd0;
            rf_idx       <= 32'd0;
            ns_idx       <= 32'd0;
            acc_re       <= '0;
            acc_im       <= '0;
            f_bb_re_flat <= '0;
            f_bb_im_flat <= '0;
            done         <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (start) begin
                        state        <= ST_RUN;
                        n_idx        <= 32'd0;
                        rf_idx       <= 32'd0;
                        ns_idx       <= 32'd0;
                        acc_re       <= '0;
                        acc_im       <= '0;
                        f_bb_re_flat <= '0;
                        f_bb_im_flat <= '0;
                    end
                end

                ST_RUN: begin
                    if (n_idx + 32'd1 >= NT[31:0]) begin
                        // Completed the NT-term reduction for one F_BB entry.
                        f_bb_re_flat[((rf_idx * NS) + ns_idx)*W +: W] <= sat_round(next_acc_re);
                        f_bb_im_flat[((rf_idx * NS) + ns_idx)*W +: W] <= sat_round(next_acc_im);
                        acc_re <= '0;
                        acc_im <= '0;
                        n_idx  <= 32'd0;

                        if (ns_idx + 32'd1 < NS[31:0]) begin
                            ns_idx <= ns_idx + 32'd1;
                        end else begin
                            ns_idx <= 32'd0;
                            if (rf_idx + 32'd1 < NRF[31:0]) begin
                                rf_idx <= rf_idx + 32'd1;
                            end else begin
                                state <= ST_IDLE;
                                done  <= 1'b1;
                            end
                        end
                    end else begin
                        // Accumulate the next antenna contribution.
                        acc_re <= next_acc_re;
                        acc_im <= next_acc_im;
                        n_idx  <= n_idx + 32'd1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
