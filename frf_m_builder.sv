`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// frf_m_builder.sv
//
// Radio Frequency Precoder M Matrix Builder.
// This block computes M = F_opt * F_DD^H after the Procrustes stage completes.
// It receives F_opt and F_DD, then produces M for the phase projection engine.
// The output M matrix is projected onto the unit circle to form the next F_RF.
// -----------------------------------------------------------------------------

module frf_m_builder #(
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
    input  logic [NRF*NS*W-1:0] f_dd_re_flat,
    input  logic [NRF*NS*W-1:0] f_dd_im_flat,
    output logic [NT*NRF*W-1:0] m_re_flat,
    output logic [NT*NRF*W-1:0] m_im_flat,
    output logic busy,
    output logic done
);

    localparam int unsigned ACC_W = (2*W) + $clog2(NS + 1) + 2;

    typedef enum logic [1:0] {
        ST_IDLE,
        ST_RUN,
        ST_DRAIN
    } state_t;

    state_t state;

    logic [31:0] s_base;
    logic [31:0] nt_idx;
    logic [31:0] rf_idx;
    logic [31:0] drain_count;

    logic signed [ACC_W-1:0] acc_re;
    logic signed [ACC_W-1:0] acc_im;
    logic signed [ACC_W-1:0] lane_sum_re;
    logic signed [ACC_W-1:0] lane_sum_im;
    logic signed [ACC_W-1:0] next_acc_re;
    logic signed [ACC_W-1:0] next_acc_im;

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

    always @* begin
        lane_sum_re = '0;
        lane_sum_im = '0;

        for (int unsigned lane = 0; lane < MAC_LANES; lane++) begin
            logic [31:0] s_idx;
            logic [31:0] f_opt_index;
            logic [31:0] f_dd_index;
            logic signed [W-1:0] opt_re;
            logic signed [W-1:0] opt_im;
            logic signed [W-1:0] dd_re;
            logic signed [W-1:0] dd_im;
            logic signed [(2*W)-1:0] prod_re;
            logic signed [(2*W)-1:0] prod_im;

            s_idx = s_base + lane[31:0];

            if ((state == ST_RUN) && (s_idx < NS)) begin
                f_opt_index = (nt_idx * NS) + s_idx;
                f_dd_index  = (rf_idx * NS) + s_idx;

                opt_re = f_opt_re_flat[f_opt_index*W +: W];
                opt_im = f_opt_im_flat[f_opt_index*W +: W];
                dd_re  = f_dd_re_flat[f_dd_index*W +: W];
                dd_im  = f_dd_im_flat[f_dd_index*W +: W];

                // F_opt * conj(F_DD)
                prod_re = (opt_re * dd_re) + (opt_im * dd_im);
                prod_im = (opt_im * dd_re) - (opt_re * dd_im);

                lane_sum_re += {{(ACC_W-(2*W)){prod_re[(2*W)-1]}}, prod_re};
                lane_sum_im += {{(ACC_W-(2*W)){prod_im[(2*W)-1]}}, prod_im};
            end
        end
    end

    assign next_acc_re = acc_re + lane_sum_re;
    assign next_acc_im = acc_im + lane_sum_im;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= ST_IDLE;
            s_base    <= 32'd0;
            nt_idx    <= 32'd0;
            rf_idx    <= 32'd0;
            drain_count <= 32'd0;
            acc_re    <= '0;
            acc_im    <= '0;
            m_re_flat <= '0;
            m_im_flat <= '0;
            busy      <= 1'b0;
            done      <= 1'b0;
        end else if (start) begin
            state     <= ST_RUN;
            s_base    <= 32'd0;
            nt_idx    <= 32'd0;
            rf_idx    <= 32'd0;
            drain_count <= 32'd0;
            acc_re    <= '0;
            acc_im    <= '0;
            m_re_flat <= '0;
            m_im_flat <= '0;
            busy      <= 1'b1;
            done      <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                end

                ST_RUN: begin
                    if (s_base + MAC_LANES[31:0] >= NS[31:0]) begin
                        m_re_flat[((nt_idx * NRF) + rf_idx)*W +: W] <= sat_round(next_acc_re);
                        m_im_flat[((nt_idx * NRF) + rf_idx)*W +: W] <= sat_round(next_acc_im);
                        acc_re <= '0;
                        acc_im <= '0;
                        s_base <= 32'd0;

                        if (rf_idx + 32'd1 < NRF[31:0]) begin
                            rf_idx <= rf_idx + 32'd1;
                        end else begin
                            rf_idx <= 32'd0;
                            if (nt_idx + 32'd1 < NT[31:0]) begin
                                nt_idx <= nt_idx + 32'd1;
                            end else begin
                                state <= ST_DRAIN;
                            end
                        end
                    end else begin
                        acc_re <= next_acc_re;
                        acc_im <= next_acc_im;
                        s_base <= s_base + MAC_LANES[31:0];
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
