`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// hybrid_beamforming_arch_top.sv
//
// Concise RTL architecture shell for the proposed hybrid beamforming pipeline.
// This is an integration skeleton, not the final numeric implementation.
//
// Coefficient update:
//   SRS/LS -> DFT-MMSE -> Partial SVD -> PE-AltMin -> F_BB solve -> coeff RAM
//
// Runtime data:
//   s -> F_BB apply -> F_RF apply -> antenna samples
// -----------------------------------------------------------------------------

module hybrid_beamforming_arch_top #(
    parameter int unsigned NT   = 64,
    parameter int unsigned NR   = 16,
    parameter int unsigned NS   = 8,
    parameter int unsigned NRF  = 8,
    parameter int unsigned W    = 24,
    parameter int unsigned FRAC = 14
)(
    input  logic clk,
    input  logic rst,

    input  logic update_start,
    input  logic data_valid,

    input  logic [NR*NT*W-1:0] srs_y_re_flat,
    input  logic [NR*NT*W-1:0] srs_y_im_flat,

    input  logic [NS*W-1:0] data_s_re_flat,
    input  logic [NS*W-1:0] data_s_im_flat,

    output logic [NT*W-1:0] ant_x_re_flat,
    output logic [NT*W-1:0] ant_x_im_flat,

    output logic update_busy,
    output logic update_done
);

    typedef enum logic [2:0] {
        ST_IDLE,
        ST_LS,
        ST_MMSE,
        ST_SVD,
        ST_PE,
        ST_FBB,
        ST_WRITE,
        ST_DONE
    } state_t;

    state_t state;

    logic ls_start, mmse_start, svd_start, pe_start, fbb_start, ram_we;
    logic ls_done, mmse_done, svd_done, pe_done, fbb_done;
    logic coeff_valid;

    logic [NR*NT*W-1:0] h_ls_re_flat,  h_ls_im_flat;
    logic [NR*NT*W-1:0] h_est_re_flat, h_est_im_flat;
    logic [NT*NS*W-1:0] f_opt_re_flat, f_opt_im_flat;
    logic [NT*NRF*W-1:0] f_rf_re_flat, f_rf_im_flat;
    logic [NRF*NS*W-1:0] f_bb_re_flat, f_bb_im_flat;

    logic [NT*NRF*W-1:0] f_rf_active_re_flat, f_rf_active_im_flat;
    logic [NRF*NS*W-1:0] f_bb_active_re_flat, f_bb_active_im_flat;
    logic [NRF*W-1:0] bb_u_re_flat, bb_u_im_flat;

    // -------------------------------------------------------------------------
    // Coefficient update path
    // -------------------------------------------------------------------------

    srs_ls_estimator #(.NT(NT), .NR(NR), .W(W)) u_ls (
        .clk(clk), .rst(rst), .start(ls_start),
        .y_re_flat(srs_y_re_flat), .y_im_flat(srs_y_im_flat),
        .h_ls_re_flat(h_ls_re_flat), .h_ls_im_flat(h_ls_im_flat),
        .done(ls_done)
    );

    dft_mmse_estimator #(.NT(NT), .NR(NR), .W(W)) u_mmse (
        .clk(clk), .rst(rst), .start(mmse_start),
        .h_ls_re_flat(h_ls_re_flat), .h_ls_im_flat(h_ls_im_flat),
        .h_est_re_flat(h_est_re_flat), .h_est_im_flat(h_est_im_flat),
        .done(mmse_done)
    );

    partial_svd_engine #(.NT(NT), .NR(NR), .NS(NS), .W(W)) u_svd (
        .clk(clk), .rst(rst), .start(svd_start),
        .h_est_re_flat(h_est_re_flat), .h_est_im_flat(h_est_im_flat),
        .f_opt_re_flat(f_opt_re_flat), .f_opt_im_flat(f_opt_im_flat),
        .done(svd_done)
    );

    pe_altmin_engine #(.NT(NT), .NS(NS), .NRF(NRF), .W(W), .FRAC(FRAC)) u_pe (
        .clk(clk), .rst(rst), .start(pe_start), .warm_start(coeff_valid),
        .f_opt_re_flat(f_opt_re_flat), .f_opt_im_flat(f_opt_im_flat),
        .f_rf_prev_re_flat(f_rf_active_re_flat), .f_rf_prev_im_flat(f_rf_active_im_flat),
        .f_rf_re_flat(f_rf_re_flat), .f_rf_im_flat(f_rf_im_flat),
        .done(pe_done)
    );

    fbb_solver #(.NT(NT), .NS(NS), .NRF(NRF), .W(W)) u_fbb (
        .clk(clk), .rst(rst), .start(fbb_start),
        .f_opt_re_flat(f_opt_re_flat), .f_opt_im_flat(f_opt_im_flat),
        .f_rf_re_flat(f_rf_re_flat), .f_rf_im_flat(f_rf_im_flat),
        .f_bb_re_flat(f_bb_re_flat), .f_bb_im_flat(f_bb_im_flat),
        .done(fbb_done)
    );

    coeff_ram #(.NT(NT), .NS(NS), .NRF(NRF), .W(W)) u_coeff (
        .clk(clk), .rst(rst), .we(ram_we),
        .f_rf_wr_re_flat(f_rf_re_flat), .f_rf_wr_im_flat(f_rf_im_flat),
        .f_bb_wr_re_flat(f_bb_re_flat), .f_bb_wr_im_flat(f_bb_im_flat),
        .f_rf_rd_re_flat(f_rf_active_re_flat), .f_rf_rd_im_flat(f_rf_active_im_flat),
        .f_bb_rd_re_flat(f_bb_active_re_flat), .f_bb_rd_im_flat(f_bb_active_im_flat)
    );

    // -------------------------------------------------------------------------
    // Runtime data path
    // -------------------------------------------------------------------------

    digital_precoder_apply #(.NS(NS), .NRF(NRF), .W(W)) u_dig_apply (
        .valid(data_valid),
        .s_re_flat(data_s_re_flat), .s_im_flat(data_s_im_flat),
        .f_bb_re_flat(f_bb_active_re_flat), .f_bb_im_flat(f_bb_active_im_flat),
        .u_re_flat(bb_u_re_flat), .u_im_flat(bb_u_im_flat)
    );

    analog_precoder_apply #(.NT(NT), .NRF(NRF), .W(W)) u_rf_apply (
        .valid(data_valid),
        .u_re_flat(bb_u_re_flat), .u_im_flat(bb_u_im_flat),
        .f_rf_re_flat(f_rf_active_re_flat), .f_rf_im_flat(f_rf_active_im_flat),
        .x_re_flat(ant_x_re_flat), .x_im_flat(ant_x_im_flat)
    );

    // -------------------------------------------------------------------------
    // Coefficient Update FSM
    // -------------------------------------------------------------------------

    //Asynchronous reset and synchronous state transition
    always_ff @(posedge clk or posedge rst) begin 
        if (rst) begin
            state       <= ST_IDLE;
            update_busy <= 1'b0;
            update_done <= 1'b0;
            ls_start    <= 1'b0;
            mmse_start  <= 1'b0;
            svd_start   <= 1'b0;
            pe_start    <= 1'b0;
            fbb_start   <= 1'b0;
            ram_we      <= 1'b0;
            coeff_valid <= 1'b0;
        end else begin
            update_done <= 1'b0;
            ls_start    <= 1'b0;
            mmse_start  <= 1'b0;
            svd_start   <= 1'b0;
            pe_start    <= 1'b0;
            fbb_start   <= 1'b0;
            ram_we      <= 1'b0;

            case (state)
                ST_IDLE: begin
                    update_busy <= 1'b0;
                    if (update_start) begin
                        update_busy <= 1'b1;
                        ls_start    <= 1'b1;
                        state       <= ST_LS;
                    end
                end

                ST_LS:    if (ls_done)    begin mmse_start <= 1'b1; state <= ST_MMSE; end
                ST_MMSE:  if (mmse_done)  begin svd_start  <= 1'b1; state <= ST_SVD;  end
                ST_SVD:   if (svd_done)   begin pe_start   <= 1'b1; state <= ST_PE;   end
                ST_PE:    if (pe_done)    begin fbb_start  <= 1'b1; state <= ST_FBB;  end
                ST_FBB:   if (fbb_done)   begin ram_we <= 1'b1; coeff_valid <= 1'b1; state <= ST_WRITE; end
                ST_WRITE: begin state <= ST_DONE; end

                ST_DONE: begin
                    update_busy <= 1'b0;
                    update_done <= 1'b1;
                    state       <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
