`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// pe_altmin_frf_engine.sv
//
// Cycle-countable RTL architecture for the PE-AltMin / F_RF computation engine.
//
// One PE-AltMin iteration is scheduled as:
//
//   1. A build       : A = F_opt^H * F_RF        (8 x 8 reduction over Nt)
//   2. SVD/Procrustes: F_DD = V * U^H            (small 8 x 8 engine)
//   3. M build       : M = F_opt * F_DD^H        (Nt x 8 matrix product)
//   4. Phase extract : F_RF = exp(j * angle(M))  (CORDIC/LUT lanes)
//   5. Check         : objective / convergence
//
// This file contains the controller and wires together the named datapath
// blocks. The sub-blocks currently expose realistic handshakes and latency
// knobs; their numeric MAC/SVD/CORDIC implementations can be filled in next.
// -----------------------------------------------------------------------------

module pe_altmin_frf_engine #(
    parameter int unsigned NT           = 1024,
    parameter int unsigned NS           = 8,
    parameter int unsigned NRF          = 8,
    parameter int unsigned W            = 24,
    parameter int unsigned FRAC         = W/2,

    // Architecture knobs.
    parameter int unsigned MAC_LANES    = 8,
    parameter int unsigned CORDIC_LANES = 8,

    // Pipeline overheads for the eventual datapaths.
    parameter int unsigned MAC_PIPE     = 8,
    parameter int unsigned SVD8_LAT     = 300,
    parameter int unsigned CORDIC_LAT   = 24,
    parameter int unsigned CHECK_LAT    = 32,

    // Cold/warm iteration targets. Later, convergence logic can stop earlier.
    parameter int unsigned COLD_ITERS   = 20,
    parameter int unsigned WARM_ITERS   = 3
)(
    input  logic clk,
    input  logic rst,
    input  logic start,
    input  logic warm_start,

    input  logic [NT*NS*W-1:0]  f_opt_re_flat,
    input  logic [NT*NS*W-1:0]  f_opt_im_flat,
    input  logic [NT*NRF*W-1:0] f_rf_prev_re_flat,
    input  logic [NT*NRF*W-1:0] f_rf_prev_im_flat,

    output logic [NT*NRF*W-1:0] f_rf_re_flat,
    output logic [NT*NRF*W-1:0] f_rf_im_flat,

    output logic [7:0]  iter_count,
    output logic [31:0] cycle_count,
    output logic [2:0]  stage_id,
    output logic        busy,
    output logic        done
);

    typedef enum logic [2:0] {
        ST_IDLE,
        ST_A,
        ST_SVD,
        ST_M,
        ST_PHASE,
        ST_CHECK,
        ST_DONE
    } state_t;

    state_t state;

    logic start_a, start_svd, start_m, start_phase, start_check;
    logic a_done, svd_done, m_done, phase_done, check_done;
    logic converged;
    logic [7:0] iter_target;

    logic [NT*NRF*W-1:0] f_rf_work_re_flat;
    logic [NT*NRF*W-1:0] f_rf_work_im_flat;
    logic [NT*NRF*W-1:0] f_rf_next_re_flat;
    logic [NT*NRF*W-1:0] f_rf_next_im_flat;

    logic [NS*NRF*W-1:0] a_re_flat;
    logic [NS*NRF*W-1:0] a_im_flat;
    logic [NRF*NS*W-1:0] f_dd_re_flat;
    logic [NRF*NS*W-1:0] f_dd_im_flat;
    logic [NT*NRF*W-1:0] m_re_flat;
    logic [NT*NRF*W-1:0] m_im_flat;

    frf_a_builder #(
        .NT(NT), .NS(NS), .NRF(NRF), .W(W),
        .FRAC(FRAC),
        .MAC_LANES(MAC_LANES), .MAC_PIPE(MAC_PIPE)
    ) u_a_builder (
        .clk(clk), .rst(rst), .start(start_a),
        .f_opt_re_flat(f_opt_re_flat), .f_opt_im_flat(f_opt_im_flat),
        .f_rf_re_flat(f_rf_work_re_flat), .f_rf_im_flat(f_rf_work_im_flat),
        .a_re_flat(a_re_flat), .a_im_flat(a_im_flat),
        .busy(), .done(a_done)
    );

    svd8_procrustes_stage #(
        .NS(NS), .NRF(NRF), .W(W), .SVD8_LAT(SVD8_LAT)
    ) u_svd8 (
        .clk(clk), .rst(rst), .start(start_svd),
        .a_re_flat(a_re_flat), .a_im_flat(a_im_flat),
        .f_dd_re_flat(f_dd_re_flat), .f_dd_im_flat(f_dd_im_flat),
        .busy(), .done(svd_done)
    );

    frf_m_builder #(
        .NT(NT), .NS(NS), .NRF(NRF), .W(W),
        .FRAC(FRAC),
        .MAC_LANES(MAC_LANES), .MAC_PIPE(MAC_PIPE)
    ) u_m_builder (
        .clk(clk), .rst(rst), .start(start_m),
        .f_opt_re_flat(f_opt_re_flat), .f_opt_im_flat(f_opt_im_flat),
        .f_dd_re_flat(f_dd_re_flat), .f_dd_im_flat(f_dd_im_flat),
        .m_re_flat(m_re_flat), .m_im_flat(m_im_flat),
        .busy(), .done(m_done)
    );

    frf_phase_projector #(
        .NT(NT), .NRF(NRF), .W(W),
        .FRAC(FRAC),
        .CORDIC_LANES(CORDIC_LANES), .CORDIC_LAT(CORDIC_LAT)
    ) u_phase (
        .clk(clk), .rst(rst), .start(start_phase),
        .m_re_flat(m_re_flat), .m_im_flat(m_im_flat),
        .f_rf_re_flat(f_rf_next_re_flat), .f_rf_im_flat(f_rf_next_im_flat),
        .busy(), .done(phase_done)
    );

    frf_convergence_check #(
        .CHECK_LAT(CHECK_LAT)
    ) u_check (
        .clk(clk), .rst(rst), .start(start_check),
        .iter_count(iter_count), .iter_target(iter_target),
        .converged(converged),
        .busy(), .done(check_done)
    );

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= ST_IDLE;
            stage_id     <= ST_IDLE;
            iter_target  <= 8'd0;
            iter_count   <= 8'd0;
            cycle_count  <= 32'd0;
            start_a      <= 1'b0;
            start_svd    <= 1'b0;
            start_m      <= 1'b0;
            start_phase  <= 1'b0;
            start_check  <= 1'b0;
            f_rf_work_re_flat <= {NT*NRF*W{1'b0}};
            f_rf_work_im_flat <= {NT*NRF*W{1'b0}};
            f_rf_re_flat <= {NT*NRF*W{1'b0}};
            f_rf_im_flat <= {NT*NRF*W{1'b0}};
            busy         <= 1'b0;
            done         <= 1'b0;
        end else begin
            done        <= 1'b0;
            start_a     <= 1'b0;
            start_svd   <= 1'b0;
            start_m     <= 1'b0;
            start_phase <= 1'b0;
            start_check <= 1'b0;

            if (busy)
                cycle_count <= cycle_count + 32'd1;

            case (state)
                ST_IDLE: begin
                    busy        <= 1'b0;
                    stage_id    <= ST_IDLE;
                    if (start) begin
                        busy        <= 1'b1;
                        cycle_count <= 32'd0;
                        iter_count  <= 8'd0;
                        iter_target <= warm_start ? WARM_ITERS[7:0] : COLD_ITERS[7:0];

                        // Warm start reuses the previous analog precoder.
                        // Cold start seed generation will replace this later.
                        f_rf_work_re_flat <= f_rf_prev_re_flat;
                        f_rf_work_im_flat <= f_rf_prev_im_flat;
                        f_rf_re_flat      <= f_rf_prev_re_flat;
                        f_rf_im_flat      <= f_rf_prev_im_flat;

                        start_a  <= 1'b1;
                        state    <= ST_A;
                        stage_id <= ST_A;
                    end
                end

                ST_A: begin
                    if (a_done) begin
                        start_svd <= 1'b1;
                        state     <= ST_SVD;
                        stage_id  <= ST_SVD;
                    end
                end

                ST_SVD: begin
                    if (svd_done) begin
                        start_m  <= 1'b1;
                        state    <= ST_M;
                        stage_id <= ST_M;
                    end
                end

                ST_M: begin
                    if (m_done) begin
                        start_phase <= 1'b1;
                        state       <= ST_PHASE;
                        stage_id    <= ST_PHASE;
                    end
                end

                ST_PHASE: begin
                    if (phase_done) begin
                        f_rf_work_re_flat <= f_rf_next_re_flat;
                        f_rf_work_im_flat <= f_rf_next_im_flat;
                        f_rf_re_flat      <= f_rf_next_re_flat;
                        f_rf_im_flat      <= f_rf_next_im_flat;
                        start_check       <= 1'b1;
                        state             <= ST_CHECK;
                        stage_id          <= ST_CHECK;
                    end
                end

                ST_CHECK: begin
                    if (check_done) begin
                        iter_count <= iter_count + 8'd1;
                        if (converged) begin
                            state    <= ST_DONE;
                            stage_id <= ST_DONE;
                        end else begin
                            start_a  <= 1'b1;
                            state    <= ST_A;
                            stage_id <= ST_A;
                        end
                    end
                end

                ST_DONE: begin
                    busy     <= 1'b0;
                    done     <= 1'b1;
                    stage_id <= ST_IDLE;
                    state    <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
