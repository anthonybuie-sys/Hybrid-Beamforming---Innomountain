`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// pe_altmin_frf_mem_core.sv
//
// Memory-backed PE-AltMin Radio Frequency Precoder Core.
//
// Interaction:
//   - Connects to pe_altmin_matrix_memory through explicit read/write ports.
//   - Sequences the memory-backed A builder, Procrustes baseline, M builder,
//     phase projector, and convergence checker.
//   - Leaves matrix loading and final coefficient export to a higher wrapper.
//
// This core is the functional-first replacement path for the older flattened-bus
// PE-AltMin engine. It is intentionally conservative and sequential so the
// algorithmic data movement is easy to inspect before latency optimization.
//
// Port arbitration:
//   - F_opt read address is shared by A-build and M-build stages.
//   - F_RF read address is owned by A-build.
//   - A, F_DD, M, and F_RF write ports are owned by their corresponding stages.
// -----------------------------------------------------------------------------

module pe_altmin_frf_mem_core #(
    parameter int unsigned NT           = 1024,
    parameter int unsigned NS           = 8,
    parameter int unsigned NRF          = 8,
    parameter int unsigned W            = 24,
    parameter int unsigned FRAC         = W/2,
    parameter int unsigned SVD8_LAT     = 300,
    parameter int unsigned CORDIC_LAT   = 24,
    parameter int unsigned CHECK_LAT    = 32,
    parameter int unsigned COLD_ITERS   = 20,
    parameter int unsigned WARM_ITERS   = 3,
    parameter int unsigned F_OPT_AW     = (NT*NS <= 1) ? 1 : $clog2(NT*NS),
    parameter int unsigned F_RF_AW      = (NT*NRF <= 1) ? 1 : $clog2(NT*NRF),
    parameter int unsigned A_AW         = (NS*NRF <= 1) ? 1 : $clog2(NS*NRF),
    parameter int unsigned F_DD_AW      = (NRF*NS <= 1) ? 1 : $clog2(NRF*NS),
    parameter int unsigned M_AW         = (NT*NRF <= 1) ? 1 : $clog2(NT*NRF)
)(
    input  logic clk,
    input  logic rst,
    input  logic start,
    input  logic warm_start,

    output logic [F_OPT_AW-1:0] f_opt_rd_addr,
    input  logic signed [W-1:0] f_opt_rd_re,
    input  logic signed [W-1:0] f_opt_rd_im,

    output logic [F_RF_AW-1:0] f_rf_rd_addr,
    input  logic signed [W-1:0] f_rf_rd_re,
    input  logic signed [W-1:0] f_rf_rd_im,
    output logic f_rf_we,
    output logic [F_RF_AW-1:0] f_rf_wr_addr,
    output logic signed [W-1:0] f_rf_wr_re,
    output logic signed [W-1:0] f_rf_wr_im,

    output logic a_we,
    output logic [A_AW-1:0] a_wr_addr,
    output logic signed [W-1:0] a_wr_re,
    output logic signed [W-1:0] a_wr_im,
    output logic [A_AW-1:0] a_rd_addr,
    input  logic signed [W-1:0] a_rd_re,
    input  logic signed [W-1:0] a_rd_im,

    output logic f_dd_we,
    output logic [F_DD_AW-1:0] f_dd_wr_addr,
    output logic signed [W-1:0] f_dd_wr_re,
    output logic signed [W-1:0] f_dd_wr_im,
    output logic [F_DD_AW-1:0] f_dd_rd_addr,
    input  logic signed [W-1:0] f_dd_rd_re,
    input  logic signed [W-1:0] f_dd_rd_im,

    output logic m_we,
    output logic [M_AW-1:0] m_wr_addr,
    output logic signed [W-1:0] m_wr_re,
    output logic signed [W-1:0] m_wr_im,
    output logic [M_AW-1:0] m_rd_addr,
    input  logic signed [W-1:0] m_rd_re,
    input  logic signed [W-1:0] m_rd_im,

    output logic [7:0]  iter_count,
    output logic [31:0] cycle_count,
    output logic [2:0]  stage_id,
    output logic busy,
    output logic done
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

    // One-cycle start strobes for memory-backed sub-stages.
    logic start_a;
    logic start_svd;
    logic start_m;
    logic start_phase;
    logic start_check;

    logic a_done;
    logic svd_done;
    logic m_done;
    logic phase_done;
    logic check_done;
    logic converged;
    logic [7:0] iter_target;

    // Internal stage-local memory addresses and write buses. The assigns below
    // multiplex these onto the external memory interface based on active stage.
    logic [F_OPT_AW-1:0] a_f_opt_rd_addr;
    logic [F_RF_AW-1:0]  a_f_rf_rd_addr;
    logic a_we_i;
    logic [A_AW-1:0] a_wr_addr_i;
    logic signed [W-1:0] a_wr_re_i;
    logic signed [W-1:0] a_wr_im_i;

    logic [A_AW-1:0] svd_a_rd_addr;
    logic f_dd_we_i;
    logic [F_DD_AW-1:0] f_dd_wr_addr_i;
    logic signed [W-1:0] f_dd_wr_re_i;
    logic signed [W-1:0] f_dd_wr_im_i;

    logic [F_OPT_AW-1:0] m_f_opt_rd_addr;
    logic [F_DD_AW-1:0]  m_f_dd_rd_addr;
    logic m_we_i;
    logic [M_AW-1:0] m_wr_addr_i;
    logic signed [W-1:0] m_wr_re_i;
    logic signed [W-1:0] m_wr_im_i;

    logic [M_AW-1:0] phase_m_rd_addr;
    logic f_rf_we_i;
    logic [F_RF_AW-1:0] f_rf_wr_addr_i;
    logic signed [W-1:0] f_rf_wr_re_i;
    logic signed [W-1:0] f_rf_wr_im_i;

    // F_opt is read by A-build and M-build at different times, so one external
    // read port can be shared in the functional baseline.
    assign f_opt_rd_addr = (state == ST_A) ? a_f_opt_rd_addr : m_f_opt_rd_addr;
    assign f_rf_rd_addr  = a_f_rf_rd_addr;

    assign a_we      = a_we_i;
    assign a_wr_addr = a_wr_addr_i;
    assign a_wr_re   = a_wr_re_i;
    assign a_wr_im   = a_wr_im_i;
    assign a_rd_addr = svd_a_rd_addr;

    assign f_dd_we      = f_dd_we_i;
    assign f_dd_wr_addr = f_dd_wr_addr_i;
    assign f_dd_wr_re   = f_dd_wr_re_i;
    assign f_dd_wr_im   = f_dd_wr_im_i;
    assign f_dd_rd_addr = m_f_dd_rd_addr;

    assign m_we      = m_we_i;
    assign m_wr_addr = m_wr_addr_i;
    assign m_wr_re   = m_wr_re_i;
    assign m_wr_im   = m_wr_im_i;
    assign m_rd_addr = phase_m_rd_addr;

    assign f_rf_we      = f_rf_we_i;
    assign f_rf_wr_addr = f_rf_wr_addr_i;
    assign f_rf_wr_re   = f_rf_wr_re_i;
    assign f_rf_wr_im   = f_rf_wr_im_i;

    // Stage 1: memory-backed A = F_opt^H * F_RF.
    frf_a_builder_mem #(
        .NT(NT), .NS(NS), .NRF(NRF), .W(W), .FRAC(FRAC),
        .F_OPT_AW(F_OPT_AW), .F_RF_AW(F_RF_AW), .A_AW(A_AW)
    ) u_a_builder_mem (
        .clk(clk), .rst(rst), .start(start_a),
        .f_opt_rd_addr(a_f_opt_rd_addr),
        .f_opt_rd_re(f_opt_rd_re), .f_opt_rd_im(f_opt_rd_im),
        .f_rf_rd_addr(a_f_rf_rd_addr),
        .f_rf_rd_re(f_rf_rd_re), .f_rf_rd_im(f_rf_rd_im),
        .a_we(a_we_i), .a_wr_addr(a_wr_addr_i),
        .a_wr_re(a_wr_re_i), .a_wr_im(a_wr_im_i),
        .busy(), .done(a_done)
    );

    // Stage 2: memory-backed Procrustes baseline writes F_DD.
    svd8_procrustes_stage_mem #(
        .NS(NS), .NRF(NRF), .W(W), .FRAC(FRAC), .SVD8_LAT(SVD8_LAT),
        .A_AW(A_AW), .F_DD_AW(F_DD_AW)
    ) u_svd_mem (
        .clk(clk), .rst(rst), .start(start_svd),
        .a_rd_addr(svd_a_rd_addr),
        .a_rd_re(a_rd_re), .a_rd_im(a_rd_im),
        .f_dd_we(f_dd_we_i), .f_dd_wr_addr(f_dd_wr_addr_i),
        .f_dd_wr_re(f_dd_wr_re_i), .f_dd_wr_im(f_dd_wr_im_i),
        .busy(), .done(svd_done)
    );

    // Stage 3: memory-backed M = F_opt * F_DD^H.
    frf_m_builder_mem #(
        .NT(NT), .NS(NS), .NRF(NRF), .W(W), .FRAC(FRAC),
        .F_OPT_AW(F_OPT_AW), .F_DD_AW(F_DD_AW), .M_AW(M_AW)
    ) u_m_builder_mem (
        .clk(clk), .rst(rst), .start(start_m),
        .f_opt_rd_addr(m_f_opt_rd_addr),
        .f_opt_rd_re(f_opt_rd_re), .f_opt_rd_im(f_opt_rd_im),
        .f_dd_rd_addr(m_f_dd_rd_addr),
        .f_dd_rd_re(f_dd_rd_re), .f_dd_rd_im(f_dd_rd_im),
        .m_we(m_we_i), .m_wr_addr(m_wr_addr_i),
        .m_wr_re(m_wr_re_i), .m_wr_im(m_wr_im_i),
        .busy(), .done(m_done)
    );

    // Stage 4: memory-backed phase projection writes updated F_RF.
    frf_phase_projector_mem #(
        .NT(NT), .NRF(NRF), .W(W), .FRAC(FRAC), .CORDIC_LAT(CORDIC_LAT),
        .M_AW(M_AW), .F_RF_AW(F_RF_AW)
    ) u_phase_mem (
        .clk(clk), .rst(rst), .start(start_phase),
        .m_rd_addr(phase_m_rd_addr),
        .m_rd_re(m_rd_re), .m_rd_im(m_rd_im),
        .f_rf_we(f_rf_we_i), .f_rf_wr_addr(f_rf_wr_addr_i),
        .f_rf_wr_re(f_rf_wr_re_i), .f_rf_wr_im(f_rf_wr_im_i),
        .busy(), .done(phase_done)
    );

    // Stage 5: iteration-count based convergence for the functional baseline.
    frf_convergence_check #(
        .CHECK_LAT(CHECK_LAT)
    ) u_check (
        .clk(clk), .rst(rst), .start(start_check),
        .iter_count(iter_count), .iter_target(iter_target),
        .converged(converged),
        .busy(), .done(check_done)
    );

    // Controller sequencing mirrors the flattened-bus PE-AltMin engine while
    // driving local memories instead of flattened intermediate matrices.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= ST_IDLE;
            stage_id    <= ST_IDLE;
            start_a     <= 1'b0;
            start_svd   <= 1'b0;
            start_m     <= 1'b0;
            start_phase <= 1'b0;
            start_check <= 1'b0;
            iter_target <= 8'd0;
            iter_count  <= 8'd0;
            cycle_count <= 32'd0;
            busy        <= 1'b0;
            done        <= 1'b0;
        end else begin
            // Default all sub-stage command pulses low. Each state below asserts
            // at most one start pulse when it advances the pipeline.
            start_a     <= 1'b0;
            start_svd   <= 1'b0;
            start_m     <= 1'b0;
            start_phase <= 1'b0;
            start_check <= 1'b0;
            done        <= 1'b0;

            if (busy)
                cycle_count <= cycle_count + 32'd1;

            case (state)
                ST_IDLE: begin
                    busy     <= 1'b0;
                    stage_id <= ST_IDLE;
                    if (start) begin
                        // Select cold-start or warm-start iteration budget and
                        // launch the first A-build stage.
                        busy        <= 1'b1;
                        cycle_count <= 32'd0;
                        iter_count  <= 8'd0;
                        iter_target <= warm_start ? WARM_ITERS[7:0] : COLD_ITERS[7:0];
                        start_a     <= 1'b1;
                        state       <= ST_A;
                        stage_id    <= ST_A;
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
                        start_check <= 1'b1;
                        state       <= ST_CHECK;
                        stage_id    <= ST_CHECK;
                    end
                end

                ST_CHECK: begin
                    if (check_done) begin
                        // Count completed iterations after the convergence
                        // stage evaluates the just-finished phase projection.
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
