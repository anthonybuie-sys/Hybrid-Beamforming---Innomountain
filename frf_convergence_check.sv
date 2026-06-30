`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// frf_convergence_check.sv
//
// Radio Frequency Precoder Convergence and Iteration Check.
// This block tells the PE-AltMin controller whether another iteration is required.
// For the current functional baseline it compares the current iteration count against the configured target.
// A later objective-based convergence calculation can replace the internal logic without changing the controller handshake.
// -----------------------------------------------------------------------------

module frf_convergence_check #(
    parameter int unsigned CHECK_LAT = 32
)(
    input  logic clk,
    input  logic rst,
    input  logic start,
    input  logic [7:0] iter_count,
    input  logic [7:0] iter_target,
    output logic converged,
    output logic busy,
    output logic done
);

    pe_altmin_latency_stage #(.LATENCY(CHECK_LAT)) u_timer (
        .clk(clk), .rst(rst), .start(start), .busy(busy), .done(done)
    );

    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            converged <= 1'b0;
        else if (start)
            converged <= (iter_count + 8'd1 >= iter_target);
    end

endmodule
