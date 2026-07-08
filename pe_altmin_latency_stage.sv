`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// pe_altmin_latency_stage.sv
//
// Reusable start/busy/done latency stage.
// Placeholder blocks such as the Procrustes stage and convergence checker use this module to model a bounded operation latency.
// It accepts a single-cycle start pulse, holds busy while counting, and raises done for one cycle at completion.
// This is useful while functional datapaths are being filled in behind stable interfaces.
//
// Protocol:
//   - start is sampled only when busy is low.
//   - busy remains high while the internal counter is active.
//   - done is a one-cycle pulse at completion.
// -----------------------------------------------------------------------------

module pe_altmin_latency_stage #(
    parameter int unsigned LATENCY = 1
)(
    input  logic clk,
    input  logic rst,
    input  logic start,
    output logic busy,
    output logic done
);

    logic [31:0] count;

    // Generic latency counter. This module intentionally has no datapath; it
    // only models operation duration behind a stable start/busy/done handshake.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            count <= 32'd0;
            busy  <= 1'b0;
            done  <= 1'b0;
        end else begin
            done <= 1'b0;

            if (start && !busy) begin
                // LATENCY==1 completes immediately, which keeps single-cycle
                // placeholder stages easy to represent.
                count <= 32'd1;
                if (LATENCY <= 1) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end else begin
                    busy <= 1'b1;
                end
            end else if (busy) begin
                if (count + 32'd1 >= LATENCY[31:0]) begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    count <= 32'd0;
                end else begin
                    count <= count + 32'd1;
                end
            end
        end
    end

endmodule
