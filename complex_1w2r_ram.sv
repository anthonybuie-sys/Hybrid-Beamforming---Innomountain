`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// complex_1w2r_ram.sv
//
// Complex one-write two-read memory primitive.
// pe_altmin_matrix_memory instantiates this primitive once for each local complex matrix.
// The write port loads or updates a matrix element; the two read ports support simple functional datapath access.
// This is a correctness-first memory model and is not yet a final banked high-throughput memory.
// -----------------------------------------------------------------------------

module complex_1w2r_ram #(
    parameter int unsigned DEPTH  = 64,
    parameter int unsigned W      = 24,
    parameter int unsigned ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH)
)(
    input  logic clk,
    input  logic wr_en,
    input  logic [ADDR_W-1:0] wr_addr,
    input  logic signed [W-1:0] wr_re,
    input  logic signed [W-1:0] wr_im,
    input  logic [ADDR_W-1:0] rd0_addr,
    input  logic [ADDR_W-1:0] rd1_addr,
    output logic signed [W-1:0] rd0_re,
    output logic signed [W-1:0] rd0_im,
    output logic signed [W-1:0] rd1_re,
    output logic signed [W-1:0] rd1_im
);

    logic signed [W-1:0] mem_re [0:DEPTH-1];
    logic signed [W-1:0] mem_im [0:DEPTH-1];

    always_ff @(posedge clk) begin
        if (wr_en) begin
            mem_re[wr_addr] <= wr_re;
            mem_im[wr_addr] <= wr_im;
        end
    end

    assign rd0_re = mem_re[rd0_addr];
    assign rd0_im = mem_im[rd0_addr];
    assign rd1_re = mem_re[rd1_addr];
    assign rd1_im = mem_im[rd1_addr];

endmodule
