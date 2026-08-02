`timescale 1ns/1ps

module tb_complex_1w2r_ram;

    // Use smaller values than the DUT defaults so the first simulation is
    // easy to read. The RAM behavior is unchanged.
    localparam int unsigned DEPTH  = 8;
    localparam int unsigned W      = 8;
    localparam int unsigned ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH);

    // Testbench signals connected to the DUT ports.
    logic clk;
    logic wr_en;
    logic [ADDR_W-1:0] wr_addr;
    logic signed [W-1:0] wr_re;
    logic signed [W-1:0] wr_im;
    logic [ADDR_W-1:0] rd0_addr;
    logic [ADDR_W-1:0] rd1_addr;
    logic signed [W-1:0] rd0_re;
    logic signed [W-1:0] rd0_im;
    logic signed [W-1:0] rd1_re;
    logic signed [W-1:0] rd1_im;

    integer error_count;

    // Device under test.
    complex_1w2r_ram #(
        .DEPTH  (DEPTH),
        .W      (W),
        .ADDR_W (ADDR_W)
    ) dut (
        .clk      (clk),
        .wr_en    (wr_en),
        .wr_addr  (wr_addr),
        .wr_re    (wr_re),
        .wr_im    (wr_im),
        .rd0_addr (rd0_addr),
        .rd1_addr (rd1_addr),
        .rd0_re   (rd0_re),
        .rd0_im   (rd0_im),
        .rd1_re   (rd1_re),
        .rd1_im   (rd1_im)
    );

    // 10 ns clock period: low for 5 ns, high for 5 ns.
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // Drive a write before a rising edge. The DUT stores the values at the
    // next rising edge because its write logic is synchronous.
    task automatic write_complex(
        input logic [ADDR_W-1:0] addr,
        input logic signed [W-1:0] real_value,
        input logic signed [W-1:0] imag_value
    );
        begin
            @(negedge clk);
            wr_en   = 1'b1;
            wr_addr = addr;
            wr_re   = real_value;
            wr_im   = imag_value;

            @(posedge clk);
            #1;
            wr_en = 1'b0;
        end
    endtask

    // Check read port 0. Case inequality (!==) also reports failure if an
    // unexpected X or Z appears.
    task automatic check_port0(
        input logic [ADDR_W-1:0] addr,
        input logic signed [W-1:0] expected_re,
        input logic signed [W-1:0] expected_im,
        input string test_name
    );
        begin
            rd0_addr = addr;
            #1;

            if ((rd0_re !== expected_re) || (rd0_im !== expected_im)) begin
                error_count = error_count + 1;
                $display("FAIL: %s | port0 addr=%0d expected=(%0d,%0d) got=(%0d,%0d)",
                         test_name, addr,
                         $signed(expected_re), $signed(expected_im),
                         $signed(rd0_re), $signed(rd0_im));
            end
            else begin
                $display("PASS: %s | port0 addr=%0d value=(%0d,%0d)",
                         test_name, addr,
                         $signed(rd0_re), $signed(rd0_im));
            end
        end
    endtask

    // Check read port 1.
    task automatic check_port1(
        input logic [ADDR_W-1:0] addr,
        input logic signed [W-1:0] expected_re,
        input logic signed [W-1:0] expected_im,
        input string test_name
    );
        begin
            rd1_addr = addr;
            #1;

            if ((rd1_re !== expected_re) || (rd1_im !== expected_im)) begin
                error_count = error_count + 1;
                $display("FAIL: %s | port1 addr=%0d expected=(%0d,%0d) got=(%0d,%0d)",
                         test_name, addr,
                         $signed(expected_re), $signed(expected_im),
                         $signed(rd1_re), $signed(rd1_im));
            end
            else begin
                $display("PASS: %s | port1 addr=%0d value=(%0d,%0d)",
                         test_name, addr,
                         $signed(rd1_re), $signed(rd1_im));
            end
        end
    endtask

    initial begin
        // Known starting values for all testbench-driven inputs.
        error_count = 0;
        wr_en       = 1'b0;
        wr_addr     = '0;
        wr_re       = '0;
        wr_im       = '0;
        rd0_addr    = '0;
        rd1_addr    = '0;

        $display("Starting complex_1w2r_ram verification...");

        // Do not check unwritten memory because the DUT has no reset or
        // initialization and unwritten locations may contain X values.

        // Test 1: write one positive/negative complex value.
        write_complex(3'd2, 8'sd12, -8'sd5);
        check_port0(3'd2, 8'sd12, -8'sd5, "first write is readable");

        // Test 2: the second read port sees the same stored value.
        check_port1(3'd2, 8'sd12, -8'sd5, "same address through port1");

        // Test 3: write a second address with the opposite sign pattern.
        write_complex(3'd5, -8'sd20, 8'sd37);

        // Test 4: read two different addresses at the same time.
        rd0_addr = 3'd2;
        rd1_addr = 3'd5;
        #1;
        check_port0(3'd2, 8'sd12, -8'sd5, "simultaneous read, first address");
        check_port1(3'd5, -8'sd20, 8'sd37, "simultaneous read, second address");

        // Test 5: changing write data while wr_en=0 must not modify memory.
        @(negedge clk);
        wr_en   = 1'b0;
        wr_addr = 3'd2;
        wr_re   = 8'sd99;
        wr_im   = -8'sd99;
        @(posedge clk);
        #1;
        check_port0(3'd2, 8'sd12, -8'sd5, "wr_en=0 prevents a write");

        // Test 6: overwrite an existing address.
        write_complex(3'd2, -8'sd64, 8'sd63);
        check_port0(3'd2, -8'sd64, 8'sd63, "overwrite returns new value");
        check_port1(3'd2, -8'sd64, 8'sd63, "overwrite visible on port1");

        // Test 7: signed 8-bit boundary values.
        write_complex(3'd7, 8'sh80, 8'sh7f);
        check_port0(3'd7, 8'sh80, 8'sh7f, "signed boundary values");

        $display("");
        if (error_count == 0) begin
            $display("============================================");
            $display("PASS: all complex_1w2r_ram tests completed.");
            $display("============================================");
        end
        else begin
            $display("============================================");
            $display("FAIL: %0d test check(s) failed.", error_count);
            $display("============================================");
        end

        $finish;
    end

endmodule
