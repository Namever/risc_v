`timescale 1ns/100ps

module testbench;

    logic clk;
    logic rst;

    // 例化頂層模組
    Top uut (
        .clk(clk),
        .rst(rst)
    );

    // 產生時脈：週期 10ns
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // 產生 reset：前 20ns 保持高電位
    initial begin
        rst = 1;
        #20 rst = 0;
    end

    // 模擬持續時間與觀察點
    initial begin
        $dumpfile("waveform.vcd");  // 若用 Verilator 或 GTKWave
        $dumpvars(0, testbench);    // 觀察全部訊號
        #50000 $stop;                 
    end

endmodule
