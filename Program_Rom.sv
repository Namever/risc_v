module Program_Rom(
   output logic [31:0] Rom_data,
   input [31:0] Rom_addr
);

   always_comb begin
      case (Rom_addr)
         32'h00000000 : Rom_data = 32'hFE010113; // addi x2 x2 -32
         32'h00000004 : Rom_data = 32'h00112E23; // sw x1 28(x2)
         32'h00000008 : Rom_data = 32'h00812C23; // sw x8 24(x2)
         32'h0000000C : Rom_data = 32'h02010413; // addi x8 x2 32
         32'h00000010 : Rom_data = 32'h00000F93; // addi x31 x0 0
         32'h00000014 : Rom_data = 32'hFE042623; // sw x0 -20(x8)
         32'h00000018 : Rom_data = 32'h0200006F; // jal x0 32
         32'h0000001C : Rom_data = 32'h000F8713; // addi x14 x31 0
         32'h00000020 : Rom_data = 32'hFEC42783; // lw x15 -20(x8)
         32'h00000024 : Rom_data = 32'h00F707B3; // add x15 x14 x15
         32'h00000028 : Rom_data = 32'h00078F93; // addi x31 x15 0
         32'h0000002C : Rom_data = 32'hFEC42783; // lw x15 -20(x8)
         32'h00000030 : Rom_data = 32'h00178793; // addi x15 x15 1
         32'h00000034 : Rom_data = 32'hFEF42623; // sw x15 -20(x8)
         32'h00000038 : Rom_data = 32'hFEC42703; // lw x14 -20(x8)
         32'h0000003C : Rom_data = 32'h00A00793; // addi x15 x0 10
         32'h00000040 : Rom_data = 32'hFCE7DEE3; // bge x15 x14 -36
         32'h00000044 : Rom_data = 32'hFCDFF06F; // jal x0 -52
         default : Rom_data = 32'h00000013;      // NOP
      endcase
   end

endmodule
