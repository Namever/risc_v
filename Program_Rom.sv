module Program_Rom(
   output logic [31:0] Rom_data,
   input [31:0] Rom_addr
);

   always_comb begin
      case (Rom_addr)
         32'h00000000 : Rom_data = 32'h1FC00113; // addi x2 x0 508
         32'h00000004 : Rom_data = 32'hFE010113; // addi x2 x2 -32
         32'h00000008 : Rom_data = 32'h00112E23; // sw x1 28(x2)
         32'h0000000C : Rom_data = 32'h00812C23; // sw x8 24(x2)
         32'h00000010 : Rom_data = 32'h02010413; // addi x8 x2 32
         32'h00000014 : Rom_data = 32'h00000F93; // addi x31 x0 0
         32'h00000018 : Rom_data = 32'hFE042623; // sw x0 -20(x8)
         32'h0000001C : Rom_data = 32'h0280006F; // jal x0 40
         32'h00000020 : Rom_data = 32'h000F8713; // addi x14 x31 0
         32'h00000024 : Rom_data = 32'hFEC42783; // lw x15 -20(x8)
         32'h00000028 : Rom_data = 32'h00F707B3; // add x15 x14 x15
         32'h0000002C : Rom_data = 32'h00078F93; // addi x31 x15 0
         32'h00000030 : Rom_data = 32'h00000317; // auipc x6 0
         32'h00000034 : Rom_data = 32'h024300E7; // jalr x1 x6 36
         32'h00000038 : Rom_data = 32'hFEC42783; // lw x15 -20(x8)
         32'h0000003C : Rom_data = 32'h00178793; // addi x15 x15 1
         32'h00000040 : Rom_data = 32'hFEF42623; // sw x15 -20(x8)
         32'h00000044 : Rom_data = 32'hFEC42703; // lw x14 -20(x8)
         32'h00000048 : Rom_data = 32'h06300793; // addi x15 x0 99
         32'h0000004C : Rom_data = 32'hFCE7DAE3; // bge x15 x14 -44
         32'h00000050 : Rom_data = 32'hFC5FF06F; // jal x0 -60
         32'h00000054 : Rom_data = 32'hFE010113; // addi x2 x2 -32
         32'h00000058 : Rom_data = 32'h00112E23; // sw x1 28(x2)
         32'h0000005C : Rom_data = 32'h00812C23; // sw x8 24(x2)
         32'h00000060 : Rom_data = 32'h02010413; // addi x8 x2 32
         32'h00000064 : Rom_data = 32'hFE042623; // sw x0 -20(x8)
         32'h00000068 : Rom_data = 32'h0380006F; // jal x0 56
         32'h0000006C : Rom_data = 32'hFE042423; // sw x0 -24(x8)
         32'h00000070 : Rom_data = 32'h0100006F; // jal x0 16
         32'h00000074 : Rom_data = 32'hFE842783; // lw x15 -24(x8)
         32'h00000078 : Rom_data = 32'h00178793; // addi x15 x15 1
         32'h0000007C : Rom_data = 32'hFEF42423; // sw x15 -24(x8)
         32'h00000080 : Rom_data = 32'hFE842703; // lw x14 -24(x8)
         32'h00000084 : Rom_data = 32'h000017B7; // lui x15 1
         32'h00000088 : Rom_data = 32'h00078793; // addi x15 x15 0
         32'h0000008C : Rom_data = 32'h38778793; // addi x15 x15 903
         32'h00000090 : Rom_data = 32'hFEE7D2E3; // bge x15 x14 -28
         32'h00000094 : Rom_data = 32'hFEC42783; // lw x15 -20(x8)
         32'h00000098 : Rom_data = 32'h00178793; // addi x15 x15 1
         32'h0000009C : Rom_data = 32'hFEF42623; // sw x15 -20(x8)
         32'h000000A0 : Rom_data = 32'hFEC42703; // lw x14 -20(x8)
         32'h000000A4 : Rom_data = 32'h000017B7; // lui x15 1
         32'h000000A8 : Rom_data = 32'h00078793; // addi x15 x15 0
         32'h000000AC : Rom_data = 32'h38778793; // addi x15 x15 903
         32'h000000B0 : Rom_data = 32'hFAE7DEE3; // bge x15 x14 -68
         32'h000000B4 : Rom_data = 32'h00000793; // addi x15 x0 0
         32'h000000B8 : Rom_data = 32'h00078513; // addi x10 x15 0
         32'h000000BC : Rom_data = 32'h01C12083; // lw x1 28(x2)
         32'h000000C0 : Rom_data = 32'h01812403; // lw x8 24(x2)
         32'h000000C4 : Rom_data = 32'h02010113; // addi x2 x2 32
         32'h000000C8 : Rom_data = 32'h00008067; // jalr x0 x1 0
         default : Rom_data = 32'h00000013;      // NOP
      endcase
   end

endmodule
