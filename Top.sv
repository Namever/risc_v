//======================
// Top Module
//======================
module Top(
	 input logic clk,
	 input logic rst,
	 output logic [31:0] regs_31
);

    logic rst_pc_;
    logic flush_IFID_, flush_IDEX_,sel_pc_,sel_pc_r,flush_IDEX_r,flush_IFID_r;

    logic [31:0] pc, inst, inst_r, pc_r,pc_rr;
    logic [6:0]  opcode_, funct7_;
    logic [2:0]  funct3_,funct3_r;
    logic [4:0]  addr_rd_, addr_rs1_, addr_rs2_, addr_rd_r;
    logic [31:0] imm_, imm_r,alu_a,alu_b,alu_out,jump_addr_;
    logic [31:0] rs1_value_, rs2_value_, rd_value_, rs1_value_r, rs2_value_r;
    logic        write_regf_en, write_regf_en_r,sel_jump_,sel_jump_r,sel_alu_a_,sel_alu_a_r;
	logic [1:0]  sel_alu_b_,sel_alu_b_r,sel_rd_value_,sel_rd_value_r;
	logic [3:0]  op,op_r;
	
	//LSU
	logic        write_read_,write_read_r;
	logic [31:0] ram_addr;

    Program_Counter u_pc (
        .clk    	(clk),
        .rst   	    (rst),
        .rst_pc 	(rst_pc_),
        .pc     	(pc),
		.jump_addr_ (jump_addr_),
		.sel_pc_r   (sel_pc_r)
    );

    Program_Rom u_rom (
        .Rom_addr (pc),
        .Rom_data (inst)
    );

    IF_ID u_ifid (
        .clk        (clk),
        .rst        (rst),
        .flush_IFID_r(flush_IFID_r),
        .inst       (inst),
        .pc         (pc),
        .inst_r     (inst_r),
        .pc_r       (pc_r)
    );

    INST_DEC u_dec (
        .inst_r     (inst_r),
        .opcode_    (opcode_),
        .funct3_    (funct3_),
        .funct7_    (funct7_),
        .addr_rd_   (addr_rd_),
        .addr_rs1_  (addr_rs1_),
        .addr_rs2_  (addr_rs2_),
        .imm_       (imm_)
    );

    CONTROLLER u_ctrl (
        .clk            (clk),
        .rst            (rst),
        .opcode_        (opcode_),
        .flush_IFID_    (flush_IFID_),
        .flush_IDEX_    (flush_IDEX_),
        .rst_pc_        (rst_pc_),
        .write_regf_en  (write_regf_en),
		.funct3_    	(funct3_),
        .funct7_   	    (funct7_),
		.rs1_value_     (rs1_value_),
        .rs2_value_     (rs2_value_),
		.op             (op),
		.sel_jump_      (sel_jump_),
		.sel_alu_a_     (sel_alu_a_),
		.sel_alu_b_     (sel_alu_b_),
		.sel_pc_        (sel_pc_),
		.sel_rd_value_  (sel_rd_value_),
		.write_read_    (write_read_)
    );

    REG_FILE u_reg (
        .clk            (clk),
        .rst            (rst),
		.write_regf_en_r(write_regf_en_r),
        .addr_rd_r      (addr_rd_r),
        .addr_rs1       (addr_rs1_),
        .addr_rs2       (addr_rs2_),
        .rd_value_      (rd_value_),
        .rs1_value_     (rs1_value_),
        .rs2_value_     (rs2_value_),
		.regs_31        (regs_31) 
    );

    ID_EX u_idex (
        .clk            (clk),
        .rst            (rst),
        .flush_IDEX_    (flush_IDEX_),
		.flush_IFID_    (flush_IFID_),
		.flush_IDEX_r	(flush_IDEX_r),
		.flush_IFID_r	(flush_IFID_r),
		.sel_pc_		(sel_pc_),
		.sel_pc_r		(sel_pc_r),
        .write_regf_en  (write_regf_en),
        .imm            (imm_),
        .rs1_value_     (rs1_value_),
        .rs2_value_     (rs2_value_),
        .addr_rd        (addr_rd_),
        .imm_r          (imm_r),
        .rs1_value_r    (rs1_value_r),
        .rs2_value_r    (rs2_value_r),
        .addr_rd_r      (addr_rd_r),
        .write_regf_en_r(write_regf_en_r),
		.op             (op),
		.op_r           (op_r),
		.sel_jump_		(sel_jump_),
		.sel_jump_r		(sel_jump_r),
		.sel_alu_a_		(sel_alu_a_),
		.sel_alu_a_r	(sel_alu_a_r),
		.sel_alu_b_     (sel_alu_b_),
	    .sel_alu_b_r    (sel_alu_b_r),
		.pc_r           (pc_r),
		.pc_rr          (pc_rr),
		.jump_addr_     (jump_addr_),
		.funct3_        (funct3_),
		.funct3_r		(funct3_r),
		.sel_rd_value_  (sel_rd_value_),
		.sel_rd_value_r (sel_rd_value_r),
		.write_read_    (write_read_),
		.write_read_r   (write_read_r)
		
    );
	 
	 ALU u_alu (
			.rs1_value_r    (rs1_value_r),
			.rs2_value_r    (rs2_value_r),
			.imm_r          (imm_r),
			.alu_out		(alu_out),
			.op_r           (op_r),
			.sel_alu_a_r	(sel_alu_a_r),
			.sel_alu_b_r    (sel_alu_b_r),
			.pc_rr			(pc_rr)
	 );
	 
	LSU u_lsu (
			.clk            (clk),
			.funct3_r       (funct3_r),
			.rs1_value_r    (rs1_value_r),
			.rs2_value_r    (rs2_value_r), 
			.write_read_r   (write_read_r),
			.alu_out	    (alu_out),
			.sel_rd_value_r (sel_rd_value_r),
			.rd_value_      (rd_value_)
	);
	 
	 
endmodule
