(* dont_touch = "true" *)
module pipeline_main (
    input clk, rst,
    output [7:0] Anode_Activate,
    output [7:0] LED_out
);
    //-------------------------------------------------------------
    // Registers / Wires
    //-------------------------------------------------------------
    
    // Clocks
    wire clk_50MHZ;
    wire clk_20MHZ;
    wire locked;
    
    // Flags
    wire Zero_Flag;
    wire Sign_Flag;

    //-----------------------Data Signals--------------------------
    wire [31:0] PC;                 // The current PC
    wire [31:0] Instruction;
    wire [31:0] Jump_Addr;
    wire [4:0] rs1;
    wire [4:0] rs2;
    wire [4:0] rd;
    wire [3:0] ALU_Op;
    wire [31:0] RF_Out1, RF_Out2;
    wire [31:0] Imm_Value;
    wire [31:0] ALU_Result;
    wire [31:0] Mem_Read_Data;
    wire [31:0] Reg_Write_Data;
    wire [31:0] Mem_LED_out;

    // Stage Register wires
    // IF_ID Stage
    wire [31:0] PC_IF_ID;
    wire [31:0] Instruction_IF_ID;
    wire [31:0] Jump_Addr_IF_ID;
    wire [31:0] Return_Addr_IF_ID;

    // ID_EX Stage
    wire [4:0] rs1_ID_EX;
    wire [4:0] rs2_ID_EX;
    wire [4:0] rd_ID_EX;
    wire [31:0] PC_ID_EX;
    wire [31:0] RF_Out1_ID_EX;
    wire [31:0] RF_Out2_ID_EX;
    wire [31:0] Imm_Value_ID_EX;
    wire [31:0] Return_Addr_ID_EX;
    wire Instruction_14_ID_EX;

    // EX_MEM Stage
    wire [4:0] rd_EX_MEM;
    wire [31:0] ALU_Result_EX_MEM;
    wire [31:0] RF_Out2_EX_MEM;
    wire [31:0] Return_Addr_EX_MEM;

    // MEM_WB Stage
    wire [4:0] rd_MEM_WB;
    wire [31:0] ALU_Result_MEM_WB;
    wire [31:0] Mem_Read_Data_MEM_WB;
    wire [31:0] Return_Addr_MEM_WB;

    //---------------------Control Signals-------------------------
    //wire [1:0] PC_Src;              // Selects PC source (sequential or branch)
    wire Reg_Write;                 // Register write signal
    wire ALU_Src;                   // Selects ALU Source (Register or Immediate)
    wire Mem_Read;                  // Memory Read Control Signal
    wire Mem_Write;                 // Memory Write Control Signal
    wire Mem_to_Reg;                // Memory to Register Control Signal

    wire Signal_Flush;              // Pipeline flush
    wire Signal_Branch;             // Branch
    wire Signal_Jump;               // Jump
    wire Signal_Stall;              // Stall
    wire [1:0] State;               // Current state of branch predictor
    wire Outcome;
    wire [1:0] Entry;    

    // Stage Register wires
    // IF_ID Stage
    wire Signal_Branch_IF_ID;
    wire Signal_Jump_IF_ID;
    wire [1:0] State_IF_ID;
    wire [1:0] Addr_IF_ID;

    // ID_EX Stage
    wire Signal_Branch_ID_EX;
    wire Signal_Jump_ID_EX;
    wire [1:0] State_ID_EX;
    wire [1:0] Addr_ID_EX;
    wire [3:0] ALU_Op_ID_EX;
    wire Reg_Write_ID_EX;
    wire ALU_Src_ID_EX;
    wire Mem_Read_ID_EX;
    wire Mem_Write_ID_EX;
    wire Mem_to_Reg_ID_EX;

    // EX_MEM stage
    wire Signal_Jump_EX_MEM;
    wire Reg_Write_EX_MEM;
    wire Mem_Read_EX_MEM;
    wire Mem_Write_EX_MEM;
    wire Mem_to_Reg_EX_MEM;

    // MEM_WB Stage
    wire Signal_Jump_MEM_WB;
    wire Reg_Write_MEM_WB;
    wire Mem_to_Reg_MEM_WB;

    // Forwarding Unit
    wire [1:0] Forward_A;
    wire [1:0] Forward_B;

    // CNN
    wire cnn_en;
    wire [3:0] predicted_class_LED;
    

    //------------------------Parameters---------------------------
    localparam [31:0] PC_exception = 32'h1C090000;  // Address to jump to in case of exceptions


    //-------------------------------------------------------------
    // Logic Definition
    //-------------------------------------------------------------

    // Defining fields from Instructions
    assign rs1 = Instruction_IF_ID[19:15];    // Register Select 1
    assign rs2 = Instruction_IF_ID[24:20];    // Register Select 1
    assign rd = Instruction_IF_ID[11:7];      // Destination Register Select
    assign Outcome = Instruction_14_ID_EX ? (Signal_Branch_ID_EX && Sign_Flag) : (Signal_Branch_ID_EX && Zero_Flag);
    
    //-------------------------------------------------------------
    // Module Instantiation
    //-------------------------------------------------------------

    // Mux takes in PC+4, Exception Cycle intialization address or Branch Address
    
    clk_wiz_0 clk_generation (
        // Clock out ports
        .clk_50MHZ(clk_50MHZ),     // output clk_50MHZ
        .clk_20MHZ(clk_20MHZ),     // output clk_20MHZ
        // Status and control signals
        .reset(rst), // input reset
        .locked(locked),       // output locked
        // Clock in ports
        .clk_in1(clk)      // input clk_in1
    );


    PC_Module m1 (
        .clk_50MHZ(clk_50MHZ),
        .rst(rst),
        .Signal_Branch(Signal_Branch),
        .Signal_Branch_ID_EX(Signal_Branch_ID_EX),
        .Signal_Jump(Signal_Jump),
        .Outcome(Outcome),
        .Signal_Stall(Signal_Stall),
        .Signal_Ecall(Signal_Ecall),
        .State(State),
        .Jump_Addr(Jump_Addr),
        .PC_ID_EX(PC_ID_EX),
        .PC(PC)
    );

    Instruction_Memory m2 (
        .Mem_Address(PC),
        .Instruction(Instruction)
    );

    Branch_Jump m3 (
        .Instruction(Instruction),
        .Signal_Branch(Signal_Branch),
        .Signal_Jump(Signal_Jump),
        .Signal_Ecall(Signal_Ecall),
        .Jump_Addr(Jump_Addr)
    );

    IF_ID_reg m4 (
        .clk_50MHZ(clk_50MHZ),
        .rst(rst),
        .Signal_Flush(Signal_Flush),
        .Signal_Branch(Signal_Branch),
        .Signal_Jump(Signal_Jump),
        .Signal_Stall(Signal_Stall),
        .State(State),
        .PC(PC),
        .Instruction(Instruction),
        .Jump_Addr(Jump_Addr),
        .Signal_Branch_IF_ID(Signal_Branch_IF_ID),
        .Signal_Jump_IF_ID(Signal_Jump_IF_ID),
        .State_IF_ID(State_IF_ID),
        .Addr_IF_ID(Addr_IF_ID),
        .PC_IF_ID(PC_IF_ID),
        .Instruction_IF_ID(Instruction_IF_ID),
        .Jump_Addr_IF_ID(Jump_Addr_IF_ID),
        .Return_Addr_IF_ID(Return_Addr_IF_ID)
    );

    Register_File m5 (
        .clk_50MHZ(clk_50MHZ),
        .rst(rst),
        .Reg_Write(Reg_Write_MEM_WB),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd_MEM_WB),
        .Reg_Write_Data(Reg_Write_Data),
        .RD_Data1_ID_EX(RF_Out1),
        .RD_Data2_ID_EX(RF_Out2)
    );

    Control_Unit m6 (
        .Signal_Stall(Signal_Stall),
        .Instruction_IF_ID(Instruction_IF_ID),
        .Reg_Write(Reg_Write),
        .ALU_Src(ALU_Src),
        .Mem_Read(Mem_Read),
        .Mem_Write(Mem_Write),
        .Mem_to_Reg(Mem_to_Reg),
        .ALU_Op(ALU_Op),
        .Imm_Value(Imm_Value),
        .cnn_en(cnn_en)
    );

    ID_EX_reg m7 (
        .clk_50MHZ(clk_50MHZ),
        .rst(rst),
        .Signal_Flush(Signal_Flush),
        .Signal_Branch_IF_ID(Signal_Branch_IF_ID),
        .Signal_Jump_IF_ID(Signal_Jump_IF_ID),
        .Signal_Stall(Signal_Stall),
        .Addr_IF_ID(Addr_IF_ID),
        .State_IF_ID(State_IF_ID),
        .ALU_Op(ALU_Op),
        .Reg_Write(Reg_Write),
        .ALU_Src(ALU_Src),
        .Mem_Read(Mem_Read),
        .Mem_Write(Mem_Write),
        .Mem_to_Reg(Mem_to_Reg),
        .rs1(rs1),
        .rs2(rs2),
        .rd(rd),
        .PC_IF_ID(PC_IF_ID),
        .RF_Out1(RF_Out1),
        .RF_Out2(RF_Out2),
        .Jump_Addr_IF_ID(Jump_Addr_IF_ID),
        .Return_Addr_IF_ID(Return_Addr_IF_ID),
        .Instruction_14_IF_ID(Instruction_IF_ID[14]),
        .Imm_Value(Imm_Value),
        .Signal_Branch_ID_EX(Signal_Branch_ID_EX),
        .Signal_Jump_ID_EX(Signal_Jump_ID_EX),
        .Addr_ID_EX(Addr_ID_EX),
        .State_ID_EX(State_ID_EX),
        .ALU_Op_ID_EX(ALU_Op_ID_EX),
        .Reg_Write_ID_EX(Reg_Write_ID_EX),
        .ALU_Src_ID_EX(ALU_Src_ID_EX),
        .Mem_Read_ID_EX(Mem_Read_ID_EX),
        .Mem_Write_ID_EX(Mem_Write_ID_EX),
        .Mem_to_Reg_ID_EX(Mem_to_Reg_ID_EX),
        .rs1_ID_EX(rs1_ID_EX),
        .rs2_ID_EX(rs2_ID_EX),
        .rd_ID_EX(rd_ID_EX),
        .PC_ID_EX(PC_ID_EX),
        .RF_Out1_ID_EX(RF_Out1_ID_EX),
        .RF_Out2_ID_EX(RF_Out2_ID_EX),
        .Return_Addr_ID_EX(Return_Addr_ID_EX),
        .Instruction_14_ID_EX(Instruction_14_ID_EX),
        .Imm_Value_ID_EX(Imm_Value_ID_EX)
    );

    ALU m8 (
        .ALU_Op_ID_EX(ALU_Op_ID_EX),
        .ALU_In1_ID_EX(RF_Out1_ID_EX),     
        .ALU_In2_ID_EX(RF_Out2_ID_EX),
        .Forward_A(Forward_A),
        .Forward_B(Forward_B),
        .ALU_Src_ID_EX(ALU_Src_ID_EX),
        .ALU_Result_EX_MEM(ALU_Result_EX_MEM),
        .Reg_Write_Data_MEM_WB(Reg_Write_Data),
        .Imm_Value_ID_EX(Imm_Value_ID_EX),     
        .Zero_Flag(Zero_Flag),
        .Sign_Flag(Sign_Flag),
        .ALU_Result(ALU_Result)
    );

    EX_MEM_Reg m9 (
        .clk_50MHZ(clk_50MHZ),
        .rst(rst),
        .Signal_Jump_ID_EX(Signal_Jump_ID_EX),
        .Reg_Write_ID_EX(Reg_Write_ID_EX),
        .Mem_Read_ID_EX(Mem_Read_ID_EX),
        .Mem_Write_ID_EX(Mem_Write_ID_EX),
        .Mem_to_Reg_ID_EX(Mem_to_Reg_ID_EX),
        .rd_ID_EX(rd_ID_EX),
        .ALU_Result(ALU_Result),
        .RF_Out2_ID_EX(RF_Out2_ID_EX),
        .Return_Addr_ID_EX(Return_Addr_ID_EX),
        .Signal_Jump_EX_MEM(Signal_Jump_EX_MEM),
        .Reg_Write_EX_MEM(Reg_Write_EX_MEM),
        .Mem_Read_EX_MEM(Mem_Read_EX_MEM),
        .Mem_Write_EX_MEM(Mem_Write_EX_MEM),
        .Mem_to_Reg_EX_MEM(Mem_to_Reg_EX_MEM),
        .rd_EX_MEM(rd_EX_MEM),
        .ALU_Result_EX_MEM(ALU_Result_EX_MEM),
        .RF_Out2_EX_MEM(RF_Out2_EX_MEM),
        .Return_Addr_EX_MEM(Return_Addr_EX_MEM)
    );

    Data_Memory m10 (
        .clk_50MHZ(clk_50MHZ),
        .Mem_Write_EX_MEM(Mem_Write_EX_MEM),
        .Mem_Read_EX_MEM(Mem_Read_EX_MEM),
        .Mem_Write_Data(RF_Out2_EX_MEM),
        .Mem_Address(ALU_Result_EX_MEM),
        .Mem_Read_Data(Mem_Read_Data),
        .Mem_LED_out(Mem_LED_out)
    );

    MEM_WB_Reg m11 (
        .clk_50MHZ(clk_50MHZ),
        .rst(rst),
        .Signal_Jump_EX_MEM(Signal_Jump_EX_MEM),
        .Reg_Write_EX_MEM(Reg_Write_EX_MEM),
        .Mem_to_Reg_EX_MEM(Mem_to_Reg_EX_MEM),
        .rd_EX_MEM(rd_EX_MEM),
        .ALU_Result_EX_MEM(ALU_Result_EX_MEM),
        .Mem_Read_Data(Mem_Read_Data),
        .Return_Addr_EX_MEM(Return_Addr_EX_MEM),
        .Signal_Jump_MEM_WB(Signal_Jump_MEM_WB),
        .Reg_Write_MEM_WB(Reg_Write_MEM_WB),
        .Mem_to_Reg_MEM_WB(Mem_to_Reg_MEM_WB),
        .rd_MEM_WB(rd_MEM_WB),
        .ALU_Result_MEM_WB(ALU_Result_MEM_WB),
        .Mem_Read_Data_MEM_WB(Mem_Read_Data_MEM_WB),
        .Return_Addr_MEM_WB(Return_Addr_MEM_WB)
    );

    Forwarding_Unit m12 (
        .Reg_Write_EX_MEM(Reg_Write_EX_MEM),
        .Reg_Write_MEM_WB(Reg_Write_MEM_WB),
        .rd_EX_MEM(rd_EX_MEM),
        .rd_MEM_WB(rd_MEM_WB),
        .rs1_ID_EX(rs1_ID_EX),
        .rs2_ID_EX(rs2_ID_EX),
        .Forward_A(Forward_A),
        .Forward_B(Forward_B)
    );

    Hazard_Detection_Unit m13 (
        .rst(rst),
        .Mem_Read_ID_EX(Mem_Read_ID_EX),
        .rd_ID_EX(rd_ID_EX),
        .rs1_IF_ID(rs1),
        .rs2_IF_ID(rs2),
        .Signal_Stall(Signal_Stall)
    );

    Branch_Table m14 (
        .clk_50MHZ(clk_50MHZ),
        .rst(rst),
        .Signal_Branch_ID_EX(Signal_Branch_ID_EX),
        .Addr(PC[4:3]),
        .Addr_ID_EX(Addr_ID_EX),
        .Entry(Entry),
        .State(State)
    );

    Branch_Predictor m15(
        .Outcome(Outcome),
        .State_ID_EX(State_ID_EX),
        .Entry(Entry),
        .Signal_Flush(Signal_Flush)
    );

    Writeback_Unit m16 (
        .Mem_to_Reg_MEM_WB(Mem_to_Reg_MEM_WB),
        .Signal_Jump_MEM_WB(Signal_Jump_MEM_WB),
        .ALU_Result_MEM_WB(ALU_Result_MEM_WB),
        .Mem_Read_Data_MEM_WB(Mem_Read_Data_MEM_WB),
        .Return_Addr_MEM_WB(Return_Addr_MEM_WB),
        .Reg_Write_Data(Reg_Write_Data)
    );

    Eight_Digit_Hex_Display m17 (
        .clk_50MHZ(clk_50MHZ),
        .rst(rst),
        .Mem_LED_in(Mem_LED_out[27:0]),
        .predicted_class_LED(predicted_class_LED),
        .Anode_Activate(Anode_Activate),
        .LED_out(LED_out)
    );
    
    cnn_core m18 (
        .clk_20MHZ(clk_20MHZ),
        .rst(rst),
        .cnn_en(cnn_en),
        .predicted_class_LED(predicted_class_LED)
    );

endmodule


//Verified 
(* dont_touch = "true" *)
module PC_Module (
    // Inputs
    input wire clk_50MHZ,
    input wire rst,
    input wire Signal_Branch,
    input wire Signal_Branch_ID_EX,
    input wire Signal_Jump,
    input wire Outcome,
    input wire Signal_Stall,
    input wire Signal_Ecall,
    input wire [1:0] State,
    input wire [31:0] Jump_Addr,
    input wire [31:0] PC_ID_EX,

    // Outputs
    output reg [31:0] PC
);
    //-------------------------------------------------------------
    // Registers / Wires
    //-------------------------------------------------------------
    wire Prediction;
    
    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------    
    assign Prediction = State[1];
    
    always @(posedge clk_50MHZ, posedge rst) begin
        if (rst==1) begin
            PC <= 32'h0; // rst
        end
        else if (Signal_Stall==1 || Signal_Ecall==1) begin
            PC <= PC+0; // Stall
        end
        else if (Signal_Branch_ID_EX==1 && Outcome==1) begin
            PC <= PC_ID_EX; // Hazard
        end
        else if ((Signal_Branch==1 && Prediction==1) || Signal_Jump==1) begin
            PC <= PC + Jump_Addr; // Jump
        end
        else if (Signal_Branch_ID_EX==0 || (Signal_Branch_ID_EX==1 && Outcome==0) || (Signal_Branch==1 && Prediction==0) || Signal_Branch==0 || Signal_Jump==0) begin
            PC <= PC + 4; // Normal execution
        end
    end

endmodule

//Verified
(* dont_touch = "true" *)
module Instruction_Memory (
    // Inputs
    input wire [31:0] Mem_Address,

    // Outputs
    output reg [31:0] Instruction
);

    //-------------------------------------------------------------
    // Registers / Wires
    //-------------------------------------------------------------
    (*ram_style = "block"*) reg [7:0] mem [43:0];

    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------
    always @(*) begin
        Instruction = {mem[Mem_Address+3], mem[Mem_Address+2], mem[Mem_Address+1], mem[Mem_Address]};
    end
    
    initial begin
            mem[3]  = 8'h00; mem[2]  = 8'h02; mem[1]  = 8'ha3; mem[0]  = 8'h03; // lw t1,0(t0)
            mem[7]  = 8'h00; mem[6]  = 8'h42; mem[5]  = 8'h82; mem[4]  = 8'h93; // addi t0,t0,4
            mem[11] = 8'h00; mem[10] = 8'h02; mem[9]  = 8'ha3; mem[8]  = 8'h83; // lw t2,0(t0)
            mem[15] = 8'h00; mem[14] = 8'h73; mem[13] = 8'h0c; mem[12] = 8'h63; // loop: beq t1,t2,exit
            mem[19] = 8'h00; mem[18] = 8'h73; mem[17] = 8'h46; mem[16] = 8'h63; // blt t1,t2,L1
            mem[23] = 8'h40; mem[22] = 8'h73; mem[21] = 8'h03; mem[20] = 8'h33; // sub t1,t1,t2
            mem[27] = 8'hff; mem[26] = 8'h5f; mem[25] = 8'hf0; mem[24] = 8'h6f; // j loop
            mem[31] = 8'h40; mem[30] = 8'h63; mem[29] = 8'h83; mem[28] = 8'hb3; // L1: sub t2,t2,t1
            mem[35] = 8'hfe; mem[34] = 8'hdf; mem[33] = 8'hf0; mem[32] = 8'h6f; // j loop
            mem[39] = 8'h00; mem[38] = 8'h62; mem[37] = 8'ha2; mem[36] = 8'h23; // exit: sw t1,4(t0)
            mem[43] = 8'hFE; mem[42] = 8'h00; mem[41] = 8'h70; mem[40] = 8'h7F; // cnn FE00707F 11111110000000000111000001111111
            mem[47] = 8'h00; mem[46] = 8'h00; mem[45] = 8'h00; mem[44] = 8'h73; // ecall
            
    end
    
endmodule

//Verified
(* dont_touch = "true" *)
module Branch_Jump (
    // Inputs
    input wire [31:0] Instruction,

    // Outputs
    output reg Signal_Branch,
    output reg Signal_Jump,
    output reg Signal_Ecall,
    output reg [31:0] Jump_Addr
);
    
    //-------------------------------------------------------------
    // Registers / Wires
    //-------------------------------------------------------------
    wire [6:0] Opcode;

    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------
    assign Opcode = Instruction[6:0];

    always @* begin
        case (Opcode)
            7'b1100011: begin       // B-Type
                Signal_Branch = 1;
                Signal_Jump = 0;
                Signal_Ecall = 0;
                Jump_Addr = {{20{Instruction[31]}}, Instruction[7], Instruction[30:25], Instruction[11:8], 1'b0};      
            end

            7'b1101111: begin       // J-Type
                Signal_Branch = 0;
                Signal_Jump = 1;
                Signal_Ecall = 0;
                Jump_Addr = {{12{Instruction[31]}}, Instruction[19:12], Instruction[20], Instruction[30:21], 1'b0};    
            end

            7'b1110011: begin       // Ecall
                Signal_Branch = 0;
                Signal_Jump = 0;
                Signal_Ecall = 1;
                Jump_Addr = 32'dx;                                                  
            end

            default: begin      // Rest of the instructions
                Signal_Branch = 0;
                Signal_Jump = 0;
                Signal_Ecall = 0;                                                   
                Jump_Addr = 32'd0;
            end
        endcase
    end
    
endmodule

//Verified and check rst
(* dont_touch = "true" *)
module IF_ID_reg (
    // Inputs
    // Control Signals
    input wire clk_50MHZ,
    input wire rst,
    input wire Signal_Flush,            // Pipeline flush
    input wire Signal_Branch,           // Branch
    input wire Signal_Jump,             // Jump
    input wire Signal_Stall,            // Stall
    input wire [1:0] State,             // Current state of branch predictor

    // Data Signals
    input wire [31:0] PC,       
    input wire [31:0] Instruction,
    input wire [31:0] Jump_Addr,

    // Outputs
    // Control Signals
    output reg Signal_Branch_IF_ID,
    output reg Signal_Jump_IF_ID,
    output reg [1:0] State_IF_ID,

    // Data Signals
    output reg [1:0] Addr_IF_ID,
    output reg [31:0] PC_IF_ID,
    output reg [31:0] Instruction_IF_ID,
    output reg [31:0] Jump_Addr_IF_ID,
    output reg [31:0] Return_Addr_IF_ID
);

    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------
    always @(posedge clk_50MHZ) begin
        if (rst == 1) begin
            // Control Signals
            Signal_Branch_IF_ID <= 0;
            Signal_Jump_IF_ID   <= 0;
            State_IF_ID         <= 0;

            // Data Signals
            Addr_IF_ID          <= 0;
            PC_IF_ID            <= 0;
            Instruction_IF_ID   <= 0;
            Jump_Addr_IF_ID     <= 0;
            Return_Addr_IF_ID   <= 0;
        end

        else if (Signal_Flush==1) begin

            // Control Signals
            Signal_Branch_IF_ID <= 0;
            Signal_Jump_IF_ID   <= 0;
            State_IF_ID         <= 0;

            // Data Signals
            Addr_IF_ID          <= 0;
            PC_IF_ID            <= 0;
            Instruction_IF_ID   <= 0;
            Jump_Addr_IF_ID     <= 0;
            Return_Addr_IF_ID   <= 0;
        end

        else if (!Signal_Stall) begin

            // Control Signals
            Signal_Branch_IF_ID <= Signal_Branch;
            Signal_Jump_IF_ID   <= Signal_Jump;
            State_IF_ID         <= State;

            // Data Signals
            Addr_IF_ID          <= PC[4:3];
            PC_IF_ID            <= PC;
            Instruction_IF_ID   <= Instruction;
            Jump_Addr_IF_ID     <= Jump_Addr;
            Return_Addr_IF_ID   <= PC + 4;
        end
    end
    
endmodule


//Verified and rst check
(* dont_touch = "true" *)
module Register_File (
    // Inputs
    input wire clk_50MHZ,
    input wire rst,    
    input wire Reg_Write,
    input wire [4:0] rs1,
    input wire [4:0] rs2,
    input wire [4:0] rd,
    input wire [31:0] Reg_Write_Data,

    // Outputs
    output reg [31:0] RD_Data1_ID_EX, RD_Data2_ID_EX
);
    
    //-------------------------------------------------------------
    // Registers / Wires
    //-------------------------------------------------------------
    reg [31:0] Reg_Mem [31:0];

    integer i;

    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------
    always @(*) begin
        RD_Data1_ID_EX <= Reg_Mem[rs1];
        RD_Data2_ID_EX <= Reg_Mem[rs2];
    end
        
    always @(negedge clk_50MHZ) begin
        if (rst == 1) begin
            for (i = 0; i < 32; i = i + 1)
                Reg_Mem[i] <= 0;
        end
   
        else begin
            if (Reg_Write & (rd != 5'b00000))
                Reg_Mem[rd] <= Reg_Write_Data;
        end
    end

endmodule


//Verified
(* dont_touch = "true" *)
module Control_Unit (
    // Inputs
    input wire Signal_Stall,
    input wire [31:0] Instruction_IF_ID,

    // Outputs
    output reg Reg_Write, ALU_Src, Mem_Read, Mem_Write, Mem_to_Reg, cnn_en,
    output reg [3:0] ALU_Op,
    output reg [31:0] Imm_Value
);
    
    //-------------------------------------------------------------
    // Registers / Wires
    //-------------------------------------------------------------
    wire [6:0] Opcode;
    wire [10:0] Control;

    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------

    // Defining opcode and control fields
    assign Opcode = Instruction_IF_ID[6:0];
    assign Control = {Instruction_IF_ID[30], Instruction_IF_ID[14:12], Instruction_IF_ID[6:0]};

    // Control Signals decode (except ALU_Op)
    always @(*) begin
        if (Signal_Stall) begin
            Reg_Write   <= 0;
            ALU_Src     <= 0;
            Mem_Read    <= 0;
            Mem_Write   <= 0;
            Mem_to_Reg  <= 0;
            Imm_Value   <= 32'h00000000;
            cnn_en      <= 1'b0;
        end

        else begin
            case (Opcode)
                7'b0110011:         // R-type instructions
                    begin
                        Reg_Write   <= 1;
                        ALU_Src     <= 0;
                        Mem_Read    <= 0;
                        Mem_Write   <= 0;
                        Mem_to_Reg  <= 0;
                        Imm_Value   <= 32'hxxxxxxxx;
                        cnn_en      <= 1'b0;
                    end
                7'b0000011:         // I-type instructions (LW)
                    begin
                        Reg_Write   <= 1;
                        ALU_Src     <= 1;
                        Mem_Read    <= 1;
                        Mem_Write   <= 0;
                        Mem_to_Reg  <= 1;
                        Imm_Value[31:0] <= {{21{Instruction_IF_ID[31]}}, Instruction_IF_ID[30:20]};
                        cnn_en      <= 1'b0;
                    end
                7'b0010011:         // I-type instructions (ADDI)
                    begin 
                        Reg_Write   <= 1;
                        ALU_Src     <= 1;
                        Mem_Read    <= 0;
                        Mem_Write   <= 0;
                        Mem_to_Reg  <= 0;
                        Imm_Value[31:0] <= {{21{Instruction_IF_ID[31]}}, Instruction_IF_ID[30:20]};
                        cnn_en      <= 1'b0;
                    end
                7'b0100011:         // S-type instructions
                    begin 
                        Reg_Write   <= 0;
                        ALU_Src     <= 1;
                        Mem_Read    <= 0;
                        Mem_Write   <= 1;
                        Mem_to_Reg  <= 1'bx;
                        Imm_Value[31:0] <= {{21{Instruction_IF_ID[31]}}, Instruction_IF_ID[30:25], Instruction_IF_ID[11:7]};
                        cnn_en      <= 1'b0;
                    end
                7'b1100011:         // B-types instructions
                    begin 
                        Reg_Write   <= 0;
                        ALU_Src     <= 0;
                        Mem_Read    <= 0;
                        Mem_Write   <= 0;
                        Mem_to_Reg  <= 1'bx;
                        Imm_Value   <= 32'hxxxxxxxx;
                        cnn_en      <= 1'b0;
                    end
                7'b1101111:         // J-type instructions
                    begin 
                        Reg_Write   <= 1;
                        ALU_Src     <= 1'bx;
                        Mem_Read    <= 0;
                        Mem_Write   <= 0;
                        Mem_to_Reg  <= 1'bx;
                        Imm_Value   <= 32'hxxxxxxxx;
                        cnn_en      <= 1'b0;
                    end
                7'b1110011:         // ecall
                    begin
                        Reg_Write   <= 1'bx;
                        ALU_Src     <= 1'bx;
                        Mem_Read    <= 1'bx;
                        Mem_Write   <= 1'b0;
                        Mem_to_Reg  <= 1'bx;
                        Imm_Value   <= 32'hxxxxxxxx;
                        cnn_en      <= 1'b0;
                    end
                7'b1111111:         // Convo
                    begin
                        Reg_Write   <= 1'bx;
                        ALU_Src     <= 1'bx;
                        Mem_Read    <= 1'bx;
                        Mem_Write   <= 1'b0;
                        Mem_to_Reg  <= 1'bx;
                        Imm_Value   <= 32'hxxxxxxxx;
                        cnn_en      <= 1'b1;
                    end
                default:
                    begin
                        Reg_Write   <= 1'bx;
                        ALU_Src     <= 1'bx;
                        Mem_Read    <= 1'bx;
                        Mem_Write   <= 1'bx;
                        Mem_to_Reg  <= 1'bx;
                        Imm_Value   <= 32'hxxxxxxxx;
                        cnn_en      <= 1'bx;
                    end
            endcase
        end
    end

    // ALU_Op decode
    always @(*) begin
        if (Signal_Stall) begin
            ALU_Op = 4'b0000;
        end

        else begin
            casez (Control)
                11'b0_000_0110011: ALU_Op = 4'b0000; //add
                11'b1_000_0110011: ALU_Op = 4'b0001; //sub
                11'b0_001_0110011: ALU_Op = 4'b0010; //sll
                11'b0_101_0110011: ALU_Op = 4'b0011; //srl
                11'b0_010_0110011: ALU_Op = 4'b0100; //slt
                11'b?_000_0010011: ALU_Op = 4'b0000; //addi
                11'b0_110_0110011: ALU_Op = 4'b0101; //or
                11'b0_111_0110011: ALU_Op = 4'b0110; //and
                11'b?_010_0000011: ALU_Op = 4'b0000; //lw
                11'b?_010_0100011: ALU_Op = 4'b0000; //sw
                11'b?_000_1100011: ALU_Op = 4'b0001; //beq
                11'b?_100_1100011: ALU_Op = 4'b0001; //blt
                11'b?_???_1101111: ALU_Op = 4'bxxxx; //jal
                11'b0_000_1110011: ALU_Op = 4'bxxxx; //ecall
                default: ALU_Op = 4'bxxxx;
            endcase
        end
    end
    
endmodule

//Verified and rst check
(* dont_touch = "true" *)
module ID_EX_reg (
    // Inputs
    // Control Signals
    input wire clk_50MHZ,
    input wire rst,
    input wire Signal_Flush,            // Pipeline flush
    input wire Signal_Branch_IF_ID,     // Branch
    input wire Signal_Jump_IF_ID,       // Jump
    input wire Signal_Stall,            // Stall
    input wire [1:0] Addr_IF_ID,
    input wire [1:0] State_IF_ID,
    input wire [3:0] ALU_Op,
    input wire Reg_Write,
    input wire ALU_Src,
    input wire Mem_Read,
    input wire Mem_Write,
    input wire Mem_to_Reg,

    // Data Signals
    input wire [4:0] rs1,
    input wire [4:0] rs2,
    input wire [4:0] rd,

    input wire [31:0] PC_IF_ID,
    input wire [31:0] RF_Out1,
    input wire [31:0] RF_Out2,
    input wire [31:0] Jump_Addr_IF_ID,
    input wire [31:0] Return_Addr_IF_ID,
    input wire Instruction_14_IF_ID,
    input wire [31:0] Imm_Value,

    // Outputs
    // Control Signals
    output reg Signal_Branch_ID_EX,
    output reg Signal_Jump_ID_EX,
    output reg [1:0] Addr_ID_EX,
    output reg [1:0] State_ID_EX,
    output reg [3:0] ALU_Op_ID_EX,
    output reg Reg_Write_ID_EX,
    output reg ALU_Src_ID_EX,
    output reg Mem_Read_ID_EX,
    output reg Mem_Write_ID_EX,
    output reg Mem_to_Reg_ID_EX,

    // Data Signals
    output reg [4:0] rs1_ID_EX,
    output reg [4:0] rs2_ID_EX,
    output reg [4:0] rd_ID_EX,

    output reg [31:0] PC_ID_EX,
    output reg [31:0] RF_Out1_ID_EX,
    output reg [31:0] RF_Out2_ID_EX,
    output reg [31:0] Return_Addr_ID_EX,
    output reg Instruction_14_ID_EX,
    output reg [31:0] Imm_Value_ID_EX
);

    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------
    always @(posedge clk_50MHZ) begin
        if (rst == 1) begin
            // Control Signals
            Signal_Branch_ID_EX <= 0;
            Signal_Jump_ID_EX   <= 0;
            Addr_ID_EX          <= 0;
            State_ID_EX         <= 0;
            ALU_Op_ID_EX        <= 0;
            Reg_Write_ID_EX     <= 0;
            ALU_Src_ID_EX       <= 0;
            Mem_Read_ID_EX      <= 0;
            Mem_Write_ID_EX     <= 0;
            Mem_to_Reg_ID_EX    <= 0;

            // Data Signals
            rs1_ID_EX           <= 0;
            rs2_ID_EX           <= 0;
            rd_ID_EX            <= 0;

            PC_ID_EX            <= 0;
            RF_Out1_ID_EX       <= 0;
            RF_Out2_ID_EX       <= 0;
            Return_Addr_ID_EX   <= 0;
            Instruction_14_ID_EX<= 0;
            Imm_Value_ID_EX     <= 0;
        end

        else if (Signal_Flush==1) begin
            // Control Signals
            Signal_Branch_ID_EX <= 0;
            Signal_Jump_ID_EX   <= 0;
            Addr_ID_EX          <= 0;
            State_ID_EX         <= 0;
            ALU_Op_ID_EX        <= 0;
            Reg_Write_ID_EX     <= 0;
            ALU_Src_ID_EX       <= 0;
            Mem_Read_ID_EX      <= 0;
            Mem_Write_ID_EX     <= 0;
            Mem_to_Reg_ID_EX    <= 0;

            // Data Signals
            rs1_ID_EX           <= 0;
            rs2_ID_EX           <= 0;
            rd_ID_EX            <= 0;

            PC_ID_EX            <= 0;
            RF_Out1_ID_EX       <= 0;
            RF_Out2_ID_EX       <= 0;
            Return_Addr_ID_EX   <= 0;
            Instruction_14_ID_EX<= 0;
            Imm_Value_ID_EX     <= 0;
        end

        else if (Signal_Stall==1) begin
            // Control Signals
            Signal_Branch_ID_EX <= 0;
            Signal_Jump_ID_EX   <= 0;
            Addr_ID_EX          <= 0;
            State_ID_EX         <= 0;
            ALU_Op_ID_EX        <= 0;
            Reg_Write_ID_EX     <= 0;
            ALU_Src_ID_EX       <= 0;
            Mem_Read_ID_EX      <= 0;
            Mem_Write_ID_EX     <= 0;
            Mem_to_Reg_ID_EX    <= 0;

            // Data Signals
            rs1_ID_EX           <= rs1_ID_EX;
            rs2_ID_EX           <= rs2_ID_EX;
            rd_ID_EX            <= rd_ID_EX;

            PC_ID_EX            <= PC_ID_EX;
            RF_Out1_ID_EX       <= RF_Out1_ID_EX;
            RF_Out2_ID_EX       <= RF_Out2_ID_EX;
            Return_Addr_ID_EX   <= Return_Addr_ID_EX;
            Instruction_14_ID_EX<= Instruction_14_ID_EX;
            Imm_Value_ID_EX     <= Imm_Value_ID_EX;
        end
        
        else begin
            // Control Signals
            Signal_Branch_ID_EX <= Signal_Branch_IF_ID;
            Signal_Jump_ID_EX   <= Signal_Jump_IF_ID;
            Addr_ID_EX          <= Addr_IF_ID;
            State_ID_EX         <= State_IF_ID;
            ALU_Op_ID_EX        <= ALU_Op;
            Reg_Write_ID_EX     <= Reg_Write;
            ALU_Src_ID_EX       <= ALU_Src;
            Mem_Read_ID_EX      <= Mem_Read;
            Mem_Write_ID_EX     <= Mem_Write;
            Mem_to_Reg_ID_EX    <= Mem_to_Reg;

            // Data Signals
            rs1_ID_EX           <= rs1;
            rs2_ID_EX           <= rs2;
            rd_ID_EX            <= rd;

            PC_ID_EX            <= PC_IF_ID + Jump_Addr_IF_ID;
            RF_Out1_ID_EX       <= RF_Out1;
            RF_Out2_ID_EX       <= RF_Out2;
            Return_Addr_ID_EX   <= Return_Addr_IF_ID;
            Instruction_14_ID_EX<= Instruction_14_IF_ID;
            Imm_Value_ID_EX     <= Imm_Value;
        end
    end

endmodule

//Verified and Changed,  Define In1,2 in Intantiaton
(* dont_touch = "true" *)
module ALU (
    // Inputs
    input wire [3:0] ALU_Op_ID_EX,
    input wire [31:0] ALU_In1_ID_EX, ALU_In2_ID_EX,
    input wire [1:0] Forward_A, Forward_B,
    input wire ALU_Src_ID_EX,
    input wire [31:0] ALU_Result_EX_MEM, Reg_Write_Data_MEM_WB,
    input wire [31:0] Imm_Value_ID_EX,

    // Outputs
    output reg Zero_Flag, Sign_Flag,
    output reg [31:0] ALU_Result
);

    //-------------------------------------------------------------
    // Registers / Wires
    //-------------------------------------------------------------

    wire [31:0] ALU_In1, ALU_In2;
    
    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------

    assign ALU_In1 = (Forward_A == 2'b00) ? ALU_In1_ID_EX :
                     (Forward_A == 2'b01) ? Reg_Write_Data_MEM_WB :
                     (Forward_A == 2'b10) ? ALU_Result_EX_MEM :
                     ALU_In1_ID_EX;

    assign ALU_In2 = ALU_Src_ID_EX ? Imm_Value_ID_EX :((Forward_B == 2'b00) ? ALU_In2_ID_EX :
                                                       (Forward_B == 2'b01) ? Reg_Write_Data_MEM_WB :
                                                       (Forward_B == 2'b10) ? ALU_Result_EX_MEM : ALU_In2_ID_EX);

     

    always @(*) begin
        case (ALU_Op_ID_EX)
            4'b0000: ALU_Result = ALU_In1 + ALU_In2;              //add, addi, lw, sw
            4'b0001: ALU_Result = ALU_In1 - ALU_In2;              //sub, beq, blt
            4'b0010: ALU_Result = ALU_In1 << ALU_In2[4:0];        //sll
            4'b0011: ALU_Result = ALU_In1 >> ALU_In2[4:0];        //srl
            4'b0100: ALU_Result = (ALU_In1 < ALU_In2) ? 1 : 0;    //slt
            4'b0101: ALU_Result = ALU_In1 | ALU_In2;              //or
            4'b0110: ALU_Result = ALU_In1 & ALU_In2;              //and
            default: ALU_Result = 32'h00000000;
        endcase
    end

    always @(ALU_Result) begin
        if (ALU_Result==0) begin
            Zero_Flag = 1;
            Sign_Flag = 0;
        end

        else if (ALU_Result[31]==1) begin
            Zero_Flag = 0;
            Sign_Flag = 1;
        end

        else begin 
            Zero_Flag = 0;
            Sign_Flag = 0;
        end
    end
endmodule 


//Verified
(* dont_touch = "true" *)
module EX_MEM_Reg (
    // Inputs
    // Control Signals
    input wire clk_50MHZ,
    input wire rst,
    input wire Signal_Jump_ID_EX,
    input wire Reg_Write_ID_EX,
    input wire Mem_Read_ID_EX,
    input wire Mem_Write_ID_EX,
    input wire Mem_to_Reg_ID_EX,

    // Data Signals
    input wire [4:0]  rd_ID_EX,
    input wire [31:0] ALU_Result,
    input wire [31:0] RF_Out2_ID_EX,
    input wire [31:0] Return_Addr_ID_EX,

    // Outputs
    // Control Signals
    output reg Signal_Jump_EX_MEM,
    output reg Reg_Write_EX_MEM,
    output reg Mem_Read_EX_MEM,
    output reg Mem_Write_EX_MEM,
    output reg Mem_to_Reg_EX_MEM,

    // Data Signals
    output reg [4:0] rd_EX_MEM,
    output reg [31:0] ALU_Result_EX_MEM,
    output reg [31:0] RF_Out2_EX_MEM,
    output reg [31:0] Return_Addr_EX_MEM
);
    
    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------
    always @(posedge clk_50MHZ, posedge rst) begin
        if (rst==1) begin

            // Control Signals
            Signal_Jump_EX_MEM  <= 0;
            Reg_Write_EX_MEM    <= 0;
            Mem_Read_EX_MEM     <= 0;
            Mem_Write_EX_MEM    <= 0;
            Mem_to_Reg_EX_MEM   <= 0;

            // Data Signals
            rd_EX_MEM           <= 0;
            ALU_Result_EX_MEM   <= 0;
            RF_Out2_EX_MEM      <= 0;
            Return_Addr_EX_MEM  <= 0;
        end

        else begin

            // Control Signals
            Signal_Jump_EX_MEM  <= Signal_Jump_ID_EX;
            Reg_Write_EX_MEM    <= Reg_Write_ID_EX;
            Mem_Read_EX_MEM     <= Mem_Read_ID_EX;
            Mem_Write_EX_MEM    <= Mem_Write_ID_EX;
            Mem_to_Reg_EX_MEM   <= Mem_to_Reg_ID_EX;

            // Data Signals
            rd_EX_MEM           <= rd_ID_EX;
            ALU_Result_EX_MEM   <= ALU_Result;
            RF_Out2_EX_MEM      <= RF_Out2_ID_EX;
            Return_Addr_EX_MEM  <= Return_Addr_ID_EX;
        end
    end

endmodule

//Verified
(* dont_touch = "true" *)
module Data_Memory (
    // Inputs
    input wire clk_50MHZ,
    input wire Mem_Write_EX_MEM,
    input wire Mem_Read_EX_MEM,

    input wire [31:0] Mem_Write_Data,
    input wire [31:0] Mem_Address, // Limit the size of Mem_Address to 5 bits

    // Outputs
    output reg [31:0] Mem_Read_Data,
    output [31:0] Mem_LED_out
);
    //-------------------------------------------------------------
    // Registers / Wires
    //-------------------------------------------------------------
    reg [7:0] mem [19:0];

    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------
    
    initial begin
            mem[3]  <= 0; mem[2]  <= 0; mem[1]  <= 0; mem[0]  <= 9; 
            mem[7]  <= 0; mem[6]  <= 0; mem[5]  <= 0; mem[4]  <= 3; 
            mem[11] <= 0; mem[10] <= 0; mem[9]  <= 0; mem[8]  <= 0;
            mem[15] <= 0; mem[14] <= 0; mem[13] <= 0; mem[12] <= 0;
            mem[19] <= 0; mem[18] <= 0; mem[17] <= 0; mem[16] <= 0;
    end
    
    assign Mem_LED_out = {mem[11], mem[10], mem[9],  mem[8]};

    always @(negedge clk_50MHZ) begin
        // Write operation
            if (Mem_Write_EX_MEM == 1) begin
                mem[Mem_Address]    <= Mem_Write_Data[7:0];
                mem[Mem_Address+1]  <= Mem_Write_Data[15:8];
                mem[Mem_Address+2]  <= Mem_Write_Data[23:16];
                mem[Mem_Address+3]  <= Mem_Write_Data[31:24];
            end
            // Read operation
            else if (Mem_Read_EX_MEM == 1) begin
                Mem_Read_Data <= {mem[Mem_Address+3], mem[Mem_Address+2], mem[Mem_Address+1], mem[Mem_Address]};
            end
            // Default value
            else begin
                Mem_Read_Data <= 32'd0;
            end
        end

endmodule


//Verified
(* dont_touch = "true" *)
module MEM_WB_Reg (
    // Inputs
    // Control Signals
    input wire clk_50MHZ,
    input wire rst,
    input wire Signal_Jump_EX_MEM,
    input wire Reg_Write_EX_MEM,
    input wire Mem_to_Reg_EX_MEM,

    // Data Signals
    input wire [4:0] rd_EX_MEM,
    input wire [31:0] ALU_Result_EX_MEM,
    input wire [31:0] Mem_Read_Data,
    input wire [31:0] Return_Addr_EX_MEM,

    // Outputs
    // Control Signals
    output reg Signal_Jump_MEM_WB,
    output reg Reg_Write_MEM_WB,
    output reg Mem_to_Reg_MEM_WB,

    // Data Signals
    output reg [4:0] rd_MEM_WB,
    output reg [31:0] ALU_Result_MEM_WB,
    output reg [31:0] Mem_Read_Data_MEM_WB,
    output reg [31:0] Return_Addr_MEM_WB
);

    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------
    always @(posedge clk_50MHZ, posedge rst) begin
        if (rst==1) begin

            // Control Signals
            Signal_Jump_MEM_WB  <= 0;
            Reg_Write_MEM_WB    <= 0;
            Mem_to_Reg_MEM_WB   <= 0;

            // Data Signals
            rd_MEM_WB           <= 0;
            ALU_Result_MEM_WB   <= 0;
            Mem_Read_Data_MEM_WB<= 0;
            Return_Addr_MEM_WB  <= 0;
        end

        else begin
            
            // Control Signals
            Signal_Jump_MEM_WB  <= Signal_Jump_EX_MEM;
            Reg_Write_MEM_WB    <= Reg_Write_EX_MEM;
            Mem_to_Reg_MEM_WB   <= Mem_to_Reg_EX_MEM;

            // Data Signals
            rd_MEM_WB           <= rd_EX_MEM;
            ALU_Result_MEM_WB   <= ALU_Result_EX_MEM;
            Mem_Read_Data_MEM_WB<= Mem_Read_Data;
            Return_Addr_MEM_WB  <= Return_Addr_EX_MEM;
        end
    end

endmodule

(* dont_touch = "true" *)
module Forwarding_Unit (
    // Inputs
    input wire Reg_Write_EX_MEM,
    input wire Reg_Write_MEM_WB,
    input wire [4:0] rd_EX_MEM,
    input wire [4:0] rd_MEM_WB,
    input wire [4:0] rs1_ID_EX,
    input wire [4:0] rs2_ID_EX,

    // Outputs
    output reg [1:0] Forward_A,
    output reg [1:0] Forward_B
);

    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------
    always @(*) begin
        // EX hazard
        if (Reg_Write_EX_MEM && (rd_EX_MEM != 0) && (rd_EX_MEM == rs1_ID_EX))
            Forward_A = 2'b10;
        else if (Reg_Write_MEM_WB && (rd_MEM_WB != 0) && (rd_MEM_WB == rs1_ID_EX))
            Forward_A = 2'b01;
        else
            Forward_A = 2'b00;

        // MEM hazard
        if (Reg_Write_EX_MEM && (rd_EX_MEM != 0) && (rd_EX_MEM == rs2_ID_EX))
            Forward_B = 2'b10;
        else if (Reg_Write_MEM_WB && (rd_MEM_WB != 0) && (rd_MEM_WB == rs2_ID_EX))
            Forward_B = 2'b01;
        else
            Forward_B = 2'b00;
    end

endmodule

(* dont_touch = "true" *)
module Hazard_Detection_Unit (
    // Inputs
    input wire rst,
    input wire Mem_Read_ID_EX,
    input wire [4:0] rd_ID_EX,
    input wire [4:0] rs1_IF_ID,
    input wire [4:0] rs2_IF_ID,

    // Outputs
    output reg Signal_Stall
);

    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------    
    always @(*) begin
        if (rst==1)
            Signal_Stall = 1'b0;
        else if (Mem_Read_ID_EX && ((rd_ID_EX == rs1_IF_ID) || (rd_ID_EX == rs2_IF_ID))) begin
            Signal_Stall = 1'b1;
        end
        else begin
            Signal_Stall = 1'b0;
        end
    end

endmodule

(* dont_touch = "true" *)
module Branch_Table (
    // Inputs
    input wire clk_50MHZ,
    input wire rst,
    input wire Signal_Branch_ID_EX,
    input wire [1:0] Addr,
    input wire [1:0] Addr_ID_EX,
    input wire [1:0] Entry,

    // Outputs
    output wire [1:0] State
);
    //-------------------------------------------------------------
    // Registers / Wires
    //-------------------------------------------------------------
    reg [1:0] BranchTable[3:0];

    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------
    always @(posedge clk_50MHZ) begin
        if (rst == 1) begin
            // Initialize BranchTable during rst
            BranchTable[0] <= 2'b00;
            BranchTable[1] <= 2'b00;
            BranchTable[2] <= 2'b00;
            BranchTable[3] <= 2'b00;
        end else begin
            // Write operation
            if (Signal_Branch_ID_EX == 1)
                BranchTable[Addr_ID_EX] <= Entry;
        end
    end

    assign State = BranchTable[Addr];
endmodule

(* dont_touch = "true" *)
module Branch_Predictor (
    // Inputs
    input wire Outcome,
    input wire [1:0] State_ID_EX,

    // Outputs
    output reg [1:0] Entry,
    output wire Signal_Flush
);

    //-------------------------------------------------------------
    // Registers / Wires
    //-------------------------------------------------------------
    parameter s0=0, s1=1, s2=2, s3=3;
    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------
    assign Signal_Flush = State_ID_EX[1] ^ Outcome; // Assigning Flush
    
    always @(*) begin
        case(State_ID_EX)
        s0: Entry = Outcome ? s1 : s0;
        s1: Entry = Outcome ? s2 : s0;
        s2: Entry = Outcome ? s3 : s1;
        s3: Entry = Outcome ? s3 : s2;
        endcase
    end

endmodule

(* dont_touch = "true" *)
module Writeback_Unit (
    // inputs
    input wire Mem_to_Reg_MEM_WB,
    input wire Signal_Jump_MEM_WB,
    input wire [31:0] ALU_Result_MEM_WB,
    input wire [31:0] Mem_Read_Data_MEM_WB,
    input wire [31:0] Return_Addr_MEM_WB,
    
    // Outputs
    output reg [31:0] Reg_Write_Data
);
    
    //-------------------------------------------------------------
    // Functionality
    //-------------------------------------------------------------
    always @(*) begin
        Reg_Write_Data = Signal_Jump_MEM_WB ? Return_Addr_MEM_WB : (Mem_to_Reg_MEM_WB ? Mem_Read_Data_MEM_WB : ALU_Result_MEM_WB);
    end
    
endmodule

(* dont_touch = "true" *)
module Eight_Digit_Hex_Display(
    input clk_50MHZ,
    input rst,
    input [27:0] Mem_LED_in,
    input [3:0] predicted_class_LED,
    output reg [7:0] Anode_Activate,
    output reg [7:0] LED_out
);
    reg [26:0] one_second_counter;
    wire one_second_enable;
    reg [31:0] displayed_value;
    reg [3:0] LED_hex;
    reg [18:0] refresh_counter;
    wire [2:0] LED_activating_counter;

    always @(posedge clk_50MHZ or posedge rst) begin
        if (rst)
            one_second_counter <= 0;
        else begin
            if (one_second_counter >= 99999999)
                one_second_counter <= 0;
            else
                one_second_counter <= one_second_counter + 1;
        end
    end

    assign one_second_enable = (one_second_counter == 99999999) ? 1 : 0;

    always @(posedge clk_50MHZ or posedge rst) begin
        if (rst)
            displayed_value <= 0;
        else if (one_second_enable)
            displayed_value <= {predicted_class_LED, Mem_LED_in};
    end

    always @(posedge clk_50MHZ or posedge rst) begin
        if (rst)
            refresh_counter <= 0;
        else
            refresh_counter <= refresh_counter + 1;
    end

    assign LED_activating_counter = refresh_counter[18:16];

    always @(*) begin
        case (LED_activating_counter)
            3'b000: begin
                Anode_Activate = 8'b11111110;
                LED_hex = displayed_value[3:0];
            end
            3'b001: begin
                Anode_Activate = 8'b11111101;
                LED_hex = displayed_value[7:4];
            end
            3'b010: begin
                Anode_Activate = 8'b11111011;
                LED_hex = displayed_value[11:8];
            end
            3'b011: begin
                Anode_Activate = 8'b11110111;
                LED_hex = displayed_value[15:12];
            end
            3'b100: begin
                Anode_Activate = 8'b11101111;
                LED_hex = displayed_value[19:16];
            end
            3'b101: begin
                Anode_Activate = 8'b11011111;
                LED_hex = displayed_value[23:20];
            end
            3'b110: begin
                Anode_Activate = 8'b10111111;
                LED_hex = displayed_value[27:24];
            end
            3'b111: begin
                Anode_Activate = 8'b01111111;
                LED_hex = displayed_value[31:28];
            end
        endcase
    end

    always @(*) begin
        case (LED_hex)
            4'h0: LED_out = 8'b00000011; //a,b,c,d,e,f,g,dot (zero)
            4'h1: LED_out = 8'b10011111; //one
            4'h2: LED_out = 8'b00100101; //two
            4'h3: LED_out = 8'b00001101; //three
            4'h4: LED_out = 8'b10011001; //four
            4'h5: LED_out = 8'b01001001; //five
            4'h6: LED_out = 8'b01000001; //six
            4'h7: LED_out = 8'b00011111; //seven
            4'h8: LED_out = 8'b00000001; //eight
            4'h9: LED_out = 8'b00001001; //nine
            4'hA: LED_out = 8'b00010001; //A
            4'hB: LED_out = 8'b11000001; //b
            4'hC: LED_out = 8'b01100011; //C
            4'hD: LED_out = 8'b10000101; //d
            4'hE: LED_out = 8'b01100001; //E
            4'hF: LED_out = 8'b01110001; //F
            default: LED_out = 8'b00000011;
        endcase
    end
endmodule

(* dont_touch = "true" *)
module cnn_core (
    input clk_20MHZ,
    input rst,
    input cnn_en,
    output reg [3:0] predicted_class_LED
);

    wire [80:0]  buffer_data_out;
    wire [15:0]  conv_data_out_1;
    wire [239:0] pool_data_out_1;
    wire [207:0] conv_data_out_2;
    wire [95:0]  pool_data_out_2;
    wire [159:0] fc_data_out;
    wire [3:0]   predicted_class;

    wire window_valid;
    wire conv_out_valid_1;
    wire pool_out_valid_1;
    wire conv_out_valid_2;
    wire pool_out_valid_2;
    wire fc_out_valid;
    wire predicted_out_valid;
    wire cnn_en_int;

    clock_cycle_counter t1(
        .clk_20MHZ(clk_20MHZ),
        .cnn_en(cnn_en),
        .cnn_en_int(cnn_en_int)
    );

    Data_Buffer m1(
        .clk_20MHZ(clk_20MHZ),
        .rst(rst),
        .cnn_en_int(cnn_en_int),
        .window_valid(window_valid),
        .buffer_data_out(buffer_data_out)
    );

    Convolution_1 m2(
        .clk_20MHZ(clk_20MHZ),
        .rst(rst),
        .cnn_en_int(cnn_en_int),
        .window_valid(window_valid),
        .buffer_data_out(buffer_data_out),
        .conv_out_valid_1(conv_out_valid_1),
        .conv_data_out_1(conv_data_out_1)
    );

    Max_Pooling_1 m3(
        .clk_20MHZ(clk_20MHZ),
        .rst(rst),
        .cnn_en_int(cnn_en_int),
        .conv_out_valid_1(conv_out_valid_1),
        .conv_data_out_1(conv_data_out_1),
        .pool_data_out_1(pool_data_out_1),
        .pool_out_valid_1(pool_out_valid_1)
        
    );

    Convolution_2 m4(
        .clk_20MHZ(clk_20MHZ),
        .rst(rst),
        .cnn_en_int(cnn_en_int),
        .pool_out_valid_1(pool_out_valid_1),
        .pool_data_out_1(pool_data_out_1),
        .conv_data_out_2(conv_data_out_2),
        .conv_out_valid_2(conv_out_valid_2)
    );

    Max_Pooling_2 m5(
        .clk_20MHZ(clk_20MHZ),
        .rst(rst),
        .cnn_en_int(cnn_en_int),
        .conv_out_valid_2(conv_out_valid_2),
        .conv_data_out_2(conv_data_out_2),
        .pool_data_out_2(pool_data_out_2),
        .pool_out_valid_2(pool_out_valid_2)
    );

    Fully_Connected m6(
        .clk_20MHZ(clk_20MHZ),
        .rst(rst),
        .cnn_en_int(cnn_en_int),
        .pool_out_valid_2(pool_out_valid_2),
        .pool_data_out_2(pool_data_out_2),
        .fc_data_out(fc_data_out),
        .fc_out_valid(fc_out_valid)
    );

    predicted_class m7(
        .clk_20MHZ(clk_20MHZ),
        .rst(rst),
        .cnn_en_int(cnn_en_int),
        .fc_out_valid(fc_out_valid),
        .fc_data_out(fc_data_out),
        .predicted_class(predicted_class),
        .predicted_out_valid(predicted_out_valid)
    );

    always @(posedge predicted_out_valid) begin
        predicted_class_LED <= predicted_class;
    end
endmodule

(* dont_touch = "true" *)
module clock_cycle_counter(
    input clk_20MHZ,
    input cnn_en,
    output reg cnn_en_int
);

    parameter TARGET_CYCLES = 1087; // Set the desired number of clock cycles

    reg [15:0] counter; // 4-bit counter to count clock cycles

    always @(posedge clk_20MHZ or posedge cnn_en) begin
        if (cnn_en) begin
            counter <= 0;
            cnn_en_int <= 1;
        end else begin
            if (counter == TARGET_CYCLES - 1) begin
                counter <= 0;
                cnn_en_int <= 0;
            end else begin
                counter <= counter + 1;
                cnn_en_int <= cnn_en_int;
            end
        end
    end

endmodule


(* dont_touch = "true" *)
module Data_Buffer (
    input clk_20MHZ,
    input rst,
    input cnn_en_int,
    output reg window_valid,
    output reg [80:0] buffer_data_out
);
    //(*ram_style = "block"*) 
    reg [7:0] buffer [1023:0];
    reg [4:0] row;
    reg [4:0] col;

    integer i,j;

    initial begin
        $readmemh("cnn.mem", buffer);
    end

    always @(posedge clk_20MHZ) begin

        if(rst == 1) begin
            window_valid <= 0;
            buffer_data_out <= 0;
            row = 0;
            col = 0;      
        end

        else if(cnn_en_int) begin
            if (row == 29 && col == 29) begin
                row <= 0;
                col <= 0;
            end
            else if (col == 29) begin
                row <= row + 1;
                col <= 0;
            end
            else begin
                col <= col + 1;
            end

            for (i=0; i<3; i=i+1) begin
                for (j=0; j<3; j=j+1) begin
                    buffer_data_out[(i*3+j)*9+8 -:9] = {{1'b0}, buffer[((($unsigned(i)+$unsigned(row))*32)+($unsigned(j)+$unsigned(col)))]};

                end        
            end
            window_valid = 1;                
        end

        else begin
            buffer_data_out <= 0; 
        end
    end
endmodule

(* dont_touch = "true" *)
module Convolution_1 (
    input clk_20MHZ,
    input rst,
    input cnn_en_int,
    input window_valid,
    input [80:0] buffer_data_out,
    output reg conv_out_valid_1,
    output reg [15:0] conv_data_out_1
);
    
    reg signed[15:0] kernel [8:0];
    reg signed [31:0] conv_output;
    integer i,j;
    
    initial begin
        $readmemh("kernel1.mem",kernel);
    end
    
    always @(posedge clk_20MHZ) begin
        if (rst==1) begin
            conv_data_out_1 <= 0; 
            conv_out_valid_1 <= 0; 
        end

        else if (cnn_en_int && window_valid) begin
            conv_output = 0;
            for (i=0;i<3;i=i+1) begin
                for (j=0;j<3;j=j+1) begin
                    conv_output = conv_output + ($signed(buffer_data_out[((i*3)+j)*9+8 -: 9]) * kernel[(i)*3+(j)]);
                end    
            end
            conv_data_out_1 <= (conv_output >= 0) ? conv_output:0;
            conv_out_valid_1 <= 1;
        end
        
        else begin
            conv_data_out_1 <= 0;
            conv_out_valid_1 <= 0; 
        end
    end
endmodule

(* dont_touch = "true" *)
module Max_Pooling_1 (
    input clk_20MHZ,
    input rst,
    input cnn_en_int,
    input conv_out_valid_1,
    input [15:0] conv_data_out_1,
    output reg [239:0] pool_data_out_1,
    output reg pool_out_valid_1
);
    
    reg signed [15:0] buffer [0:59];
    reg [4:0] row, col;
    
    integer i,j;

    function [7:0] max_value;
        input [7:0] a, b, c, d;
        begin
            max_value = (a > b) ? ((a > c) ? ((a > d) ? a : d) : ((c > d) ? c : d)) : ((b > c) ? ((b > d) ? b : d) : ((c > d) ? c : d));
        end
    endfunction

    always @(posedge clk_20MHZ) begin
        if (rst) begin
            pool_out_valid_1 <= 0;
            row <= 0;
            col <= 0;
            pool_data_out_1 <= 0;
            for (i=0;i<60;i=i+1) begin
                buffer[i] = 0;
            end
        end        
        
        else if (cnn_en_int && conv_out_valid_1) begin
            buffer[row * 30 + col] <= conv_data_out_1;

            if (row == 1 && col == 29) begin
                // Reset counters and max_val
                row <= 0;
                col <= 0;
            end
            else if (col == 29) begin
                // Move to next row
                row <= row + 1;
                col <= 0;
            end
            else begin
                // Move to next column
                col <= col + 1;
            end

            if (row == 1 && col == 29) begin
                for (j=0;j<30;j=j+2) begin
                    pool_data_out_1[(j/2)*16+15 -:16] = max_value(
                    buffer[((j))],
                    buffer[((j+1))],
                    buffer[((30)+(j))],
                    buffer[((30)+(j+1))]
                );
                end
                pool_out_valid_1 = 1;
            end
            else begin
                pool_out_valid_1 = 0;
            end
        end
        else begin
            pool_data_out_1 <= 0;
            pool_out_valid_1 = 0;
        end

    end
endmodule

(* dont_touch = "true" *)
module Convolution_2 (
    input clk_20MHZ,
    input rst,
    input cnn_en_int,
    input pool_out_valid_1,
    input [239:0] pool_data_out_1,
    output reg [207:0] conv_data_out_2,
    output reg conv_out_valid_2
);
    reg signed [15:0] buffer [224:0];
    reg signed [15:0] kernel [8:0];
    reg signed [31:0] conv_output;
    reg [4:0] row;

    integer i,j,k;

    initial begin
        $readmemh("kernel2.mem", kernel);
    end
    
    always @(posedge clk_20MHZ) begin
        if (rst) begin
            conv_data_out_2 <= 0;
            conv_out_valid_2 <= 0;
            for (i=0;i<225;i=i+1) begin
                buffer[i] <= 0;
            end
            conv_output <= 0;
            row <= 0;
        end

        else if (cnn_en_int && pool_out_valid_1) begin            
            for(i=0;i<15;i=i+1) begin
                buffer[row*15+$unsigned(i)] <= pool_data_out_1[$unsigned(i)*16+15 -:16];
            end

            if (row == 14) begin
                row <= 0;
            end
            else begin
                row <= row + 1;
            end

            if (row >= 2) begin
                for (k=0;k<13;k=k+1) begin
                    conv_output = 0;
                    for (i=0;i<3;i=i+1) begin
                        for (j=k;j<k+3;j=j+1) begin
                            conv_output = conv_output + (buffer[($unsigned(i)+(row-2))*15+$unsigned(j)]*kernel[(($unsigned(i)))*3+($unsigned(j)-$unsigned(k))]);
                        end
                    end
                    conv_data_out_2[k*16+15 -:16] <= (conv_output>=0) ? conv_output : 0;
                    conv_out_valid_2 <= 1;
                end
            end
            else begin
                conv_out_valid_2 <= 0;
            end
        end

        else begin
            conv_data_out_2 <= 0;
            conv_out_valid_2 <= 0;
        end
    end
endmodule

(* dont_touch = "true" *)
module Max_Pooling_2 (
    input clk_20MHZ,
    input rst,
    input cnn_en_int,
    input conv_out_valid_2,
    input [207:0] conv_data_out_2,
    output reg [95:0] pool_data_out_2,
    output reg pool_out_valid_2    
);
    reg signed [15:0] buffer [25:0];
    reg [4:0] row;
    
    integer i,j;

    function [15:0] max_value;
        input [15:0] a, b, c, d;
        begin
            max_value = (a > b) ? ((a > c) ? ((a > d) ? a : d) : ((c > d) ? c : d)) : ((b > c) ? ((b > d) ? b : d) : ((c > d) ? c : d));
        end
    endfunction

    always @(posedge clk_20MHZ) begin
        if (rst) begin
            pool_data_out_2 <= 0;
            pool_out_valid_2 <= 0;
            for (i=0;i<26;i=i+1) begin
                buffer[i] <= 0;
            end
            row <= 0;
        end
        
        else if (cnn_en_int && conv_out_valid_2) begin
            for(i=0;i<13;i=i+1)begin
                buffer[row*13+$unsigned(i)] = conv_data_out_2[$unsigned(i)*16+15 -:16];
            end
            
            if (row == 1) begin
                row <= 0;
            end
            else begin
                row <= row + 1;
            end
            if (row == 1) begin
                for (j=0;j<6;j=j+1) begin
                    pool_data_out_2[j*16+15 -:16] = max_value(
                    buffer[j*2],
                    buffer[j*2+1],
                    buffer[13+j*2],
                    buffer[13+j*2+1]
                );
                end
                pool_out_valid_2 <= 1;
            end
            else begin
                pool_out_valid_2 <= 0;
            end
        end
        else begin
            pool_out_valid_2 <= 0;
            pool_data_out_2 <= 0;
        end
    end
       
endmodule

(* dont_touch = "true" *)
module Fully_Connected (
    input clk_20MHZ,
    input rst,
    input cnn_en_int,
    input pool_out_valid_2,
    input [95:0] pool_data_out_2,
    output reg [159:0] fc_data_out,
    output reg fc_out_valid
);
    reg signed [15:0] fc_weights [359:0];
    reg signed [15:0] fc_biases [9:0];
    reg signed [31:0] sum;
    reg [4:0] row;
    integer i,j;

    initial begin
        $readmemh("weights.mem", fc_weights);
        $readmemh("biases.mem", fc_biases);
    end

    always @(posedge clk_20MHZ) begin
        if (rst) begin
            fc_data_out <= 0;
            fc_out_valid <= 0;
            sum <= 0;
            row <= 0;
        end

        else if (cnn_en_int && pool_out_valid_2) begin
            if (row == 5) begin
                row <= 0;
            end
            else begin
                row <= row + 1;
            end

            for(i=0;i<10;i=i+1) begin
                for(j=0;j<6;j=j+1) begin
                    sum = sum + ($signed(pool_data_out_2[$unsigned(j)*16+15 -:16])*fc_weights[10*(6*row+$unsigned(j))+$unsigned(i)]);
                end
                fc_data_out[($unsigned(i)*16)+15 -:16] = (row == 5) ? sum + fc_biases[$unsigned(i)] : sum;
                fc_out_valid = 1;
                sum = 0;  
            end
        end
        else begin
            fc_out_valid <= 0;      
            fc_data_out <= 0;
        end
    end

endmodule

(* dont_touch = "true" *)
module predicted_class (
    input clk_20MHZ,
    input rst,
    input cnn_en_int,
    input fc_out_valid,
    input [159:0] fc_data_out,
    output reg [3:0] predicted_class,
    output reg predicted_out_valid
);
    reg signed [15:0] buffer [9:0];
    reg [4:0] row;
    integer i;

    always @(posedge clk_20MHZ) begin
        if (rst) begin
            predicted_class <= 0;
            predicted_out_valid <= 0;
            for(i=0;i<10;i=i+1) begin    
                buffer[i] <= 0;
            end
            row <= 0;
        end

        else if (cnn_en_int && fc_out_valid) begin
            if (row == 6) begin
                predicted_class = 0;
                for (i = 0; i < 10; i=i+1) begin
                    if (buffer[i] > buffer[predicted_class])
                        predicted_class = i;
                end
                predicted_out_valid = 1; 
                for(i=0;i<10;i=i+1) begin
                    buffer[i] = 0;
                end
                row <= 0;
            end
            else begin
                row <= row + 1;
                predicted_out_valid = 0;
                
            end  
            for(i=0;i<10;i=i+1) begin
                buffer[i] = buffer[i] + ($signed(fc_data_out[i*16+15 -:16]));
            end         
        end
        else begin
            predicted_class <= 0;
            predicted_out_valid <= 0; 
        end
    end 
endmodule
