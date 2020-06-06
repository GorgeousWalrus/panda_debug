// ---------------RISCV-Luca---------------
// 
// Module:          writeBack
// 
// Functionality:   Pipeline Stage WriteBack
// 
// -------------(c) Luca Hanel-------------

`include "instructions.sv"

module WB
#(
    BITSIZE
)(
    input                       clk,
    input                       resetn_i,
    //EX-WB
    output                      WB_EX_get_o,
    input                       EX_WB_give_i,
    input [31 : 0]              EX_WB_instruction_i,
    input [BITSIZE - 1 : 0]     EX_WB_d_i,
    //WB-REG
    output [4:0]                WB_REG_rd_o,
    output [BITSIZE - 1 : 0]    WB_REG_d_o,
    output                      WB_REG_access_o
);

enum {GET_INSTR, WB_INSTR} CS, NS;

logic [31 : 0]                                  WB_instruction;

logic [BITSIZE - 1 : 0]                         WB_d;

assign WB_REG_access_o = 'b0;

always_ff@(posedge clk)
begin
    if(!resetn_i)
    begin
       CS <= GET_INSTR; 
    end
    begin
        CS <= NS;
    end
end

always_comb
begin
    WB_EX_get_o     = 1'b0;
    WB_REG_access_o = 1'b1;
    case(CS)
        GET_INSTR: begin
            WB_EX_get_o = 1'b1;
            if(EX_WB_give_i)
            begin
                WB_instruction  = EX_WB_instruction_i;
                WB_d            = EX_WB_d_i;
                NS              = WB_INSTR;
            end
        end

        WB_INSTR: begin
            case(WB_instruction[6:0])
                `LUI, `IMM_REG_ALU, `REG_REG_ALU:
                    WB_REG_rd_o = WB_instruction[11 : 7];
                default:
                    WB_REG_rd_o = 'b0;
            endcase
            WB_REG_d_o = WB_d;
            NS = GET_INSTR;
        end

        default: begin
        end
    endcase
    
end
endmodule