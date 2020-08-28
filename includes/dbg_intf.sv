// ------------------------ Disclaimer -----------------------
// No warranty of correctness, synthesizability or 
// functionality of this code is given.
// Use this code under your own risk.
// When using this code, copy this disclaimer at the top of 
// Your file
//
// (c) Luca Hanel 2020
//
// ------------------------------------------------------------
//
// Module name: dbg_intf
//
// Authors: Luca Hanel
// 
// Functionality: Interface for the debug module
//
// ------------------------------------------------------------
/* verilator lint_off MODDUP */
`ifndef DBG_INTF_SV
`define DBG_INFT_SV

interface dbg_intf#(parameter BITSIZE = 32);
    wire [7:0]                  cmd;
    wire [BITSIZE-1:0]          addr;
    wire [BITSIZE-1:0]          data_dut_dbg;
    wire [BITSIZE-1:0]          data_dbg_dut;
    wire                        dut_done;

    modport dut (
        input cmd, addr, data_dbg_dut,
        output data_dut_dbg, dut_done
    );

    modport dbg (
        input data_dut_dbg, dut_done,
        output cmd, addr, data_dbg_dut
    );
endinterface //dbg_intf

`endif