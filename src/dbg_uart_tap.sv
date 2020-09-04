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
// Module name: dbg_uart_tap
// 
// Functionality: A uart controller for the debug module
//                This is only intended for temporary use, in the
//                future this shall be a JTAG
//
// ------------------------------------------------------------

module dbg_uart_tap#(
`ifdef RTL_TEST
  parameter int BAUDRATE = 10000000,
`else
  parameter int BAUDRATE = 115200,
`endif
  parameter int CLK_FREQ = 50000000
  )(
  input logic             clk,
  input logic             rstn_i,
  // UART
  input logic             rx_i,
  output logic            tx_o,
  // Debug module
  output logic            core_rst_req_o,
  output logic            periph_rst_req_o,
  wb_bus_t.master         wb_bus,
  dbg_intf.dbg            dbg_bus
);

// uart registers
localparam TX_DATA = 0;
localparam RX_DATA = 1;
localparam CLK_DIV = 2;
localparam CTRL = 3;
localparam STATUS = 4;

// control bits
localparam TX_EN = 0;
localparam RX_EN = 1;
localparam PARITY_EN = 2;

// Status bits
localparam TX_EMPTY = 0;
localparam TX_FULL = 1;
localparam RX_EMPTY = 2;
localparam RX_FULL = 3;
localparam RX_ERR = 4;

// DEBUG DEFINES
localparam DBG_END = 8'haa;

// DEBUG signals
logic [2:0]     data_cnt;
logic           incr_cnt;
logic           rst_cnt;
logic           dbg_exec;
logic           dbg_ready;
logic [7:0]     cmd_n, cmd_q, dbg_cmd;
logic [31:0]    addr_n, addr_q;
logic [31:0]    data_w_n, data_w_q;
logic [31:0]    dbg_data_n, dbg_data_q;
logic [31:0]    data_r;

enum logic [2:0] {  IDLE, 
              CMD_DECODE,
              ADDR_READ,
              DATA_READ,
              CMD_EXEC,
              DATA_WRITE,
              CMD_DONE
            } CS, NS;

assign dbg_cmd = (dbg_exec) ? cmd_q : 8'b0;

// UART signals
// clk div for baudrate
localparam logic [31:0] INIT_CLK_DIV = CLK_FREQ/BAUDRATE;

// registers
logic [31:0] uart_regs_n[3:0];
logic [31:0] uart_regs_q[3:0];

// TX signals
logic [7:0]     tx_data;
logic           tx_done;
logic           tx_enable;

// RX signals
logic [7:0]     rx_data;
logic           rx_valid;
logic           rx_parity_err;

// fifo signals
logic           rx_fifo_read;
logic           rx_fifo_write_n, rx_fifo_write_q;
logic [7:0]     rx_data_read;
logic           rx_empty, rx_full;
logic           tx_fifo_read;
logic           tx_fifo_write_n, tx_fifo_write_q;
logic           tx_empty, tx_full;

always_comb
begin
    // Debug
    dbg_exec   = 1'b0;
    NS         = CS;
    cmd_n      = cmd_q;
    addr_n     = addr_q;
    data_w_n   = data_w_q;
    dbg_data_n = dbg_data_q;
    incr_cnt   = 1'b0;
    rst_cnt    = 1'b0;

    uart_regs_n = uart_regs_q;

    // UART tx
    tx_enable   = 1'b0;

    // tx fifo
    tx_fifo_read  = 1'b0;
    tx_fifo_write_n = 1'b0;
    uart_regs_n[STATUS][TX_EMPTY] = tx_empty;
    uart_regs_n[STATUS][TX_FULL] = tx_full;

    // rx fifo
    rx_fifo_read = 1'b0;
    rx_fifo_write_n = 1'b0;
    uart_regs_n[STATUS][RX_EMPTY] = rx_empty;
    uart_regs_n[STATUS][RX_FULL] = rx_full;
    uart_regs_n[RX_DATA] = {24'b0, rx_data_read};

    // UART TX
    if(!tx_empty) begin
        tx_enable = 1'b1;
        if(tx_done) begin
            tx_enable = 1'b0;
            // notify fifo that we processed the data
            tx_fifo_read = 1'b1;
        end
    end

    // UART RX
    if(!rx_full) begin
        if(rx_valid) begin
            rx_fifo_write_n = 1'b1;
        end
    end

    // Debug CTRL
    case(CS)
      IDLE: begin
        if(!rx_empty) begin
          // Read the command
          cmd_n = rx_data_read;
          rx_fifo_read = 1'b1;
          NS = CMD_DECODE;
        end
      end

      CMD_DECODE: begin
        rst_cnt = 1'b1;
        // If a memory operation is performed, read address, else execute command
        if(cmd_q[7])
          NS = ADDR_READ;
        else begin
          NS = CMD_EXEC;
          dbg_exec = 1'b1;
        end
      end

      ADDR_READ: begin
        if(!rx_empty) begin
          // Read address
          if(!data_cnt[2]) begin
            case(data_cnt[1:0])
              2'b00: addr_n[7:0]   = rx_data_read;
              2'b01: addr_n[15:8]  = rx_data_read;
              2'b10: addr_n[23:16] = rx_data_read;
              2'b11: addr_n[31:24] = rx_data_read;
            endcase
            incr_cnt = 1'b1;
            rx_fifo_read = 1'b1;
          end
        end
        // Go to next state
        // If a write operation is performed, read the data from uart, else execute
        if(data_cnt == 3'b100) begin
          rst_cnt = 1'b1;
          if(cmd_q[6])
            NS = DATA_READ;
          else begin
            NS = CMD_EXEC;
            dbg_exec = 1'b1;
          end
        end
      end
      
      DATA_READ: begin
        if(!rx_empty) begin
          // Read address
          if(!data_cnt[2]) begin
            case(data_cnt[1:0])
              2'b00: data_w_n[7:0]   = rx_data_read;
              2'b01: data_w_n[15:8]  = rx_data_read;
              2'b10: data_w_n[23:16] = rx_data_read;
              2'b11: data_w_n[31:24] = rx_data_read;
            endcase
            incr_cnt = 1'b1;
            rx_fifo_read = 1'b1;
          end
        end
        if(data_cnt == 3'b100) begin
          dbg_exec = 1'b1;
          NS = CMD_EXEC;
        end
      end

      CMD_EXEC: begin
        rst_cnt = 1'b1;
        dbg_exec = 1'b1;
        if(dbg_ready) begin
          dbg_exec = 1'b0;
          if(cmd_q[7] && !cmd_q[6]) begin // We wanted to read data, so write it back
            dbg_data_n = data_r;
            NS = DATA_WRITE;
          end else begin // notify that the command was executed
            NS = CMD_DONE;
          end
        end
      end

      DATA_WRITE: begin
        if(!tx_full) begin
          if(!data_cnt[2]) begin
            case(data_cnt[1:0])
              2'b00: uart_regs_n[TX_DATA] = {24'b0, dbg_data_q[7:0]};
              2'b01: uart_regs_n[TX_DATA] = {24'b0, dbg_data_q[15:8]};
              2'b10: uart_regs_n[TX_DATA] = {24'b0, dbg_data_q[23:16]};
              2'b11: uart_regs_n[TX_DATA] = {24'b0, dbg_data_q[31:24]};
            endcase          
            tx_fifo_write_n = 1'b1;
            incr_cnt = 1'b1;
          end
        end
        if(data_cnt == 3'b100)
          NS = CMD_DONE;
        else begin
        end
      end

      CMD_DONE: begin
        rst_cnt = 1'b1;
        if(!tx_full) begin
          uart_regs_n[TX_DATA] = {24'b0, DBG_END};
          tx_fifo_write_n = 1'b1;
          // Go to next state
          NS = IDLE;
        end
      end

      default: begin end
    endcase
end

always_ff @(posedge clk, negedge rstn_i)
begin
  if(!rstn_i) begin
    // UART
    uart_regs_q[CLK_DIV] <= INIT_CLK_DIV;
    uart_regs_q[CTRL]    <= 'b0;
    uart_regs_q[TX_DATA] <= 'b0;
    uart_regs_q[RX_DATA] <= 'b0;
    uart_regs_q[STATUS]  <= 'b0;
    // DBG
    CS        <= IDLE;
    cmd_q     <= 'b0;
    addr_q    <= 'b0;
    data_w_q  <= 'b0;
  end else begin
    // UART
    uart_regs_q[CLK_DIV] <= uart_regs_n[CLK_DIV];
    uart_regs_q[CTRL]    <= uart_regs_n[CTRL];
    uart_regs_q[TX_DATA] <= uart_regs_n[TX_DATA];
    uart_regs_q[RX_DATA] <= uart_regs_n[RX_DATA];
    uart_regs_q[STATUS]  <= uart_regs_n[STATUS];

    // FIFO
    rx_fifo_write_q <= rx_fifo_write_n;
    tx_fifo_write_q <= tx_fifo_write_n;

    // DBG CTRL
    CS <= NS;
    dbg_data_q <= dbg_data_n;
    cmd_q <= cmd_n;
    addr_q <= addr_n;
    data_w_q <= data_w_n;

    if(rst_cnt)
      data_cnt <= 'b0;
    else if(incr_cnt)
      data_cnt <= data_cnt + 1;
  end
end

/* verilator lint_off PINMISSING */
/* verilator lint_off PINCONNECTEMPTY */
fifo #(
    .WIDTH      ( 8         ),
    .DEPTH      ( 32        ),
    .ALMOST_EN  ( 0         )
) rx_fifo (
    .clk_i      ( clk               ),
    .rstn_i     ( rstn_i            ),
    .din_i      ( rx_data           ),
    .we_i       ( rx_fifo_write_q   ),
    .re_i       ( rx_fifo_read      ),
    .dout_o     ( rx_data_read      ),
    .E_o        ( rx_empty          ),
    .F_o        ( rx_full           ),
    .AE_o       (),
    .AF_o       ()
);

fifo #(
    .WIDTH      ( 8         ),
    .DEPTH      ( 32        ),
    .ALMOST_EN  ( 0         )
) tx_fifo (
    .clk_i      ( clk                     ),
    .rstn_i     ( rstn_i                  ),
    .din_i      ( uart_regs_q[TX_DATA][7:0]),
    .we_i       ( tx_fifo_write_q         ),
    .re_i       ( tx_fifo_read            ),
    .dout_o     ( tx_data                 ),
    .E_o        ( tx_empty                ),
    .F_o        ( tx_full                 ),
    .AE_o       (),
    .AF_o       ()
);
/* verilator lint_on PINCONNECTEMPTY */
/* verilator lint_on PINMISSING */

uart_tx tx_mod_i (
    .clk        ( clk                       ),
    .rstn_i     ( rstn_i                    ),
    .clk_div_i  ( uart_regs_q[CLK_DIV]      ),
    .tx_data_i  ( tx_data                   ),
    .tx_valid_i ( tx_enable                 ),
    .tx_done_o  ( tx_done                   ),
    .parity_en_i( 1'b0                      ),
    .tx_o       ( tx_o                      )
);

uart_rx rx_mod_i (
    .clk        ( clk                       ),
    .rstn_i     ( rstn_i                    ),
    .clk_div_i  ( uart_regs_q[CLK_DIV]      ),
    .rx_enable_i( 1'b1                      ),
    .rx_data_o  ( rx_data                   ),
    .rx_valid_o ( rx_valid                  ),
    .rx_err_o   ( rx_parity_err             ),
    .parity_en_i( 1'b0                      ),
    .rx_i       ( rx_i                      )
);

dbg_module dbg_module_i (
  .clk              ( clk               ),
  .rstn_i           ( rstn_i            ),
  .cmd_i            ( dbg_cmd           ),
  .addr_i           ( addr_q            ),
  .data_i           ( data_w_q          ),
  .data_o           ( data_r            ),
  .ready_o          ( dbg_ready         ),
  .core_rst_req_o   ( core_rst_req_o    ),
  .periph_rst_req_o ( periph_rst_req_o  ),
  .dbg_bus          ( dbg_bus           ),
  .wb_bus           ( wb_bus            )
);

endmodule