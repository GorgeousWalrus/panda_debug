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
  parameter int BAUDRATE = 1000000,
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
localparam DBG_ACK = 32'h00000001;
localparam DBG_END = 32'h00000002;

// DEBUG signals
logic           dbg_exec;
logic           dbg_ready;
logic [7:0]     cmd_n, cmd_q, dbg_cmd;
logic [31:0]    addr_n, addr_q;
logic [31:0]    data_w_n, data_w_q;
logic [31:0]    data_r;

enum logic [2:0] {  IDLE, 
              CMD_DECODE,
              ADDR_READ,
              DATA_READ,
              CMD_EXEC,
              DATA_WRITE,
              CMD_FINISH
            } CS, NS;

assign dbg_cmd = (dbg_exec) ? cmd_q : 8'b0;

// UART signals
// clk div for baudrate
localparam logic [31:0] INIT_CLK_DIV = CLK_FREQ/BAUDRATE;

// registers
logic [31:0] uart_regs_n[3:0];
logic [31:0] uart_regs_q[3:0];

// TX signals
logic [1:0]     tx_cnt;
logic [7:0]     tx_data;
logic           tx_incr_cnt;
logic           tx_rst_cnt;
logic           tx_done;
logic           tx_enable;

// RX signals
logic [1:0]     rx_cnt;
logic [7:0]     rx_data;
logic           rx_incr_cnt;
logic           rx_rst_cnt;
logic           rx_valid;
logic           rx_parity_err;

always_comb
begin
    // Debug
    dbg_exec = 1'b0;
    NS       = CS;
    cmd_n    = cmd_q;
    addr_n   = addr_q;
    data_w_n = data_w_q;

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
    uart_regs_n[RX_DATA] = rx_data_read;

    // UART TX
    if(uart_regs_q[CTRL][TX_EN] && (!uart_regs_q[STATUS][TX_EMPTY])) begin
        tx_enable = 1'b1;
        if(tx_done) begin
            tx_enable = 1'b0;
            // notify fifo that we processed the data
            tx_fifo_read = 1'b1;
            // If this was the last word in txfifo, disable uart
            if(uart_regs_q[STATUS][TX_EMPTY])
                uart_regs_n[CTRL][TX_EN] = 1'b0;
        end
    end

    // UART RX
    if((!uart_regs_q[STATUS][RX_FULL]) && uart_regs_q[CTRL][RX_EN]) begin
        if(rx_valid) begin
            rx_fifo_write_n = 1'b1;
        end
    end

    // Debug CTRL
    case(CS)
      IDLE: begin
        if(uart_regs_q[CTRL][RX_FULL]) begin
          // Read the command
          cmd_n = uart_regs_q[RX_DATA][7:0];
          uart_regs_n[RX_DATA] = 'b0;
          uart_regs_n[CTRL][RX_FULL] = 1'b0;
          // Acknowledge cmd
          uart_regs_n[CTRL][TX_EN] = 1'b1;
          uart_regs_n[CTRL][TX_FULL] = 1'b1;
          uart_regs_n[TX_DATA] = DBG_ACK;
          // Go to next state
          NS = CMD_DECODE;
        end
      end

      CMD_DECODE: begin
        // If a memory operation is performed, read address, else execute command
        if(cmd_q[7])
          NS = ADDR_READ;
        else begin
          NS = CMD_EXEC;
          dbg_exec = 1'b1;
        end
      end

      ADDR_READ: begin
        if(uart_regs_q[CTRL][RX_FULL]) begin
          // Read address
          addr_n = uart_regs_q[RX_DATA];
          uart_regs_n[CTRL][RX_FULL] = 1'b0;
          uart_regs_n[RX_DATA] = 'b0;
          // Acknowledge address
          if(!uart_regs_q[CTRL][TX_FULL]) begin
            uart_regs_n[CTRL][TX_EN] = 1'b1;
            uart_regs_n[CTRL][TX_FULL] = 1'b1;
            uart_regs_n[TX_DATA] = DBG_ACK;
            // Go to next state
            // If a write operation is performed, read the data from uart, else execute
            if(cmd_q[6])
              NS = DATA_READ;
            else begin
              NS = CMD_EXEC;
              dbg_exec = 1'b1;
            end
          end
        end
      end

      DATA_READ: begin
        if(uart_regs_q[CTRL][RX_FULL]) begin
          // Read data
          data_w_n = uart_regs_q[RX_DATA];
          uart_regs_n[RX_DATA] = 'b0;
          uart_regs_n[CTRL][RX_FULL] = 1'b0;
          // Acknowledge data
          if(!uart_regs_q[CTRL][TX_FULL]) begin
            uart_regs_n[CTRL][TX_EN] = 1'b1;
            uart_regs_n[CTRL][TX_FULL] = 1'b1;
            uart_regs_n[TX_DATA] = DBG_ACK;
            // Go to next state
            NS = CMD_EXEC;
            dbg_exec = 1'b1;
          end
        end
      end

      CMD_EXEC: begin
        dbg_exec = 1'b1;
        if(dbg_ready) begin
          dbg_exec = 1'b0;
          if(!uart_regs_q[CTRL][TX_FULL]) begin
            if(cmd_q[7] && !cmd_q[6]) begin
              // write data
              uart_regs_n[CTRL][TX_EN] = 1'b1;
              uart_regs_n[CTRL][TX_FULL] = 1'b1;
              uart_regs_n[TX_DATA] = data_r;
              // Go to write state
              NS = DATA_WRITE;
            end else begin
              // write data
              uart_regs_n[CTRL][TX_EN] = 1'b1;
              uart_regs_n[CTRL][TX_FULL] = 1'b1;
              uart_regs_n[TX_DATA] = DBG_END;
              NS = CMD_FINISH;
            end
          end
        end
      end

      DATA_WRITE: begin
        // wait for ack
        if(uart_regs_q[CTRL][RX_FULL]) begin
          uart_regs_n[CTRL][RX_FULL] = 1'b0;
          uart_regs_n[RX_DATA] = 'b0;
          // write data
          if(!uart_regs_q[CTRL][TX_FULL]) begin
            uart_regs_n[CTRL][TX_EN] = 1'b1;
            uart_regs_n[CTRL][TX_FULL] = 1'b1;
            uart_regs_n[TX_DATA] = DBG_END;
            NS = CMD_FINISH;
          end
        end
      end

      CMD_FINISH: begin
        // wait for ack
        if(uart_regs_q[CTRL][RX_FULL]) begin
          uart_regs_n[CTRL][RX_FULL] = 1'b0;
          uart_regs_n[RX_DATA] = 'b0;
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

    // DBG CTRL
    CS <= NS;
      
    cmd_q <= cmd_n;
    addr_q <= addr_n;
    data_w_q <= data_w_n;
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
    .rstn_i     ( rstn              ),
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
    .rstn_i     ( rstn                    ),
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