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
  parameter int BAUDRATE = 50000000,
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

// control bits
localparam TX_EN = 0;
localparam TX_FULL = 1;
localparam RX_FULL = 2;
localparam RX_ERR = 3;

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
localparam logic [31:0] INIT_CLK_DIV = 0;

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

    // UART
    tx_enable   = 1'b0;
    tx_incr_cnt = 1'b0;
    tx_rst_cnt  = 1'b0;
    tx_data     = 'b0;

    rx_incr_cnt = 1'b0;
    rx_rst_cnt  = 1'b0;

    uart_regs_n = uart_regs_q;

    // UART TX
    if(uart_regs_q[CTRL][TX_EN]) begin
        tx_enable = 1'b1;
        /* verilator lint_off WIDTH */
        tx_data = (uart_regs_q[TX_DATA] >> (8*tx_cnt));
        /* verilator lint_on WIDTH */
        if(tx_done) begin
            tx_enable = 1'b0;
            tx_incr_cnt = 1'b1;
            if(tx_cnt == 2'b11) begin
                tx_rst_cnt = 1'b1;
                uart_regs_n[CTRL][TX_EN]   = 1'b0;
                uart_regs_n[CTRL][TX_FULL] = 1'b0;
            end
        end
    end

    // UART RX
    if(!uart_regs_q[CTRL][RX_FULL]) begin
        if(rx_valid) begin
            uart_regs_n[RX_DATA] = uart_regs_q[RX_DATA] | ({{24{1'b0}}, rx_data} << (8*rx_cnt));
            rx_incr_cnt = 1'b1;
            if(rx_cnt == 2'b11) begin
                rx_rst_cnt = 1'b1;
                uart_regs_n[CTRL][RX_FULL] = 1'b1;
                uart_regs_n[CTRL][RX_ERR]  = rx_parity_err;
            end
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

    // increment and reset TX counter
    if(tx_rst_cnt)
        tx_cnt <= 'b0;
    else if(tx_incr_cnt)
        tx_cnt <= tx_cnt + 1;

    // increment and reset RX counter
    if(rx_rst_cnt)
        rx_cnt <= 'b0;
    else if(rx_incr_cnt)
        rx_cnt <= rx_cnt + 1;

    // DBG CTRL
    CS <= NS;
      
    cmd_q <= cmd_n;
    addr_q <= addr_n;
    data_w_q <= data_w_n;
  end
end

uart_tx tx_mod_i (
    .clk        ( clk                       ),
    .rstn_i     ( rstn_i                    ),
    .clk_div_i  ( uart_regs_q[CLK_DIV]      ),
    .tx_data_i  ( tx_data                   ),
    .tx_valid_i ( tx_enable                 ),
    .tx_done_o  ( tx_done                   ),
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