// See LICENSE.Berkeley for license details.

/**********************************************************************
 * CS152 Lab 2: Open-Ended Problem 4.2                                *
 **********************************************************************/

import "DPI-C" function void prefetcher_init
(
  input int unsigned line_shift
);

import "DPI-C" function void prefetcher_tick
(
  input  bit              cpu_req_valid,
  input  longint unsigned cpu_req_bits_addr,
  input  int unsigned     cpu_req_bits_tag,
  input  int unsigned     cpu_req_bits_cmd,
  input  int unsigned     cpu_req_bits_size,
  input  bit              cpu_req_bits_signed,
  input  bit              cpu_miss,

  input  bit              dmem_req_ready,
  output bit              dmem_req_valid,
  output longint unsigned dmem_req_bits_addr,
  output bit              dmem_req_bits_write,
  input  bit              dmem_nack
);

module ModelL1PrefetcherHarness #(
  parameter ADDR_BITS = 40,
            TAG_BITS = 8,
            CMD_BITS = 5,
            SIZE_BITS = 2,
            LINE_SHIFT = 6)(
  input                  clock,
  input                  reset,

  input                  cpu_req_valid,
  input  [ADDR_BITS-1:0] cpu_req_bits_addr,
  input  [TAG_BITS-1:0]  cpu_req_bits_tag,
  input  [CMD_BITS-1:0]  cpu_req_bits_cmd,
  input  [SIZE_BITS-1:0] cpu_req_bits_size,
  input                  cpu_req_bits_signed,
  input  [1:0]           cpu_req_bits_dprv,
  input                  cpu_miss, // Core request from 2 cycles ago has missed

  input                  dmem_req_ready,
  output                 dmem_req_valid,
  output [ADDR_BITS-1:0] dmem_req_bits_addr,
  output                 dmem_req_bits_write,
  input                  dmem_nack // Prefetch request from 2 cycles ago is rejected
);

  initial begin
    prefetcher_init(LINE_SHIFT);
  end

  bit              _dmem_req_valid;
  longint unsigned _dmem_req_bits_addr;
  bit              _dmem_req_bits_write;

  reg                 _dmem_req_valid_reg;
  reg [ADDR_BITS-1:0] _dmem_req_bits_addr_reg;
  reg                 _dmem_req_bits_write_reg;

  always @(posedge clock) begin
    if (reset) begin
      _dmem_req_valid_reg <= 1'b0;
    end else begin
      prefetcher_tick(
        cpu_req_valid,
        cpu_req_bits_addr,
        cpu_req_bits_tag,
        cpu_req_bits_cmd,
        cpu_req_bits_size,
        cpu_req_bits_signed,
        cpu_miss,
        dmem_req_ready,
        _dmem_req_valid,
        _dmem_req_bits_addr,
        _dmem_req_bits_write,
        dmem_nack);

      /* verilator lint_off WIDTH */
      _dmem_req_valid_reg <= _dmem_req_valid;
      _dmem_req_bits_addr_reg <= _dmem_req_bits_addr;
      _dmem_req_bits_write_reg <= _dmem_req_bits_write;
      /* verilator lint_on WIDTH */
    end
  end

  assign dmem_req_valid = _dmem_req_valid_reg;
  assign dmem_req_bits_addr = _dmem_req_bits_addr_reg;
  assign dmem_req_bits_write = _dmem_req_bits_write_reg;

endmodule
