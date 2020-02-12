// See LICENSE.Berkeley for license details.

/**********************************************************************
 * CS152 Lab 2: Open-Ended Problem 4.2                                *
 **********************************************************************/

#include <cstdio>

/*
 * Called at the beginning of simulation
 *
 * (1 << line_shift) yields the cache line size, which can be safely
 * assumed to be 64 bytes in this context.
 */
extern "C" void prefetcher_init(unsigned int line_shift)
{
  fputs("Using L1 prefetcher model\n", stderr);
}

/*
 * Encoding of `cpu_req_bits_cmd` field
 * Refer to MemoryOpConstants definition for all values
 */
enum cmd
{
  M_XRD     = 0x0, // load
  M_XWR     = 0x1, // store
  M_XA_SWAP = 0x4, // AMO swap
  M_XLR     = 0x6, // load-reserved
  M_XSC     = 0x7, // store-conditional
  M_XA_ADD  = 0x8, // AMO add
  M_XA_XOR  = 0x9, // AMO xor
  M_XA_OR   = 0xA, // AMO or
  M_XA_AND  = 0xB, // AMO and
  M_XA_MIN  = 0xC, // AMO min
  M_XA_MAX  = 0xD, // AMO max
  M_XA_MINU = 0xE, // AMO minu
  M_XA_MAXU = 0xF, // AMO maxu
};

/*
 * Called for each target clock cycle
 *
 * TODO: Implement your custom prefetcher
 */
extern "C" void prefetcher_tick(
    unsigned char cpu_req_valid,
    unsigned long long cpu_req_bits_addr,
    unsigned int cpu_req_bits_tag,
    unsigned int cpu_req_bits_cmd,
    unsigned int cpu_req_bits_size,
    unsigned char cpu_req_bits_signed,
    unsigned char cpu_miss, // Delayed by 2 clock cycles after `cpu_req_*`

    unsigned char dmem_ready,
    unsigned char *dmem_valid,
    unsigned long long *dmem_bits_addr, // Must be aligned to XLEN
    unsigned char *dmem_bits_write)
{
    // FIXME
    *dmem_req_valid = false;
    *dmem_req_bits_addr = 0LL;
    *dmem_req_bits_write = false;
}
