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
 * Called for each clock cycle
 *
 * TODO: Implement your custom prefetcher
 *
 * NOTE: Treat assigning to `*dmem_valid`, `*dmem_addr`, and
 * `*dmem_write` as if the values are being latched by registers, and
 * avoid combinationally coupling them with `dmem_ready`.
 * If, for example, `dmem_valid` is asserted only when `dmem_ready` is
 * true, the outputs will be improperly delayed by one cycle relative to
 * `dmem_ready`, which will cause the prefetch request to be ignored by
 * the cache.
 */
extern "C" void prefetcher_tick(
    unsigned char cpu_req_valid,
    unsigned long long cpu_req_bits_addr,
    unsigned int cpu_req_bits_tag,
    unsigned int cpu_req_bits_cmd,
    unsigned int cpu_req_bits_size,
    unsigned char cpu_req_bits_signed,
    unsigned char cpu_miss, // Core request from 2 cycles ago has missed

    unsigned char dmem_req_ready,
    unsigned char *dmem_req_valid,
    unsigned long long *dmem_req_bits_addr, // Must be aligned to XLEN
    unsigned char *dmem_req_bits_write,
    unsigned char dmem_nack) // Prefetch request from 2 cycles ago is rejected
{
    // FIXME
    *dmem_req_valid = false;
    *dmem_req_bits_addr = 0LL;
    *dmem_req_bits_write = false;
}
