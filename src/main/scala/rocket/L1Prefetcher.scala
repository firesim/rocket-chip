// See LICENSE.Berkeley for license details.

/*********************************************************************
 * CS152 Lab 2: Open-Ended Problem 4.2                               *
 *********************************************************************/

package freechips.rocketchip.rocket

import chisel3._
import chisel3.util._
import chisel3.experimental.IntParam

import freechips.rocketchip.config._
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem.CacheBlockBytes

case object BuildL1Prefetcher extends Field[Option[Parameters => L1Prefetcher]](None)

trait CanHaveL1Prefetcher extends HasHellaCache { this: BaseTile =>
  val module: CanHaveL1PrefetcherModule
  implicit val p: Parameters
  if (p(BuildL1Prefetcher).isDefined) {
    nDCachePorts += 1
  }
}

trait CanHaveL1PrefetcherModule extends HasHellaCacheModule { this: RocketTileModuleImp =>
  val outer: CanHaveL1Prefetcher with HasTileParameters
  implicit val p: Parameters

  val prefetchOpt = p(BuildL1Prefetcher).map(_(p))
  val prefetchPort = prefetchOpt.map { pfu =>
    val dmem = Wire(new HellaCacheIO()(p))
    pfu.io.dmem.nack := dmem.s2_nack
    pfu.io.dmem.req.ready := dmem.req.ready
    dmem.req.valid := pfu.io.dmem.req.valid
    dmem.req.bits.addr := pfu.io.dmem.req.bits.addr
    dmem.req.bits.idx.foreach(_ := dmem.req.bits.addr)
    dmem.req.bits.tag := DontCare
    dmem.req.bits.cmd := Mux(pfu.io.dmem.req.bits.write, M_PFW, M_PFR)
    dmem.req.bits.size := log2Ceil(xLen/8).U
    dmem.req.bits.signed := false.B
    dmem.req.bits.phys := false.B
    dmem.req.bits.no_alloc := false.B
    dmem.req.bits.no_xcpt := false.B
    dmem.req.bits.data := DontCare
    dmem.req.bits.mask := 0.U
    dmem.s1_data.data := DontCare
    dmem.s1_data.mask := 0.U
    // Tie off unused control signals
    dmem.s1_kill := false.B
    dmem.s2_kill := false.B
    dmem.keep_clock_enabled := pfu.io.dmem.req.valid
    dmem
  }
}

// See MemoryOpConstants for encoding of `cmd` field
class CoreMemReq(implicit p: Parameters) extends CoreBundle()(p) with HasCoreMemOp

class L1PrefetchReq(implicit p: Parameters) extends CoreBundle()(p) {
  val addr = UInt(coreMaxAddrBits.W) // Must be aligned to XLEN
  val write = Bool()
}

class L1PrefetcherIO(implicit p: Parameters) extends CoreBundle()(p) {
  val cpu = new Bundle {
    val req = Flipped(Valid(new CoreMemReq))
    val miss = Input(Bool()) // Core request from 2 cycles ago has missed
  }
  val dmem = new Bundle {
    val req = Decoupled(new L1PrefetchReq)
    val nack = Input(Bool()) // Prefetch request from 2 cycles ago is rejected
  }
}

abstract class L1Prefetcher(implicit p: Parameters) extends CoreModule()(p) {
  val io = IO(new L1PrefetcherIO)
}

/**
 * Naive implementation of the one-block prefetch-on-miss scheme
 */
class ExampleL1Prefetcher(implicit p: Parameters) extends L1Prefetcher {

  val s1_valid = RegNext(io.cpu.req.valid, false.B)
  val s1_req = RegEnable(io.cpu.req.bits, io.cpu.req.valid)
  val s1_addr = s1_req.addr(coreMaxAddrBits-1, lgCacheBlockBytes)
  val s1_addr_next = s1_addr + 1.U // Compute address of next block

  val s2_req = RegNext(s1_req)
  s2_req.addr := Cat(s1_addr_next, 0.U(lgCacheBlockBytes.W))

  // Keep prefetch request active after a miss is signaled if L1D is not
  // immediately ready to accept it
  val miss_hold = RegInit(false.B)
  when (io.cpu.miss) {
    miss_hold := true.B
  }
  // Clear flag after a prefetch request goes through or if another
  // memory request is arriving the next cycle
  when (s1_valid || io.dmem.req.ready) {
    miss_hold := false.B
  }

  io.dmem.req.valid := io.cpu.miss || miss_hold
  io.dmem.req.bits.addr := s2_req.addr
  io.dmem.req.bits.write := isWrite(s2_req.cmd)

  dontTouch(io.dmem.nack)
}


/**
 * TODO: Implement your custom prefetcher
 */
class CustomL1Prefetcher(implicit p: Parameters) extends L1Prefetcher {
  // See L1PrefetcherIO bundle for IO port definitions
  io.dmem.req.valid := false.B // FIXME
}


/**
 * Wrapper around C++ software model
 */
class ModelL1Prefetcher(implicit p: Parameters) extends L1Prefetcher {
  val model = Module(new ModelL1PrefetcherHarness(
    addrBits = io.cpu.req.bits.addr.getWidth,
    tagBits = io.cpu.req.bits.tag.getWidth,
    cmdBits = io.cpu.req.bits.cmd.getWidth,
    sizeBits = io.cpu.req.bits.size.getWidth))

  model.io.clock := clock
  model.io.reset := reset
  model.io.cpu := io.cpu
  io.dmem <> model.io.dmem
}

/**
 * Blackbox for Verilog DPI harness
 */
class ModelL1PrefetcherHarness(addrBits: Int, tagBits: Int, cmdBits: Int, sizeBits: Int)(implicit p: Parameters)
    extends BlackBox(Map(
      "ADDR_BITS" -> IntParam(addrBits),
      "TAG_BITS" -> IntParam(tagBits),
      "CMD_BITS" -> IntParam(cmdBits),
      "SIZE_BITS" -> IntParam(sizeBits),
      "LINE_SHIFT" -> IntParam(log2Up(p(CacheBlockBytes)))))
    with HasBlackBoxResource {

  val io = IO(new L1PrefetcherIO {
    val clock = Input(Clock())
    val reset = Input(Bool())
  })

  addResource("/vsrc/L1Prefetcher.v")
  addResource("/csrc/L1Prefetcher.cc")
}
