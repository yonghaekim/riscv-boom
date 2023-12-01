//******************************************************************************
// Copyright (c) 2012 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Out-of-Order Load/Store Unit
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Load/Store Unit is made up of the Load-Address Queue, the Store-Address
// Queue, and the Store-Data queue (LAQ, SAQ, and SDQ).
//
// Stores are sent to memory at (well, after) commit, loads are executed
// optimstically ASAP.  If a misspeculation was discovered, the pipeline is
// cleared. Loads put to sleep are retried.  If a LoadAddr and StoreAddr match,
// the Load can receive its data by forwarding data out of the Store-Data
// Queue.
//
// Currently, loads are sent to memory immediately, and in parallel do an
// associative search of the SAQ, on entering the LSU. If a hit on the SAQ
// search, the memory request is killed on the next cycle, and if the SDQ entry
// is valid, the store data is forwarded to the load (delayed to match the
// load-use delay to delay with the write-port structural hazard). If the store
// data is not present, or it's only a partial match (SB->LH), the load is put
// to sleep in the LAQ.
//
// Memory ordering violations are detected by stores at their addr-gen time by
// associatively searching the LAQ for newer loads that have been issued to
// memory.
//
// The store queue contains both speculated and committed stores.
//
// Only one port to memory... loads and stores have to fight for it, West Side
// Story style.
//
// TODO:
//    - Add predicting structure for ordering failures
//    - currently won't STD forward if DMEM is busy
//    - ability to turn off things if VM is disabled
//    - reconsider port count of the wakeup, retry stuff

package boom.lsu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.Str

import boom.common._
import boom.exu.{BrUpdateInfo, Exception, FuncUnitResp, CommitSignals, ExeUnitResp}
import boom.exu.FuncUnitCapResp //yh+
import boom.util.{BoolToChar, AgePriorityEncoder, IsKilledByBranch, GetNewBrMask, WrapInc, IsOlder, UpdateBrMask}

class LSUExeIO(implicit p: Parameters) extends BoomBundle()(p)
{
  // The "resp" of the maddrcalc is really a "req" to the LSU
  val req       = Flipped(new ValidIO(new FuncUnitResp(xLen)))
  // Send load data to regfiles
  val iresp    = new DecoupledIO(new boom.exu.ExeUnitResp(xLen))
  val fresp    = new DecoupledIO(new boom.exu.ExeUnitResp(xLen+1)) // TODO: Should this be fLen?
}

//yh+begin
class LSUCapExeIO(implicit p: Parameters) extends BoomBundle()(p)
{
  // The "resp" of the caddrcalc is really a "cap_req" to the LSU
  val cap_req		= Flipped(new ValidIO(new FuncUnitCapResp(xLen)))
}
//yh+end

class BoomDCacheReq(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomUOP
{
  val addr  = UInt(coreMaxAddrBits.W)
  val data  = Bits(coreDataBits.W)
  val is_hella = Bool() // Is this the hellacache req? If so this is not tracked in LDQ or STQ
}

class BoomDCacheResp(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomUOP
	with rocket.HasL1HellaCacheParameters //yh+
{
  val data = Bits(coreDataBits.W)
  val dataBeats = Bits(encRowBits.W) //yh+
  val is_hella = Bool()
}

//yh+begin
class BoomCapltyResp(implicit p: Parameters) extends BoomBundle()(p)
  with HasBoomUOP
	with rocket.HasL1HellaCacheParameters
{
  val data = Bits(encRowBits.W)
}
//yh+end

class LSUDMemIO(implicit p: Parameters, edge: TLEdgeOut) extends BoomBundle()(p)
{
  // In LSU's dmem stage, send the request
  val req         = new DecoupledIO(Vec(memWidth, Valid(new BoomDCacheReq)))
  // In LSU's LCAM search stage, kill if order fail (or forwarding possible)
  val s1_kill     = Output(Vec(memWidth, Bool()))
  // Get a request any cycle
  val resp        = Flipped(Vec(memWidth, new ValidIO(new BoomDCacheResp)))
  // In our response stage, if we get a nack, we need to reexecute
  val nack        = Flipped(Vec(memWidth, new ValidIO(new BoomDCacheReq)))
	//yh+begin
  val cap_resp = Flipped(Vec(memWidth, new ValidIO(new BoomDCacheResp)))
  val cap_nack = Flipped(Vec(memWidth, new ValidIO(new BoomDCacheReq)))
	//yh+end

  val brupdate       = Output(new BrUpdateInfo)
  val exception    = Output(Bool())
  val rob_pnr_idx  = Output(UInt(robAddrSz.W))
  val rob_head_idx = Output(UInt(robAddrSz.W))

  val release = Flipped(new DecoupledIO(new TLBundleC(edge.bundle)))

  // Clears prefetching MSHRs
  val force_order  = Output(Bool())
  val ordered     = Input(Bool())

  val perf = Input(new Bundle {
    val acquire = Bool()
    val release = Bool()
  })

}

class LSUCoreIO(implicit p: Parameters) extends BoomBundle()(p)
{
  val exe = Vec(memWidth, new LSUExeIO)
  val cap_exe = Vec(memWidth, new LSUCapExeIO) //yh+

  val dis_uops    = Flipped(Vec(coreWidth, Valid(new MicroOp)))
  val dis_ldq_idx = Output(Vec(coreWidth, UInt(ldqAddrSz.W)))
  val dis_stq_idx = Output(Vec(coreWidth, UInt(stqAddrSz.W)))

  val ldq_full    = Output(Vec(coreWidth, Bool()))
  val stq_full    = Output(Vec(coreWidth, Bool()))

  val fp_stdata   = Flipped(Decoupled(new ExeUnitResp(fLen)))

  val commit      = Input(new CommitSignals)
  val commit_load_at_rob_head = Input(Bool())

  // Stores clear busy bit when stdata is received
  // memWidth for int, 1 for fp (to avoid back-pressure fpstdat)
  val clr_bsy         = Output(Vec(memWidth + 1, Valid(UInt(robAddrSz.W))))

	//yh+begin
  // Stores clear needCC bit when capability check is done
  val clr_needCC      = Output(Vec(memWidth + memWidth, Valid(UInt(robAddrSz.W))))
	//yh+end

  // Speculatively safe load (barring memory ordering failure)
  val clr_unsafe      = Output(Vec(memWidth, Valid(UInt(robAddrSz.W))))

  // Tell the DCache to clear prefetches/speculating misses
  val fence_dmem   = Input(Bool())

  // Speculatively tell the IQs that we'll get load data back next cycle
  val spec_ld_wakeup = Output(Vec(memWidth, Valid(UInt(maxPregSz.W))))
  // Tell the IQs that the load we speculated last cycle was misspeculated
  val ld_miss      = Output(Bool())

  val brupdate       = Input(new BrUpdateInfo)
  val rob_pnr_idx  = Input(UInt(robAddrSz.W))
  val rob_head_idx = Input(UInt(robAddrSz.W))
  val exception    = Input(Bool())

  val fencei_rdy  = Output(Bool())
	//yh+begin
	val slq_nonempty = Output(Bool())
	val ssq_nonempty = Output(Bool())
	val scq_nonempty = Output(Bool())
	//yh+end

  val lxcpt       = Output(Valid(new Exception))

  val tsc_reg     = Input(UInt())

  val perf        = Output(new Bundle {
    val acquire = Bool()
    val release = Bool()
    val tlbMiss = Bool()
  })

  //yh+begin
  val dis_slq_idx = Output(Vec(coreWidth, UInt(slqAddrSz.W)))
  val dis_ssq_idx = Output(Vec(coreWidth, UInt(ssqAddrSz.W)))
  val dis_scq_idx = Output(Vec(coreWidth, UInt(scqAddrSz.W)))

  val slq_full    = Output(Vec(coreWidth, Bool()))
  val ssq_full    = Output(Vec(coreWidth, Bool()))
  val scq_full    = Output(Vec(coreWidth, Bool()))

  val dpt_csrs            = Input(new LSUDptCSRs)
  val num_store    				= Output(UInt(xLen.W))
  val num_load    				= Output(UInt(xLen.W))
  val num_tagged_store    = Output(UInt(xLen.W))
  val num_tagged_load     = Output(UInt(xLen.W))
	val ldst_traffic			  = Output(UInt(xLen.W))
	val bounds_traffic		  = Output(UInt(xLen.W))
  val num_store_hit       = Output(UInt(xLen.W))
  val num_load_hit        = Output(UInt(xLen.W))
  val num_cstr            = Output(UInt(xLen.W))
  val num_cclr            = Output(UInt(xLen.W))
  val num_slq_itr         = Output(UInt(xLen.W))
  val num_ssq_itr         = Output(UInt(xLen.W))
  val num_scq_itr         = Output(UInt(xLen.W))
  //yh+end
}

class LSUIO(implicit p: Parameters, edge: TLEdgeOut) extends BoomBundle()(p)
{
  val ptw   = new rocket.TLBPTWIO
  val core  = new LSUCoreIO
  val dmem  = new LSUDMemIO

  val hellacache = Flipped(new freechips.rocketchip.rocket.HellaCacheIO)
}

class LDQEntry(implicit p: Parameters) extends BoomBundle()(p)
    with HasBoomUOP
{
  val addr                = Valid(UInt(coreMaxAddrBits.W))
  val addr_is_virtual     = Bool() // Virtual address, we got a TLB miss
  val addr_is_uncacheable = Bool() // Uncacheable, wait until head of ROB to execute

  val executed            = Bool() // load sent to memory, reset by NACKs
  val succeeded           = Bool()
  val order_fail          = Bool()
  val observed            = Bool()

  val st_dep_mask         = UInt(numStqEntries.W) // list of stores older than us
  val youngest_stq_idx    = UInt(stqAddrSz.W) // index of the oldest store younger than us

  val forward_std_val     = Bool()
  val forward_stq_idx     = UInt(stqAddrSz.W) // Which store did we get the store-load forward from?

  val debug_wb_data       = UInt(xLen.W)
}

class STQEntry(implicit p: Parameters) extends BoomBundle()(p)
   with HasBoomUOP
{
  val addr                = Valid(UInt(coreMaxAddrBits.W))
  val addr_is_virtual     = Bool() // Virtual address, we got a TLB miss
  val data                = Valid(UInt(xLen.W))

  val committed           = Bool() // committed by ROB
  val succeeded           = Bool() // D$ has ack'd this, we don't need to maintain this anymore

  val debug_wb_data       = UInt(xLen.W)
}

//yh+begin
class SLQEntry(implicit p: Parameters) extends BoomBundle()(p)
    with HasBoomUOP
{
	val tag									= UInt(tagWidth.W)
	val cmt_addr						= UInt(vaddrBits.W)
	val num_ways						= UInt(wayAddrSz.W)
	val addr								= UInt((vaddrBits+1).W)
	val way									= UInt(wayAddrSz.W)
	val count								= UInt(wayAddrSz.W)
	val state								= UInt(3.W)
	val tagged						  = Bool()
  val tag_dep_mask        = UInt(numScqEntries.W)
	//val dir									= Bool() // 1: forward, 0: backward
  val ccache_hit          = Bool()
  val is_rob_head         = Bool()
}

class SSQEntry(implicit p: Parameters) extends BoomBundle()(p)
    with HasBoomUOP
{
	val tag									= UInt(tagWidth.W)
	val cmt_addr						= UInt(vaddrBits.W)
	val num_ways						= UInt(wayAddrSz.W)
	val addr								= UInt((vaddrBits+1).W)
	val way									= UInt(wayAddrSz.W)
	val count								= UInt(wayAddrSz.W)
	val state								= UInt(3.W)
	val tagged						  = Bool()
  val tag_dep_mask        = UInt(numScqEntries.W)
	//val dir									= Bool() // 1: forward, 0: backward
  val ccache_hit          = Bool()
  val is_rob_head         = Bool()
}

class SCQEntry(implicit p: Parameters) extends BoomBundle()(p)
   with HasBoomUOP
{
	val tag									= UInt(tagWidth.W)
	val cmt_addr						= UInt(vaddrBits.W)
	val num_ways						= UInt(wayAddrSz.W)
	val addr								= UInt((vaddrBits+1).W)
  val data                = Valid(UInt(vaddrBits.W))
	val way									= UInt(wayAddrSz.W)
	val count								= UInt(wayAddrSz.W)
	val state								= UInt(3.W)
	val tagged						  = Bool()
	//val dir									= Bool() // 1: forward, 0: backward
  val ccache_hit          = Bool()
  val is_rob_head         = Bool()
}

class LSUDptCSRs(implicit p: Parameters) extends BoomBundle()(p)
{
  val enableDPT           = Bool()
  val enableStat          = Bool()
  val disableSpec         = Bool()
  val suppressFault       = Bool()
  val ignoreDepMask				= Bool()
  val cmt_base            = UInt(coreMaxAddrBits.W)
  val num_store    				= UInt(xLen.W)
  val num_load    				= UInt(xLen.W)
  val num_tagged_store    = UInt(xLen.W)
  val num_tagged_load     = UInt(xLen.W)
  val ldst_traffic        = UInt(xLen.W)
  val bounds_traffic      = UInt(xLen.W)
  val num_store_hit       = UInt(xLen.W)
  val num_load_hit        = UInt(xLen.W)
  val num_cstr            = UInt(xLen.W)
  val num_cclr            = UInt(xLen.W)
	val num_slq_itr					= UInt(xLen.W)
	val num_ssq_itr					= UInt(xLen.W)
	val num_scq_itr					= UInt(xLen.W)
  val bounds_margin       = UInt(xLen.W)
}
//yh+end

class LSU(implicit p: Parameters, edge: TLEdgeOut) extends BoomModule()(p)
  with rocket.HasL1HellaCacheParameters
{
  val io = IO(new LSUIO)


  val ldq = Reg(Vec(numLdqEntries, Valid(new LDQEntry)))
  val stq = Reg(Vec(numStqEntries, Valid(new STQEntry)))

  val ldq_head         = Reg(UInt(ldqAddrSz.W))
  val ldq_tail         = Reg(UInt(ldqAddrSz.W))
  val stq_head         = Reg(UInt(stqAddrSz.W)) // point to next store to clear from STQ (i.e., send to memory)
  val stq_tail         = Reg(UInt(stqAddrSz.W))
  val stq_commit_head  = Reg(UInt(stqAddrSz.W)) // point to next store to commit
  val stq_execute_head = Reg(UInt(stqAddrSz.W)) // point to next store to execute

  //yh+begin
  val slq = Reg(Vec(numSlqEntries, Valid(new SLQEntry)))
  val ssq = Reg(Vec(numSsqEntries, Valid(new SSQEntry)))
  val scq = Reg(Vec(numScqEntries, Valid(new SCQEntry)))

  val slq_head         = Reg(UInt(slqAddrSz.W))
  val slq_tail         = Reg(UInt(slqAddrSz.W))
  val ssq_head         = Reg(UInt(ssqAddrSz.W))
  val ssq_tail         = Reg(UInt(ssqAddrSz.W))
  val scq_head         = Reg(UInt(scqAddrSz.W))
  val scq_tail         = Reg(UInt(scqAddrSz.W))

  val s_init :: s_update :: s_fire :: s_wait :: s_fire2 :: s_wait2 :: s_fail :: s_done :: Nil = Enum(8)

	//def WrapIncOrDecWay(dir: Bool, cur_way: UInt, num_ways: UInt): UInt = {
	//	Mux(dir, Mux(cur_way === (num_ways - 1.U), 0.U, cur_way + 1.U),
	//			Mux(cur_way === 0.U, num_ways - 1.U, cur_way - 1.U))
	//}

	def WrapIncWay(cur_way: UInt, num_ways: UInt): UInt = {
		Mux(cur_way === (num_ways - 1.U), 0.U, cur_way + 1.U)
	}

	//def WrapDecWay(cur_way: UInt, size: UInt, num_ways: UInt): UInt = {
	//	Mux(cur_way === 0.U, num_ways - size, cur_way - size)
	//}
  //yh+end

  // If we got a mispredict, the tail will be misaligned for 1 extra cycle
  assert (io.core.brupdate.b2.mispredict ||
          stq(stq_execute_head).valid ||
          stq_head === stq_execute_head ||
          stq_tail === stq_execute_head,
            "stq_execute_head got off track.")

  val h_ready :: h_s1 :: h_s2 :: h_s2_nack :: h_wait :: h_replay :: h_dead :: Nil = Enum(7)
  // s1 : do TLB, if success and not killed, fire request go to h_s2
  //      store s1_data to register
  //      if tlb miss, go to s2_nack
  //      if don't get TLB, go to s2_nack
  //      store tlb xcpt
  // s2 : If kill, go to dead
  //      If tlb xcpt, send tlb xcpt, go to dead
  // s2_nack : send nack, go to dead
  // wait : wait for response, if nack, go to replay
  // replay : refire request, use already translated address
  // dead : wait for response, ignore it
  val hella_state           = RegInit(h_ready)
  val hella_req             = Reg(new rocket.HellaCacheReq)
  val hella_data            = Reg(new rocket.HellaCacheWriteData)
  val hella_paddr           = Reg(UInt(paddrBits.W))
  val hella_xcpt            = Reg(new rocket.HellaCacheExceptions)


  val dtlb = Module(new NBDTLB(
    instruction = false, lgMaxSize = log2Ceil(coreDataBytes), rocket.TLBConfig(dcacheParams.nTLBSets, dcacheParams.nTLBWays)))

  io.ptw <> dtlb.io.ptw
  io.core.perf.tlbMiss := io.ptw.req.fire
  io.core.perf.acquire := io.dmem.perf.acquire
  io.core.perf.release := io.dmem.perf.release



  val clear_store     = WireInit(false.B)
  val live_store_mask = RegInit(0.U(numStqEntries.W))
  var next_live_store_mask = Mux(clear_store, live_store_mask & ~(1.U << stq_head),
                                              live_store_mask)

  //yh+begin
  val enableDPT        		= RegInit(false.B)
  val initDPT           	= Reg(Bool())
  val enableStat       		= RegInit(false.B)
  val disableSpec       	= Reg(Bool())
  val suppressFault       = Reg(Bool())
  val ignoreDepMask      	= Reg(Bool())
  val initStat          	= Reg(Bool())
  val cmt_base            = Reg(UInt(coreMaxAddrBits.W))

  val num_store    				= Reg(UInt(xLen.W))
  val num_load    				= Reg(UInt(xLen.W))
  val num_tagged_store    = Reg(UInt(xLen.W))
  val num_tagged_load     = Reg(UInt(xLen.W))
  val ldst_traffic        = Reg(UInt(xLen.W))
  val bounds_traffic      = Reg(UInt(xLen.W))
  val num_store_hit       = Reg(UInt(xLen.W))
  val num_load_hit        = Reg(UInt(xLen.W))
  val num_cstr            = Reg(UInt(xLen.W))
  val num_cclr            = Reg(UInt(xLen.W))
  val num_slq_itr         = Reg(UInt(xLen.W))
  val num_ssq_itr         = Reg(UInt(xLen.W))
  val num_scq_itr         = Reg(UInt(xLen.W))

  val bounds_margin       = Reg(UInt(vaddrBits.W))
  
  enableDPT              	:= io.core.dpt_csrs.enableDPT
  initDPT                	:= io.core.dpt_csrs.enableDPT & !enableDPT
  enableStat             	:= io.core.dpt_csrs.enableStat
  initStat               	:= io.core.dpt_csrs.enableStat & !enableStat
  disableSpec             := io.core.dpt_csrs.disableSpec
  suppressFault           := io.core.dpt_csrs.suppressFault
  ignoreDepMask           := io.core.dpt_csrs.ignoreDepMask
  cmt_base                := io.core.dpt_csrs.cmt_base(coreMaxAddrBits-1,0)
  bounds_margin          	:= ~(io.core.dpt_csrs.bounds_margin(vaddrBits-1,0))

  io.core.num_store  					:= num_store
  io.core.num_load  					:= num_load
  io.core.num_tagged_store  	:= num_tagged_store
  io.core.num_tagged_load   	:= num_tagged_load
  io.core.ldst_traffic      	:= ldst_traffic
  io.core.bounds_traffic    	:= bounds_traffic
  io.core.num_store_hit       := num_store_hit
  io.core.num_load_hit        := num_load_hit
  io.core.num_cstr            := num_cstr
  io.core.num_cclr            := num_cclr
  io.core.num_slq_itr         := num_slq_itr
  io.core.num_ssq_itr         := num_ssq_itr
  io.core.num_scq_itr         := num_scq_itr

  //val clear_capst = WireInit(false.B) // cstr, cclr
  val live_tag_mask = RegInit(0.U(numScqEntries.W))
  val zero_tag_mask = RegInit(0.U(numScqEntries.W))
	zero_tag_mask := 0.U
	var new_tag_mask = zero_tag_mask
  var next_live_tag_mask = live_tag_mask
  //var next_live_tag_mask = Mux(clear_capst, live_tag_mask & ~(1.U << scq_head),
  //                                          live_tag_mask)

  var temp_scq_head = scq_head
  var temp_num_cstr = num_cstr
  var temp_num_cclr = num_cclr
  var temp_num_scq_itr = num_scq_itr

	printf("[%d] num_slq_itr: %d num_ssq_itr: %d num_scq_itr: %d\n",
					io.core.tsc_reg, num_slq_itr, num_ssq_itr, num_scq_itr)

  for (w <- 0 until coreWidth)
  {
    val commit_capst = (io.core.commit.valids(w) && io.core.commit.uops(w).uses_stq &&
                        io.core.commit.uops(w).is_cap)

		when (commit_capst) {
      assert (scq(temp_scq_head).valid, "[lsu] trying to commit an un-allocated SCQ entry.")
      assert (scq(temp_scq_head).bits.state === s_done,
							"[lsu] trying to commit an un-executed SCQ entry.")

      scq(temp_scq_head).valid      := false.B
      scq(temp_scq_head).bits.state	:= s_init
      printf("[%d] Commit scq(%d) num_ways: %d count: %d\n",
							io.core.tsc_reg, temp_scq_head, scq(temp_scq_head).bits.num_ways, scq(temp_scq_head).bits.count)
		}

		temp_num_cstr = Mux(enableStat & commit_capst & scq(temp_scq_head).bits.uop.cap_cmd === CAP_CS,
												temp_num_cstr + 1.U, temp_num_cstr)
		temp_num_cclr = Mux(enableStat & commit_capst & scq(temp_scq_head).bits.uop.cap_cmd === CAP_CC,
												temp_num_cclr + 1.U, temp_num_cclr)
		temp_num_scq_itr = Mux(enableStat & commit_capst,
												temp_num_scq_itr + (scq(temp_scq_head).bits.num_ways - scq(temp_scq_head).bits.count),
												temp_num_scq_itr)

  	next_live_tag_mask = Mux(commit_capst, next_live_tag_mask & ~(1.U << temp_scq_head),
                                          next_live_tag_mask)
		new_tag_mask = Mux(commit_capst, new_tag_mask | (1.U << temp_scq_head), new_tag_mask)
    temp_scq_head = Mux(commit_capst, WrapInc(temp_scq_head, numScqEntries), temp_scq_head)

		when (commit_capst) {
			printf("[%d] new_tag_mask: %x next_live_tag_mask: %x\n", io.core.tsc_reg, new_tag_mask, next_live_tag_mask)
		}
	}

	scq_head := temp_scq_head
  num_cstr := Mux(initStat, io.core.dpt_csrs.num_cstr, temp_num_cstr)
  num_cclr := Mux(initStat, io.core.dpt_csrs.num_cclr, temp_num_cclr)
  num_scq_itr := Mux(initStat, io.core.dpt_csrs.num_scq_itr, temp_num_scq_itr)
  //yh+end

  def widthMap[T <: Data](f: Int => T) = VecInit((0 until memWidth).map(f))


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Enqueue new entries
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // This is a newer store than existing loads, so clear the bit in all the store dependency masks
  for (i <- 0 until numLdqEntries)
  {
    when (clear_store)
    {
      ldq(i).bits.st_dep_mask := ldq(i).bits.st_dep_mask & ~(1.U << stq_head)
    }
  }

  //yh+begin
  for (i <- 0 until numSlqEntries) {
    //when (clear_capst) {
      slq(i).bits.tag_dep_mask := (slq(i).bits.tag_dep_mask & ~new_tag_mask)
    //}
		when (new_tag_mask =/= 0.U) {
			printf("[%d] Update tag mask slq(%d) (%x->%x) new_tag_mask: %x\n",
							io.core.tsc_reg, i.U, slq(i).bits.tag_dep_mask,
							slq(i).bits.tag_dep_mask & ~new_tag_mask, new_tag_mask)
		}
  }

  for (i <- 0 until numSsqEntries) {
    //when (clear_capst) {
      ssq(i).bits.tag_dep_mask := (ssq(i).bits.tag_dep_mask & ~new_tag_mask)
    //}
		when (new_tag_mask =/= 0.U) {
			printf("[%d] Update tag mask ssq(%d) (%x->%x) new_tag_mask: %x\n",
							io.core.tsc_reg, i.U, ssq(i).bits.tag_dep_mask,
							ssq(i).bits.tag_dep_mask & ~new_tag_mask, new_tag_mask)
		}
  }
  //yh+end

  // Decode stage
  var ld_enq_idx = ldq_tail
  var st_enq_idx = stq_tail

  val stq_nonempty = (0 until numStqEntries).map{ i => stq(i).valid }.reduce(_||_) =/= 0.U

  var ldq_full = Bool()
  var stq_full = Bool()

  for (w <- 0 until coreWidth)
  {
    ldq_full = WrapInc(ld_enq_idx, numLdqEntries) === ldq_head
    io.core.ldq_full(w)    := ldq_full
    io.core.dis_ldq_idx(w) := ld_enq_idx

    stq_full = WrapInc(st_enq_idx, numStqEntries) === stq_head
    io.core.stq_full(w)    := stq_full
    io.core.dis_stq_idx(w) := st_enq_idx

    val dis_ld_val = io.core.dis_uops(w).valid && io.core.dis_uops(w).bits.uses_ldq && !io.core.dis_uops(w).bits.exception
    //yh-val dis_st_val = io.core.dis_uops(w).valid && io.core.dis_uops(w).bits.uses_stq && !io.core.dis_uops(w).bits.exception
		//yh+begin
    val dis_st_val = (io.core.dis_uops(w).valid && io.core.dis_uops(w).bits.uses_stq &&
                      !io.core.dis_uops(w).bits.is_cap && !io.core.dis_uops(w).bits.exception)
		//yh+end
    when (dis_ld_val)
    {
      ldq(ld_enq_idx).valid                := true.B
      ldq(ld_enq_idx).bits.uop             := io.core.dis_uops(w).bits
      ldq(ld_enq_idx).bits.youngest_stq_idx  := st_enq_idx
      ldq(ld_enq_idx).bits.st_dep_mask     := next_live_store_mask

      ldq(ld_enq_idx).bits.addr.valid      := false.B
      ldq(ld_enq_idx).bits.executed        := false.B
      ldq(ld_enq_idx).bits.succeeded       := false.B
      ldq(ld_enq_idx).bits.order_fail      := false.B
      ldq(ld_enq_idx).bits.observed        := false.B
      ldq(ld_enq_idx).bits.forward_std_val := false.B

      assert (ld_enq_idx === io.core.dis_uops(w).bits.ldq_idx, "[lsu] mismatch enq load tag.")
      assert (!ldq(ld_enq_idx).valid, "[lsu] Enqueuing uop is overwriting ldq entries")
    }
      .elsewhen (dis_st_val)
    {
      stq(st_enq_idx).valid           := true.B
      stq(st_enq_idx).bits.uop        := io.core.dis_uops(w).bits
      stq(st_enq_idx).bits.addr.valid := false.B
      stq(st_enq_idx).bits.data.valid := false.B
      stq(st_enq_idx).bits.committed  := false.B
      stq(st_enq_idx).bits.succeeded  := false.B

      assert (st_enq_idx === io.core.dis_uops(w).bits.stq_idx, "[lsu] mismatch enq store tag.")
      assert (!stq(st_enq_idx).valid, "[lsu] Enqueuing uop is overwriting stq entries")
    }

    ld_enq_idx = Mux(dis_ld_val, WrapInc(ld_enq_idx, numLdqEntries),
                                 ld_enq_idx)

    next_live_store_mask = Mux(dis_st_val, next_live_store_mask | (1.U << st_enq_idx),
                                           next_live_store_mask)
    st_enq_idx = Mux(dis_st_val, WrapInc(st_enq_idx, numStqEntries),
                                 st_enq_idx)

    assert(!(dis_ld_val && dis_st_val), "A UOP is trying to go into both the LDQ and the STQ")
  }

  ldq_tail := ld_enq_idx
  stq_tail := st_enq_idx

  //yh+begin
  var sl_enq_idx = slq_tail
  var ss_enq_idx = ssq_tail
  var sc_enq_idx = scq_tail

  val slq_nonempty = (0 until numSlqEntries).map{ i => slq(i).valid }.reduce(_||_) =/= 0.U
  val ssq_nonempty = (0 until numSsqEntries).map{ i => ssq(i).valid }.reduce(_||_) =/= 0.U
  val scq_nonempty = (0 until numScqEntries).map{ i => scq(i).valid }.reduce(_||_) =/= 0.U

  var slq_full = Bool()
  var ssq_full = Bool()
  var scq_full = Bool()

  for (w <- 0 until coreWidth)
  {
    slq_full = WrapInc(sl_enq_idx, numSlqEntries) === slq_head
    io.core.slq_full(w)    := slq_full
    io.core.dis_slq_idx(w) := sl_enq_idx

    ssq_full = WrapInc(ss_enq_idx, numSsqEntries) === ssq_head
    io.core.ssq_full(w)    := ssq_full
    io.core.dis_ssq_idx(w) := ss_enq_idx

    scq_full = WrapInc(sc_enq_idx, numScqEntries) === scq_head
    io.core.scq_full(w)    := scq_full
    io.core.dis_scq_idx(w) := sc_enq_idx

    val dis_sl_val = (io.core.dis_uops(w).valid && io.core.dis_uops(w).bits.uses_ldq &&
                        io.core.dis_uops(w).bits.needCC && !io.core.dis_uops(w).bits.exception)
    val dis_ss_val = (io.core.dis_uops(w).valid && io.core.dis_uops(w).bits.uses_stq &&
                        io.core.dis_uops(w).bits.needCC && !io.core.dis_uops(w).bits.is_cap &&
												!io.core.dis_uops(w).bits.exception)
    val dis_sc_val = (io.core.dis_uops(w).valid && io.core.dis_uops(w).bits.is_cap /* cstr, cclr */ &&
                        !io.core.dis_uops(w).bits.exception)

    when (dis_sl_val)
    {
      slq(sl_enq_idx).valid               := true.B
      slq(sl_enq_idx).bits.uop            := io.core.dis_uops(w).bits
      slq(sl_enq_idx).bits.uop.uses_ldq   := true.B
      slq(sl_enq_idx).bits.uop.uses_stq   := false.B
      slq(sl_enq_idx).bits.uop.is_cap     := true.B
      slq(sl_enq_idx).bits.uop.mem_size   := 3.U
      slq(sl_enq_idx).bits.uop.mem_cmd    := rocket.M_XRD
      slq(sl_enq_idx).bits.state        	:= s_init
      slq(sl_enq_idx).bits.tagged         := false.B
      slq(sl_enq_idx).bits.tag_dep_mask   := next_live_tag_mask
      slq(sl_enq_idx).bits.ccache_hit     := false.B
      slq(sl_enq_idx).bits.is_rob_head    := false.B

      printf("[%d] Dispatch slq(%d) tag_mask: %x ldq(%d) rob(%d)\n",
							io.core.tsc_reg, sl_enq_idx, next_live_tag_mask,
							io.core.dis_uops(w).bits.ldq_idx, io.core.dis_uops(w).bits.rob_idx)
    }
      .elsewhen (dis_ss_val)
    {
      ssq(ss_enq_idx).valid             	:= true.B
      ssq(ss_enq_idx).bits.uop            := io.core.dis_uops(w).bits
      ssq(ss_enq_idx).bits.uop.uses_ldq   := true.B
      ssq(ss_enq_idx).bits.uop.uses_stq   := false.B
      ssq(ss_enq_idx).bits.uop.is_cap     := true.B
      ssq(ss_enq_idx).bits.uop.mem_size   := 3.U
      ssq(ss_enq_idx).bits.uop.mem_cmd    := rocket.M_XRD
      ssq(ss_enq_idx).bits.state          := s_init
      ssq(ss_enq_idx).bits.tagged         := false.B
      ssq(ss_enq_idx).bits.tag_dep_mask   := next_live_tag_mask
      ssq(ss_enq_idx).bits.ccache_hit     := false.B
      ssq(ss_enq_idx).bits.is_rob_head    := false.B

      printf("[%d] Dispatch ssq(%d) tag_mask: %x stq(%d) rob(%d)\n",
							io.core.tsc_reg, ss_enq_idx, next_live_tag_mask, io.core.dis_uops(w).bits.stq_idx,
							io.core.dis_uops(w).bits.rob_idx)
    }
      .elsewhen (dis_sc_val)
    {
      scq(sc_enq_idx).valid             	:= true.B
      scq(sc_enq_idx).bits.uop            := io.core.dis_uops(w).bits
      scq(sc_enq_idx).bits.uop.is_cap     := true.B
      scq(sc_enq_idx).bits.uop.mem_size   := 3.U
      scq(sc_enq_idx).bits.data.valid     := false.B
      scq(sc_enq_idx).bits.state          := s_init
      scq(sc_enq_idx).bits.tagged         := false.B
      scq(sc_enq_idx).bits.ccache_hit     := false.B
      scq(sc_enq_idx).bits.is_rob_head    := false.B

      printf("[%d] Dispatch scq(%d) tag_mask: %x stq(%d) rob(%d) cstr: %d cclr: %d mem_cmd: %d\n",
							io.core.tsc_reg, sc_enq_idx, next_live_tag_mask, io.core.dis_uops(w).bits.stq_idx,
							io.core.dis_uops(w).bits.rob_idx, io.core.dis_uops(w).bits.cap_cmd === CAP_CS,
							io.core.dis_uops(w).bits.cap_cmd === CAP_CC, io.core.dis_uops(w).bits.mem_cmd)
    }

    sl_enq_idx = Mux(dis_sl_val, WrapInc(sl_enq_idx, numSlqEntries), sl_enq_idx)

    ss_enq_idx = Mux(dis_ss_val, WrapInc(ss_enq_idx, numSsqEntries), ss_enq_idx)

    next_live_tag_mask = Mux(dis_sc_val, next_live_tag_mask | (1.U << sc_enq_idx),
                              next_live_tag_mask)

    sc_enq_idx = Mux(dis_sc_val, WrapInc(sc_enq_idx, numScqEntries), sc_enq_idx)
  }

  slq_tail := sl_enq_idx
  ssq_tail := ss_enq_idx
  scq_tail := sc_enq_idx

	io.core.slq_nonempty := slq_nonempty
	io.core.ssq_nonempty := ssq_nonempty
	io.core.scq_nonempty := scq_nonempty
  //yh+end

  io.dmem.force_order   := io.core.fence_dmem
  //yh-io.core.fencei_rdy    := !stq_nonempty && io.dmem.ordered
  io.core.fencei_rdy    := !stq_nonempty && !ssq_nonempty && !slq_nonempty && !scq_nonempty && io.dmem.ordered //yh+
  //io.core.fencei_rdy    := !stq_nonempty && io.dmem.ordered //yh+


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Execute stage (access TLB, send requests to Memory)
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // We can only report 1 exception per cycle.
  // Just be sure to report the youngest one
  val mem_xcpt_valid  = Wire(Bool())
  val mem_xcpt_cause  = Wire(UInt())
  val mem_xcpt_uop    = Wire(new MicroOp)
  val mem_xcpt_vaddr  = Wire(UInt())


  //---------------------------------------
  // Can-fire logic and wakeup/retry select
  //
  // First we determine what operations are waiting to execute.
  // These are the "can_fire"/"will_fire" signals

  val will_fire_load_incoming  = Wire(Vec(memWidth, Bool()))
  val will_fire_stad_incoming  = Wire(Vec(memWidth, Bool()))
  val will_fire_sta_incoming   = Wire(Vec(memWidth, Bool()))
  val will_fire_std_incoming   = Wire(Vec(memWidth, Bool()))
  val will_fire_sfence         = Wire(Vec(memWidth, Bool()))
  val will_fire_hella_incoming = Wire(Vec(memWidth, Bool()))
  val will_fire_hella_wakeup   = Wire(Vec(memWidth, Bool()))
  val will_fire_release        = Wire(Vec(memWidth, Bool()))
  val will_fire_load_retry     = Wire(Vec(memWidth, Bool()))
  val will_fire_sta_retry      = Wire(Vec(memWidth, Bool()))
  val will_fire_store_commit   = Wire(Vec(memWidth, Bool()))
  val will_fire_load_wakeup    = Wire(Vec(memWidth, Bool()))

  val exe_req = WireInit(VecInit(io.core.exe.map(_.req)))
  // Sfence goes through all pipes
  for (i <- 0 until memWidth) {
    when (io.core.exe(i).req.bits.sfence.valid) {
      exe_req := VecInit(Seq.fill(memWidth) { io.core.exe(i).req })
    }
  }

  //yh+begin
  val cap_exe_req = WireInit(VecInit(io.core.cap_exe.map(_.cap_req)))
	val clr_needCC_valid = RegInit(VecInit(Seq.fill(memWidth+memWidth){false.B}))
	val clr_needCC_rob_idx = Reg(Vec(memWidth+memWidth, UInt(robAddrSz.W)))
	val clr_needCC_brmask = Reg(Vec(memWidth+memWidth, UInt(maxBrCount.W)))

	for (w <- 0 until memWidth+memWidth) {
		clr_needCC_valid(w) := false.B
		clr_needCC_rob_idx(w) := 0.U
		clr_needCC_brmask(w) := 0.U

		io.core.clr_needCC(w).valid := (clr_needCC_valid(w) &&
		                               !IsKilledByBranch(io.core.brupdate, clr_needCC_brmask(w)) &&
		                               !io.core.exception && !RegNext(io.core.exception) && !RegNext(RegNext(io.core.exception)))
		io.core.clr_needCC(w).bits	:= clr_needCC_rob_idx(w)
	}
  //yh+end

  // -------------------------------
  // Assorted signals for scheduling

  // Don't wakeup a load if we just sent it last cycle or two cycles ago
  // The block_load_mask may be wrong, but the executing_load mask must be accurate
  val block_load_mask    = WireInit(VecInit((0 until numLdqEntries).map(x=>false.B)))
  val p1_block_load_mask = RegNext(block_load_mask)
  val p2_block_load_mask = RegNext(p1_block_load_mask)

 // Prioritize emptying the store queue when it is almost full
  val stq_almost_full = RegNext(WrapInc(WrapInc(st_enq_idx, numStqEntries), numStqEntries) === stq_head ||
                                WrapInc(st_enq_idx, numStqEntries) === stq_head)

  // The store at the commit head needs the DCache to appear ordered
  // Delay firing load wakeups and retries now
  val store_needs_order = WireInit(false.B)

  val ldq_incoming_idx = widthMap(i => exe_req(i).bits.uop.ldq_idx)
  val ldq_incoming_e   = widthMap(i => ldq(ldq_incoming_idx(i)))

  val stq_incoming_idx = widthMap(i => exe_req(i).bits.uop.stq_idx)
  val stq_incoming_e   = widthMap(i => stq(stq_incoming_idx(i)))

  val ldq_retry_idx = RegNext(AgePriorityEncoder((0 until numLdqEntries).map(i => {
    val e = ldq(i).bits
    val block = block_load_mask(i) || p1_block_load_mask(i)
    e.addr.valid && e.addr_is_virtual && !block
  }), ldq_head))
  val ldq_retry_e            = ldq(ldq_retry_idx)

  val stq_retry_idx = RegNext(AgePriorityEncoder((0 until numStqEntries).map(i => {
    val e = stq(i).bits
    e.addr.valid && e.addr_is_virtual
  }), stq_commit_head))
  val stq_retry_e   = stq(stq_retry_idx)

  val stq_commit_e  = stq(stq_execute_head)

  val ldq_wakeup_idx = RegNext(AgePriorityEncoder((0 until numLdqEntries).map(i=> {
    val e = ldq(i).bits
    val block = block_load_mask(i) || p1_block_load_mask(i)
    e.addr.valid && !e.executed && !e.succeeded && !e.addr_is_virtual && !block
  }), ldq_head))
  val ldq_wakeup_e   = ldq(ldq_wakeup_idx)

	//yh+begin
	//////////////////////////////////
	// 1. Can update queue entries? //
	//////////////////////////////////
	// SLQ
  val slq_update_idx = RegNext(AgePriorityEncoder((0 until numSlqEntries).map(i => {
    val e = slq(i).bits
		e.state === s_update
  }), slq_head))
  val slq_update_e = slq(slq_update_idx)

	val can_update_ldchk = widthMap(w =>
													slq_update_e.valid									        								&&
													(slq_update_e.bits.state === s_update)	    								&&
													!io.core.exception                  												&&
													(ignoreDepMask || slq_update_e.bits.tag_dep_mask.asUInt === 0.U) &&
													!IsKilledByBranch(io.core.brupdate, slq_update_e.bits.uop)	&&
													(w == 0).B)

	// SSQ
  val ssq_update_idx = RegNext(AgePriorityEncoder((0 until numSsqEntries).map(i => {
    val e = ssq(i).bits
		e.state === s_update
  }), ssq_head))
  val ssq_update_e = ssq(ssq_update_idx)

	val can_update_stchk = widthMap(w =>
													ssq_update_e.valid									        								&&
													(ssq_update_e.bits.state === s_update)	    								&&
													!io.core.exception                  												&&
													(ignoreDepMask || ssq_update_e.bits.tag_dep_mask.asUInt === 0.U) &&
													!IsKilledByBranch(io.core.brupdate, ssq_update_e.bits.uop)	&&
													(w == 0).B)

	// SCQ
  val scq_update_e = scq(scq_head)

	val can_update_capst = widthMap(w =>
													scq_update_e.valid									        								&&
													(scq_update_e.bits.state === s_update)	    								&&
													!io.core.exception                  												&&
													!IsKilledByBranch(io.core.brupdate, scq_update_e.bits.uop)	&&
													(w == 0).B)

	for (w <- 0 until memWidth) {
		when (can_update_ldchk(w) || can_update_stchk(w) || can_update_capst(w)) {
			printf("can_update_ldchk: %d can_update_stchk: %d can_update_capst: %d\n",
							can_update_ldchk(w), can_update_stchk(w), can_update_capst(w))
		}
	}

  // Store/Clear Head Buffer
  //val shb_meta = Reg(Vec(256, UInt((tagWidth/2).W)))
  val shb_data = Reg(Vec(256, UInt(wayAddrSz.W)))
  //val chb_meta = Reg(Vec(256, UInt((tagWidth/2).W)))
  //val chb_data = Reg(Vec(256, UInt(wayAddrSz.W)))

  // Capability Cache
  val ccache_meta = Reg(Vec(256, UInt((tagWidth/2).W)))
	val ccache_lbnd = Reg(Vec(256, UInt(vaddrBits.W)))
	val ccache_ubnd = Reg(Vec(256, UInt(vaddrBits.W)))

	for (w <- 0 until memWidth) {
		// Update SLQ
		when (can_update_ldchk(w)) {
			val tag = slq_update_e.bits.tag
			val tagged = slq_update_e.bits.tagged
			val high4 = tag(tagWidth-1,tagWidth/2)
			val low4 = tag(tagWidth/2-1,0)
			val meta = ccache_meta(low4)
			val low_bnd = (ccache_lbnd(low4) & bounds_margin)
			//val upp_bnd = (ccache_ubnd(low4) + 1.U)
			val upp_bnd = (ccache_ubnd(low4) & bounds_margin)
			val addr = slq_update_e.bits.addr
			val hit = (high4 === meta &&
								(low_bnd(vaddrBits-1,0) <= addr(vaddrBits-1,0) && addr(vaddrBits-1,0) <= upp_bnd(vaddrBits-1,0)))

			slq_update_e.bits.way					:= 0.U
			slq_update_e.bits.state				:= Mux(hit || !tagged, s_done, s_fire)
			slq_update_e.bits.ccache_hit	:= (tagged & hit)

			when (tagged) {
				val tag = slq_update_e.bits.tag
				when (hit) { printf("[%d] C-Cache Hit! ", io.core.tsc_reg) }
				.otherwise { printf("[%d] C-Cache Miss! ", io.core.tsc_reg) }
				printf("slq(%d) tag: %x addr: %x, low_bnd: %x, upp_bnd: %x\n",
								slq_update_idx, tag, addr, low_bnd, upp_bnd)
			}

			// Clear needCC
			clr_needCC_valid(w)	:= (!tagged || hit)
			clr_needCC_rob_idx(w)	:= slq_update_e.bits.uop.rob_idx
			clr_needCC_brmask(w) := GetNewBrMask(io.core.brupdate, slq_update_e.bits.uop)
			when (!tagged || hit) {
				printf("[%d] Clear needCC(1) slq(%d) rob(%d)\n", io.core.tsc_reg, slq_update_idx, slq_update_e.bits.uop.rob_idx)
			}
		// Update SSQ
		} .elsewhen (can_update_stchk(w)) {
			val tag = ssq_update_e.bits.tag
			val tagged = ssq_update_e.bits.tagged
			val high4 = tag(tagWidth-1,tagWidth/2)
			val low4 = tag(tagWidth/2-1,0)
			val meta = ccache_meta(low4)
			val low_bnd = ccache_lbnd(low4)
			val upp_bnd = ccache_ubnd(low4)
			val addr = ssq_update_e.bits.addr
			val hit = (high4 === meta &&
								(low_bnd(vaddrBits-1,0) <= addr(vaddrBits-1,0) && addr(vaddrBits-1,0) <= upp_bnd(vaddrBits-1,0)))

			ssq_update_e.bits.way					:= 0.U
			ssq_update_e.bits.state				:= Mux(hit || !tagged, s_done, s_fire)
			ssq_update_e.bits.ccache_hit	:= (tagged & hit)

			when (tagged) {
				val tag = ssq_update_e.bits.tag
				when (hit) { printf("[%d] C-Cache Hit! ", io.core.tsc_reg) }
				.otherwise { printf("[%d] C-Cache Miss! ", io.core.tsc_reg) }
				printf("ssq(%d) tag: %x addr: %x, low_bnd: %x, upp_bnd: %x\n",
								ssq_update_idx, tag, addr, low_bnd, upp_bnd)
			}

			// Clear needCC
			clr_needCC_valid(w)	:= (!tagged || hit)
			clr_needCC_rob_idx(w)	:= ssq_update_e.bits.uop.rob_idx
			clr_needCC_brmask(w) := GetNewBrMask(io.core.brupdate, ssq_update_e.bits.uop)
			when (!tagged || hit) {
				printf("[%d] Clear needCC(1) ssq(%d) rob(%d)\n", io.core.tsc_reg, ssq_update_idx, ssq_update_e.bits.uop.rob_idx)
			}
		}

		// Update SCQ
		when (can_update_capst(w)) {
			val tag = scq_update_e.bits.tag
			//val high4 = tag(tagWidth-1,tagWidth/2)
			val low4 = tag(tagWidth/2-1,0)
			val is_cstr = (scq_update_e.bits.uop.cap_cmd === CAP_CS)
			val is_cclr = (scq_update_e.bits.uop.cap_cmd === CAP_CC)
			//val meta = Mux(is_cstr, shb_meta(low4), chb_meta(low4))
			//val hit = (high4 === meta)
			//val cstr_head = WrapIncWay(shb_data(low4), scq_update_e.bits.num_ways)
			//val cclr_head = chb_data(low4)
			val head = shb_data(low4)

			//scq_update_e.bits.way			:= Mux(hit, Mux(is_cstr, cstr_head, cclr_head), 0.U)
			scq_update_e.bits.way			:= Mux(is_cstr, WrapIncWay(head, scq_update_e.bits.num_ways), head)
			scq_update_e.bits.state		:= s_fire

			//when (is_cstr) {
				//printf("[%d] SHB Lookup! scq(%d) cstr_head: %d\n", io.core.tsc_reg, scq_head, cstr_head)
				printf("[%d] SHB Lookup! scq(%d) cstr_head: %d\n", io.core.tsc_reg, scq_head, head)
			//} .otherwise {
			//	printf("[%d] CHB Lookup! scq(%d) cclr_head: %d\n", io.core.tsc_reg, scq_head, cclr_head)
			//}
		}
	}

	//////////////////////////////////
	// 2. Can fire cap requests?    //
	//////////////////////////////////
	// SLQ
  val block_ldchk_mask    = WireInit(VecInit((0 until numSlqEntries).map(x=>false.B)))
  val p1_block_ldchk_mask = RegNext(block_ldchk_mask)
  val p2_block_ldchk_mask = RegNext(p1_block_ldchk_mask)

  val slq_fire_idx = RegNext(AgePriorityEncoder((0 until numSlqEntries).map(i => {
    val e = slq(i).bits
    val block = block_ldchk_mask(i) || p1_block_ldchk_mask(i)
		(e.state === s_fire && !block)
  }), slq_head))
  val slq_fire_e = slq(slq_fire_idx)

	val can_fire_ldchk = widthMap(w =>
												slq_fire_e.valid									        								&&
												(!disableSpec || slq_fire_e.bits.is_rob_head)							&&
												(slq_fire_e.bits.state === s_fire)	    									&&
												!p1_block_ldchk_mask(slq_fire_idx)  											&&
												!p2_block_ldchk_mask(slq_fire_idx)  											&&
												!io.core.exception                  											&&
												RegNext(dtlb.io.req(w).ready) 														&&
												!store_needs_order                        								&&
												!mem_xcpt_valid                         									&&
												(w == 0).B)

	// SSQ
  val block_stchk_mask    = WireInit(VecInit((0 until numSsqEntries).map(x=>false.B)))
  val p1_block_stchk_mask = RegNext(block_stchk_mask)
  val p2_block_stchk_mask = RegNext(p1_block_stchk_mask)

  val ssq_fire_idx = RegNext(AgePriorityEncoder((0 until numSsqEntries).map(i => {
    val e = ssq(i).bits
    val block = block_stchk_mask(i) || p1_block_stchk_mask(i)
		(e.state === s_fire && !block)
  }), ssq_head))
  val ssq_fire_e = ssq(ssq_fire_idx)

	val can_fire_stchk = widthMap(w =>
												ssq_fire_e.valid									        								&&
												(!disableSpec || ssq_fire_e.bits.is_rob_head)							&&
												(ssq_fire_e.bits.state === s_fire)	    									&&
												!p1_block_stchk_mask(ssq_fire_idx) 												&&
												!p2_block_stchk_mask(ssq_fire_idx) 												&&
												!io.core.exception                  											&&
												RegNext(dtlb.io.req(w).ready) 														&&
												!store_needs_order                        								&&
												!mem_xcpt_valid                         									&&
												(w == 0).B)

	// SCQ
  val block_capst_mask    = WireInit(VecInit((0 until numScqEntries).map(x=>false.B)))
  val p1_block_capst_mask = RegNext(block_capst_mask)
  val p2_block_capst_mask = RegNext(p1_block_capst_mask)

  val scq_fire_e = scq(scq_head)

  val can_fire_capst = widthMap(w =>
												scq_fire_e.valid                      										&&
												scq_fire_e.bits.data.valid																&&
												scq_fire_e.bits.is_rob_head																&&
												(scq_fire_e.bits.state === s_fire ||
													scq_fire_e.bits.state === s_fire2)											&&
												!p1_block_capst_mask(scq_head)  									&&
												!p2_block_capst_mask(scq_head)  									&&
												!io.core.exception                  											&&
												RegNext(dtlb.io.req(w).ready) 														&&
												!store_needs_order                      									&&
												!mem_xcpt_valid                         									&&
												(w == 0).B)

	// Pipeline 1 cycle
  val will_fire_cap_check = Wire(Vec(memWidth, Bool()))
  val will_fire_cap_store = Wire(Vec(memWidth, Bool()))

	// Cap Check
	val slq_fire_uop = widthMap(w => slq_fire_e.bits.uop)
	val slq_fire_vaddr = widthMap(w => slq_fire_e.bits.cmt_addr + (slq_fire_e.bits.way << wayOffset))

	val ssq_fire_uop = widthMap(w => ssq_fire_e.bits.uop)
	val ssq_fire_vaddr = widthMap(w => ssq_fire_e.bits.cmt_addr + (ssq_fire_e.bits.way << wayOffset))

	val can_fire_cap_check = Reg(Vec(memWidth, Bool()))
	val cap_check_uop = Reg(Vec(memWidth, new MicroOp))
  val cap_check_vaddr = Reg(Vec(memWidth, UInt((vaddrBits+1).W)))
	val cap_check_idx = Reg(Vec(memWidth, UInt(ssqAddrSz.W)))
  val can_fire_ldchk_p1 = Reg(Vec(memWidth, Bool()))
	val fire_ldchk_valid = Wire(Vec(memWidth, Bool()))
	val fire_stchk_valid = Wire(Vec(memWidth, Bool()))

	// TODO: add w == 0 condition
	for (w <- 0 until memWidth) {
		fire_ldchk_valid(w) := (can_fire_ldchk(w) && !IsKilledByBranch(io.core.brupdate, slq_fire_e.bits.uop.br_mask))
		fire_stchk_valid(w) := (can_fire_stchk(w) && !IsKilledByBranch(io.core.brupdate, ssq_fire_e.bits.uop.br_mask))

		can_fire_cap_check(w) := (!will_fire_cap_check(w) && (fire_ldchk_valid(w) || fire_stchk_valid(w)))
		cap_check_uop(w) := Mux(fire_ldchk_valid(w), slq_fire_uop(w), ssq_fire_uop(w))
		cap_check_uop(w).br_mask := Mux(fire_ldchk_valid(w), GetNewBrMask(io.core.brupdate, slq_fire_uop(w).br_mask), 
																		GetNewBrMask(io.core.brupdate, ssq_fire_uop(w).br_mask))
		cap_check_vaddr(w) := Mux(fire_ldchk_valid(w), slq_fire_vaddr(w), ssq_fire_vaddr(w))
		cap_check_idx(w) := Mux(fire_ldchk_valid(w), slq_fire_idx, ssq_fire_idx)
		can_fire_ldchk_p1(w) := fire_ldchk_valid(w)
	}

	// Cap Store
	val can_fire_cap_store = Reg(Vec(memWidth, Bool()))
	val cap_store_uop = Reg(Vec(memWidth, new MicroOp))
	for (w <- 0 until memWidth) {
		can_fire_cap_store(w) := (!will_fire_cap_store(w) && can_fire_capst(w) &&
		                         !IsKilledByBranch(io.core.brupdate, scq_fire_e.bits.uop.br_mask))
		cap_store_uop(w) := scq_fire_e.bits.uop
		cap_store_uop(w).br_mask := GetNewBrMask(io.core.brupdate, scq_fire_e.bits.uop.br_mask)
    cap_store_uop(w).mem_cmd := Mux(scq_fire_e.bits.state === s_fire2, rocket.M_XWR,
                                    scq_fire_e.bits.uop.mem_cmd)
    when (can_fire_capst(w)) {
      printf("[%d] state: %d mem_cmd: (%d, %d) bounds_margin: %x\n", io.core.tsc_reg, scq_fire_e.bits.state,
              scq_fire_e.bits.uop.mem_cmd,
              Mux(scq_fire_e.bits.state === s_fire2, rocket.M_XWR, cap_store_uop(w).mem_cmd), bounds_margin)
    }
	}

	val cap_vaddr = (scq_fire_e.bits.cmt_addr + (scq_fire_e.bits.way << wayOffset))
	val cap_store_vaddr = RegNext(widthMap(w => Mux(scq_fire_e.bits.state === s_fire,
													cap_vaddr, cap_vaddr | 8.U)))
	val cap_store_data = RegNext(widthMap(w => Mux(scq_fire_e.bits.state === s_fire, scq_fire_e.bits.addr(vaddrBits-1,0),
                        Mux(scq_fire_e.bits.uop.cap_cmd === CAP_CS,
                          scq_fire_e.bits.addr(vaddrBits-1,0) + scq_fire_e.bits.data.bits(vaddrBits-1,0), 0.U))))
	val cap_store_idx = RegNext(widthMap(w => scq_head))

	for (w <- 0 until memWidth) {
		when (fire_ldchk_valid(w)) {
			printf("[%d] Can fire load check! slq(%d) vaddr: %x tag: %x way: %d\n",
							io.core.tsc_reg, slq_fire_idx, slq_fire_vaddr(w),
							slq_fire_e.bits.tag, slq_fire_e.bits.way)
		} .elsewhen (fire_stchk_valid(w)) {
			printf("[%d] Can fire store check! ssq(%d) vaddr: %x tag: %x way: %d\n",
							io.core.tsc_reg, ssq_fire_idx, ssq_fire_vaddr(w),
							ssq_fire_e.bits.tag, ssq_fire_e.bits.way)
		}

		when (can_fire_capst(w)) {
			printf("[%d] Can fire cap store! scq(%d) vaddr: %x tag: %x way: %d\n",
							io.core.tsc_reg, scq_head,
							Mux(scq_fire_e.bits.state === s_fire, cap_vaddr, cap_vaddr | 8.U),
							scq_fire_e.bits.tag, scq_fire_e.bits.way)
		}
	}

  //printf("[%d] next_live_tag_mask: %x\n", io.core.tsc_reg, next_live_tag_mask)

	//////////////////////////////////
	// 3. Detect Failures    				//
	//////////////////////////////////
	val slq_head_e = slq(slq_head)
	val ssq_head_e = ssq(ssq_head)
	val scq_head_e = scq(scq_head)

	val ldchk_xcpt_valid = (slq_head_e.valid &&
													slq_head_e.bits.state === s_fail &&
													slq_head_e.bits.is_rob_head)

	val stchk_xcpt_valid = (ssq_head_e.valid &&
													ssq_head_e.bits.state === s_fail &&
													ssq_head_e.bits.is_rob_head)

	val capst_xcpt_valid = (scq_head_e.valid &&
													scq_head_e.bits.state === s_fail &&
													scq_head_e.bits.is_rob_head)

 	when (ldchk_xcpt_valid) {
		printf("[%d] Found ldchk_xcpt_valid! slq(%d) rob(%d)\n",
							io.core.tsc_reg, slq_head, slq_head_e.bits.uop.rob_idx)
  } .elsewhen (stchk_xcpt_valid) {
		printf("[%d] Found stchk_xcpt_valid! ssq(%d) rob(%d)\n",
							io.core.tsc_reg, ssq_head, ssq_head_e.bits.uop.rob_idx)
  } .elsewhen (capst_xcpt_valid) {
		printf("[%d] Found capst_xcpt_valid! scq(%d) rob(%d)\n",
							io.core.tsc_reg, scq_head, scq_head_e.bits.uop.rob_idx)
	}

	val dpt_xcpt_valid = RegNext(ldchk_xcpt_valid || stchk_xcpt_valid || capst_xcpt_valid)

	val dpt_xcpt_cause = RegNext(Mux(ldchk_xcpt_valid, rocket.Causes.ldchk_fault.U,
											Mux(stchk_xcpt_valid, rocket.Causes.stchk_fault.U,
												Mux(capst_xcpt_valid && scq_head_e.bits.uop.cap_cmd === CAP_CS,
													rocket.Causes.cstr_fault.U, rocket.Causes.cclr_fault.U))))

	val dpt_xcpt_uop = RegNext(Mux(ldchk_xcpt_valid, slq_head_e.bits.uop,
											Mux(stchk_xcpt_valid, ssq_head_e.bits.uop,
												Mux(capst_xcpt_valid, scq_head_e.bits.uop, NullMicroOp))))

	val dpt_xcpt_addr = RegNext(Mux(ldchk_xcpt_valid, slq_head_e.bits.addr,
											Mux(stchk_xcpt_valid, ssq_head_e.bits.addr,
												Mux(capst_xcpt_valid, scq_head_e.bits.addr, 0.U))))

	val update_shb_valid	= Reg(Vec(memWidth, Bool()))
	//val update_chb_valid	= Reg(Vec(memWidth, Bool()))
	//val update_hb_tag   	= Reg(Vec(memWidth, UInt(tagWidth.W)))
	val update_hb_tag   	= Reg(Vec(memWidth, UInt((tagWidth/2).W)))
	val update_hb_way   	= Reg(Vec(memWidth, UInt(wayAddrSz.W)))
	//TODO val update_hb_dir  		= Reg(Vec(memWidth, Bool()))

	for (w <- 0 until memWidth) {
		update_shb_valid(w) := false.B
		//update_chb_valid(w) := false.B
	}
	//yh+end

  // -----------------------
  // Determine what can fire

  // Can we fire a incoming load
  val can_fire_load_incoming = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_load)

  // Can we fire an incoming store addrgen + store datagen
  val can_fire_stad_incoming = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_sta
                                                              && exe_req(w).bits.uop.ctrl.is_std)

  // Can we fire an incoming store addrgen
  val can_fire_sta_incoming  = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_sta
                                                              && !exe_req(w).bits.uop.ctrl.is_std)

  // Can we fire an incoming store datagen
  val can_fire_std_incoming  = widthMap(w => exe_req(w).valid && exe_req(w).bits.uop.ctrl.is_std
                                                              && !exe_req(w).bits.uop.ctrl.is_sta)

  // Can we fire an incoming sfence
  val can_fire_sfence        = widthMap(w => exe_req(w).valid && exe_req(w).bits.sfence.valid)

  // Can we fire a request from dcache to release a line
  // This needs to go through LDQ search to mark loads as dangerous
  val can_fire_release       = widthMap(w => (w == memWidth-1).B && io.dmem.release.valid)
  io.dmem.release.ready     := will_fire_release.reduce(_||_)

  // Can we retry a load that missed in the TLB
  val can_fire_load_retry    = widthMap(w =>
                               ( ldq_retry_e.valid                            &&
                                 ldq_retry_e.bits.addr.valid                  &&
                                 ldq_retry_e.bits.addr_is_virtual             &&
                                !p1_block_load_mask(ldq_retry_idx)            &&
                                !p2_block_load_mask(ldq_retry_idx)            &&
                                RegNext(dtlb.io.miss_rdy)                     &&
                                !store_needs_order                            &&
                                (w == memWidth-1).B                           && // TODO: Is this best scheduling?
                                !ldq_retry_e.bits.order_fail))

  // Can we retry a store addrgen that missed in the TLB
  // - Weird edge case when sta_retry and std_incoming for same entry in same cycle. Delay this
  val can_fire_sta_retry     = widthMap(w =>
                               ( stq_retry_e.valid                            &&
                                 stq_retry_e.bits.addr.valid                  &&
                                 stq_retry_e.bits.addr_is_virtual             &&
                                 (w == memWidth-1).B                          &&
                                 RegNext(dtlb.io.miss_rdy)                    &&
                                 !(widthMap(i => (i != w).B               &&
                                                 can_fire_std_incoming(i) &&
                                                 stq_incoming_idx(i) === stq_retry_idx).reduce(_||_))
                               ))
  // Can we commit a store
  val can_fire_store_commit  = widthMap(w =>
                               ( stq_commit_e.valid                           &&
                                !stq_commit_e.bits.uop.is_fence               &&
                                !mem_xcpt_valid                               &&
                                !stq_commit_e.bits.uop.exception              &&
                                (w == 0).B                                    &&
                                (stq_commit_e.bits.committed || ( stq_commit_e.bits.uop.is_amo      &&
                                                                  stq_commit_e.bits.addr.valid      &&
                                                                 !stq_commit_e.bits.addr_is_virtual &&
                                                                  stq_commit_e.bits.data.valid))))

  // Can we wakeup a load that was nack'd
  val block_load_wakeup = WireInit(false.B)
  val can_fire_load_wakeup = widthMap(w =>
                             ( ldq_wakeup_e.valid                                      &&
                               ldq_wakeup_e.bits.addr.valid                            &&
                              !ldq_wakeup_e.bits.succeeded                             &&
                              !ldq_wakeup_e.bits.addr_is_virtual                       &&
                              !ldq_wakeup_e.bits.executed                              &&
                              !ldq_wakeup_e.bits.order_fail                            &&
                              !p1_block_load_mask(ldq_wakeup_idx)                      &&
                              !p2_block_load_mask(ldq_wakeup_idx)                      &&
                              !store_needs_order                                       &&
                              !block_load_wakeup                                       &&
                              (w == memWidth-1).B                                      &&
                              (!ldq_wakeup_e.bits.addr_is_uncacheable || (io.core.commit_load_at_rob_head &&
                                                                          ldq_head === ldq_wakeup_idx &&
                                                                          ldq_wakeup_e.bits.st_dep_mask.asUInt === 0.U))))

  // Can we fire an incoming hellacache request
  val can_fire_hella_incoming  = WireInit(widthMap(w => false.B)) // This is assigned to in the hellashim ocntroller

  // Can we fire a hellacache request that the dcache nack'd
  val can_fire_hella_wakeup    = WireInit(widthMap(w => false.B)) // This is assigned to in the hellashim controller

  //---------------------------------------------------------
  // Controller logic. Arbitrate which request actually fires

  val exe_tlb_valid = Wire(Vec(memWidth, Bool()))
  for (w <- 0 until memWidth) {
    var tlb_avail  = true.B
    var dc_avail   = true.B
    var lcam_avail = true.B
    var rob_avail  = true.B

    def lsu_sched(can_fire: Bool, uses_tlb:Boolean, uses_dc:Boolean, uses_lcam: Boolean, uses_rob:Boolean): Bool = {
      val will_fire = can_fire && !(uses_tlb.B && !tlb_avail) &&
                                  !(uses_lcam.B && !lcam_avail) &&
                                  !(uses_dc.B && !dc_avail) &&
                                  !(uses_rob.B && !rob_avail)
      tlb_avail  = tlb_avail  && !(will_fire && uses_tlb.B)
      lcam_avail = lcam_avail && !(will_fire && uses_lcam.B)
      dc_avail   = dc_avail   && !(will_fire && uses_dc.B)
      rob_avail  = rob_avail  && !(will_fire && uses_rob.B)
      dontTouch(will_fire) // dontTouch these so we can inspect the will_fire signals
      will_fire
    }

    // The order of these statements is the priority
    // Some restrictions
    //  - Incoming ops must get precedence, can't backpresure memaddrgen
    //  - Incoming hellacache ops must get precedence over retrying ops (PTW must get precedence over retrying translation)
    // Notes on performance
    //  - Prioritize releases, this speeds up cache line writebacks and refills
    //  - Store commits are lowest priority, since they don't "block" younger instructions unless stq fills up
    will_fire_load_incoming (w) := lsu_sched(can_fire_load_incoming (w) , true , true , true , false) // TLB , DC , LCAM
    will_fire_stad_incoming (w) := lsu_sched(can_fire_stad_incoming (w) , true , false, true , true)  // TLB ,    , LCAM , ROB
    will_fire_sta_incoming  (w) := lsu_sched(can_fire_sta_incoming  (w) , true , false, true , true)  // TLB ,    , LCAM , ROB
    will_fire_std_incoming  (w) := lsu_sched(can_fire_std_incoming  (w) , false, false, false, true)  //                 , ROB
    will_fire_sfence        (w) := lsu_sched(can_fire_sfence        (w) , true , false, false, true)  // TLB ,    ,      , ROB
    will_fire_release       (w) := lsu_sched(can_fire_release       (w) , false, false, true , false) //            LCAM
    will_fire_hella_incoming(w) := lsu_sched(can_fire_hella_incoming(w) , true , true , false, false) // TLB , DC
    will_fire_hella_wakeup  (w) := lsu_sched(can_fire_hella_wakeup  (w) , false, true , false, false) //     , DC
    will_fire_load_retry    (w) := lsu_sched(can_fire_load_retry    (w) , true , true , true , false) // TLB , DC , LCAM
    will_fire_sta_retry     (w) := lsu_sched(can_fire_sta_retry     (w) , true , false, true , true)  // TLB ,    , LCAM , ROB // TODO: This should be higher priority
    will_fire_load_wakeup   (w) := lsu_sched(can_fire_load_wakeup   (w) , false, true , true , false) //     , DC , LCAM1
    will_fire_store_commit  (w) := lsu_sched(can_fire_store_commit  (w) , false, true , false, false) //     , DC
		//yh+begin
    will_fire_cap_check     (w) := lsu_sched(can_fire_cap_check     (w) , true , true , false, false) // TLB , DC
    will_fire_cap_store     (w) := lsu_sched(can_fire_cap_store     (w) , true , true , false, false) // TLB , DC

    when (will_fire_cap_check(w)) {
      when (can_fire_ldchk_p1(w)) {
        block_ldchk_mask(cap_check_idx(w)) := true.B
      } .otherwise {
        assert(!can_fire_ldchk_p1(w))
        block_stchk_mask(cap_check_idx(w)) := true.B
      }
    } .elsewhen (will_fire_cap_store(w)) {
      block_capst_mask(cap_store_idx(w)) := true.B
		}
		//yh+end

    assert(!(exe_req(w).valid && !(will_fire_load_incoming(w) || will_fire_stad_incoming(w) || will_fire_sta_incoming(w) || will_fire_std_incoming(w) || will_fire_sfence(w))))

    when (will_fire_load_wakeup(w)) {
      block_load_mask(ldq_wakeup_idx)           := true.B
    } .elsewhen (will_fire_load_incoming(w)) {
      block_load_mask(exe_req(w).bits.uop.ldq_idx) := true.B
    } .elsewhen (will_fire_load_retry(w)) {
      block_load_mask(ldq_retry_idx)            := true.B
    }
    exe_tlb_valid(w) := !tlb_avail
  }
  assert((memWidth == 1).B ||
    (!(will_fire_sfence.reduce(_||_) && !will_fire_sfence.reduce(_&&_)) &&
     !will_fire_hella_incoming.reduce(_&&_) &&
     !will_fire_hella_wakeup.reduce(_&&_)   &&
     !will_fire_load_retry.reduce(_&&_)     &&
     !will_fire_sta_retry.reduce(_&&_)      &&
     !will_fire_store_commit.reduce(_&&_)   &&
     !will_fire_load_wakeup.reduce(_&&_)),
    "Some operations is proceeding down multiple pipes")

  require(memWidth <= 2)

  //--------------------------------------------
  // TLB Access

  assert(!(hella_state =/= h_ready && hella_req.cmd === rocket.M_SFENCE),
    "SFENCE through hella interface not supported")

  val exe_tlb_uop = widthMap(w =>
                    Mux(will_fire_load_incoming (w) ||
                        will_fire_stad_incoming (w) ||
                        will_fire_sta_incoming  (w) ||
                        will_fire_sfence        (w)  , exe_req(w).bits.uop,
                    Mux(will_fire_load_retry    (w)  , ldq_retry_e.bits.uop,
                    Mux(will_fire_sta_retry     (w)  , stq_retry_e.bits.uop,
                    Mux(will_fire_hella_incoming(w)  , NullMicroOp,
										//yh+begin
                    Mux(will_fire_cap_check    (w)   , cap_check_uop(w),
                    Mux(will_fire_cap_store    (w)   , cap_store_uop(w),
                                                       NullMicroOp)))))))
										//yh+end
                    //yh-                                   NullMicroOp)))))

  val exe_tlb_vaddr = widthMap(w =>
                    Mux(will_fire_load_incoming (w) ||
                        will_fire_stad_incoming (w) ||
                        will_fire_sta_incoming  (w)  , exe_req(w).bits.addr,
                    Mux(will_fire_sfence        (w)  , exe_req(w).bits.sfence.bits.addr,
                    Mux(will_fire_load_retry    (w)  , ldq_retry_e.bits.addr.bits,
                    Mux(will_fire_sta_retry     (w)  , stq_retry_e.bits.addr.bits,
                    Mux(will_fire_hella_incoming(w)  , hella_req.addr,
										//yh+begin
										Mux(will_fire_cap_check		(w)    , cap_check_vaddr(w),
										Mux(will_fire_cap_store		(w)    , cap_store_vaddr(w),
                                                       0.U))))))))
										//yh+end
                    //yh-                                   0.U))))))

  val exe_sfence = WireInit((0.U).asTypeOf(Valid(new rocket.SFenceReq)))
  for (w <- 0 until memWidth) {
    when (will_fire_sfence(w)) {
      exe_sfence := exe_req(w).bits.sfence
    }
  }

  val exe_size   = widthMap(w =>
                   Mux(will_fire_load_incoming (w) ||
                       will_fire_stad_incoming (w) ||
                       will_fire_sta_incoming  (w) ||
                       will_fire_sfence        (w) ||
                       will_fire_load_retry    (w) ||
											 //yh+begin
									     will_fire_cap_check		 (w) ||
									     will_fire_cap_store		 (w) ||
		 									 //yh+end
                       will_fire_sta_retry     (w)  , exe_tlb_uop(w).mem_size,
                   Mux(will_fire_hella_incoming(w)  , hella_req.size,
                                                      0.U)))
  val exe_cmd    = widthMap(w =>
                   Mux(will_fire_load_incoming (w) ||
                       will_fire_stad_incoming (w) ||
                       will_fire_sta_incoming  (w) ||
                       will_fire_sfence        (w) ||
                       will_fire_load_retry    (w) ||
											 //yh+begin
									     will_fire_cap_check		 (w) ||
									     will_fire_cap_store		 (w) ||
		 									 //yh+end
                       will_fire_sta_retry     (w)  , exe_tlb_uop(w).mem_cmd,
                   Mux(will_fire_hella_incoming(w)  , hella_req.cmd,
                                                      0.U)))

  val exe_passthr= widthMap(w =>
                   Mux(will_fire_hella_incoming(w)  , hella_req.phys,
                                                      false.B))
  val exe_kill   = widthMap(w =>
                   Mux(will_fire_hella_incoming(w)  , io.hellacache.s1_kill,
                                                      false.B))

	//yh+begin
	val cap_ldchk_incoming = widthMap(w => cap_exe_req(w).valid && cap_exe_req(w).bits.uop.cap_cmd === CAP_LD)

	val cap_stchk_incoming = widthMap(w => cap_exe_req(w).valid && cap_exe_req(w).bits.uop.cap_cmd === CAP_ST
																					&& cap_exe_req(w).bits.uop.ctrl.is_sta)

	// cstr, cclr
	val cap_capsta_incoming = widthMap(w => cap_exe_req(w).valid && cap_exe_req(w).bits.uop.cap_cmd(1)
																					&& cap_exe_req(w).bits.uop.ctrl.is_sta)

	val cap_capstd_incoming = widthMap(w => cap_exe_req(w).valid && cap_exe_req(w).bits.uop.cap_cmd(1)
																					&& cap_exe_req(w).bits.uop.ctrl.is_std)

  // Write C-Cache
  val ccache_write_lbnd_val = Reg(Vec(memWidth, Bool()))
  val ccache_write_ubnd_val = Reg(Vec(memWidth, Bool()))
  val ccache_write_tag = Reg(Vec(memWidth, UInt(tagWidth.W)))
  val ccache_write_lbnd_data = Reg(Vec(memWidth, UInt(vaddrBits.W)))
  val ccache_write_ubnd_data = Reg(Vec(memWidth, UInt(vaddrBits.W)))

  for (w <- 0 until memWidth) {
    ccache_write_lbnd_val(w) := false.B
    ccache_write_ubnd_val(w) := false.B
    ccache_write_tag(w) := 0.U
    ccache_write_lbnd_data(w) := 0.U
    ccache_write_ubnd_data(w) := 0.U

    when (ccache_write_lbnd_val(w)) {
			ccache_meta(ccache_write_tag(w)(tagWidth/2-1,0)) := ccache_write_tag(w)(tagWidth-1,tagWidth/2)
      ccache_lbnd(ccache_write_tag(w)(tagWidth/2-1,0)) := ccache_write_lbnd_data(w)
      printf("[%d] Write C-Cache! tag: %x lbnd: %x\n",
							io.core.tsc_reg, ccache_write_tag(w), ccache_write_lbnd_data(w))
    }

    when (ccache_write_ubnd_val(w)) {
      ccache_ubnd(ccache_write_tag(w)(tagWidth/2-1,0)) := ccache_write_ubnd_data(w)
      printf("[%d] Write C-Cache! tag: %x ubnd: %x\n",
							io.core.tsc_reg, ccache_write_tag(w), ccache_write_ubnd_data(w))
		}
  }
	//yh+end

  for (w <- 0 until memWidth) {
    dtlb.io.req(w).valid            := exe_tlb_valid(w)
    dtlb.io.req(w).bits.vaddr       := exe_tlb_vaddr(w)
    dtlb.io.req(w).bits.size        := exe_size(w)
    dtlb.io.req(w).bits.cmd         := exe_cmd(w)
    dtlb.io.req(w).bits.passthrough := exe_passthr(w)
    dtlb.io.req(w).bits.v           := io.ptw.status.v
    dtlb.io.req(w).bits.prv         := io.ptw.status.prv
  }
  dtlb.io.kill                      := exe_kill.reduce(_||_)
  dtlb.io.sfence                    := exe_sfence

  // exceptions
  val ma_ld = widthMap(w => will_fire_load_incoming(w) && exe_req(w).bits.mxcpt.valid) // We get ma_ld in memaddrcalc
  val ma_st = widthMap(w => (will_fire_sta_incoming(w) || will_fire_stad_incoming(w)) && exe_req(w).bits.mxcpt.valid) // We get ma_ld in memaddrcalc
  val pf_ld = widthMap(w => dtlb.io.req(w).valid && dtlb.io.resp(w).pf.ld && exe_tlb_uop(w).uses_ldq)
  val pf_st = widthMap(w => dtlb.io.req(w).valid && dtlb.io.resp(w).pf.st && exe_tlb_uop(w).uses_stq)
  val ae_ld = widthMap(w => dtlb.io.req(w).valid && dtlb.io.resp(w).ae.ld && exe_tlb_uop(w).uses_ldq)
  val ae_st = widthMap(w => dtlb.io.req(w).valid && dtlb.io.resp(w).ae.st && exe_tlb_uop(w).uses_stq)

  // TODO check for xcpt_if and verify that never happens on non-speculative instructions.
  val mem_xcpt_valids = RegNext(widthMap(w =>
                     (pf_ld(w) || pf_st(w) || ae_ld(w) || ae_st(w) || ma_ld(w) || ma_st(w)) &&
                     !io.core.exception &&
                     !IsKilledByBranch(io.core.brupdate, exe_tlb_uop(w))))
  val mem_xcpt_uops   = RegNext(widthMap(w => UpdateBrMask(io.core.brupdate, exe_tlb_uop(w))))
  val mem_xcpt_causes = RegNext(widthMap(w =>
    Mux(ma_ld(w), rocket.Causes.misaligned_load.U,
    Mux(ma_st(w), rocket.Causes.misaligned_store.U,
    Mux(pf_ld(w), rocket.Causes.load_page_fault.U,
    Mux(pf_st(w), rocket.Causes.store_page_fault.U,
    Mux(ae_ld(w), rocket.Causes.load_access.U,
                  rocket.Causes.store_access.U)))))))
  val mem_xcpt_vaddrs = RegNext(exe_tlb_vaddr)

  for (w <- 0 until memWidth) {
    assert (!(dtlb.io.req(w).valid && exe_tlb_uop(w).is_fence), "Fence is pretending to talk to the TLB")
    assert (!((will_fire_load_incoming(w) || will_fire_sta_incoming(w) || will_fire_stad_incoming(w)) &&
      exe_req(w).bits.mxcpt.valid && dtlb.io.req(w).valid &&
    !(exe_tlb_uop(w).ctrl.is_load || exe_tlb_uop(w).ctrl.is_sta)),
      "A uop that's not a load or store-address is throwing a memory exception.")
  }

  mem_xcpt_valid := mem_xcpt_valids.reduce(_||_)
  mem_xcpt_cause := mem_xcpt_causes(0)
  mem_xcpt_uop   := mem_xcpt_uops(0)
  mem_xcpt_vaddr := mem_xcpt_vaddrs(0)
  var xcpt_found = mem_xcpt_valids(0)
  var oldest_xcpt_rob_idx = mem_xcpt_uops(0).rob_idx
  for (w <- 1 until memWidth) {
    val is_older = WireInit(false.B)
    when (mem_xcpt_valids(w) &&
      (IsOlder(mem_xcpt_uops(w).rob_idx, oldest_xcpt_rob_idx, io.core.rob_head_idx) || !xcpt_found)) {
      is_older := true.B
      mem_xcpt_cause := mem_xcpt_causes(w)
      mem_xcpt_uop   := mem_xcpt_uops(w)
      mem_xcpt_vaddr := mem_xcpt_vaddrs(w)
    }
    xcpt_found = xcpt_found || mem_xcpt_valids(w)
    oldest_xcpt_rob_idx = Mux(is_older, mem_xcpt_uops(w).rob_idx, oldest_xcpt_rob_idx)
  }

  val exe_tlb_miss  = widthMap(w => dtlb.io.req(w).valid && (dtlb.io.resp(w).miss || !dtlb.io.req(w).ready))
  val exe_tlb_paddr = widthMap(w => Cat(dtlb.io.resp(w).paddr(paddrBits-1,corePgIdxBits),
                                        exe_tlb_vaddr(w)(corePgIdxBits-1,0)))
  val exe_tlb_uncacheable = widthMap(w => !(dtlb.io.resp(w).cacheable))

  for (w <- 0 until memWidth) {
    assert (exe_tlb_paddr(w) === dtlb.io.resp(w).paddr || exe_req(w).bits.sfence.valid, "[lsu] paddrs should match.")

    when (mem_xcpt_valids(w))
    {
      //yh-assert(RegNext(will_fire_load_incoming(w) || will_fire_stad_incoming(w) || will_fire_sta_incoming(w) ||
      //yh-  will_fire_load_retry(w) || will_fire_sta_retry(w)))
			//yh+begin
      assert(RegNext(will_fire_load_incoming(w) || will_fire_stad_incoming(w) || will_fire_sta_incoming(w) ||
        will_fire_load_retry(w) || will_fire_sta_retry(w) || will_fire_cap_check(w) || will_fire_cap_store(w)))
			//yh+end
      // Technically only faulting AMOs need this
      assert(mem_xcpt_uops(w).uses_ldq ^ mem_xcpt_uops(w).uses_stq)
      //yh-when (mem_xcpt_uops(w).uses_ldq)
			//yh+begin
			when (mem_xcpt_uops(w).is_cap) {
      } .elsewhen (mem_xcpt_uops(w).uses_ldq)
			//yh+end
      {
        ldq(mem_xcpt_uops(w).ldq_idx).bits.uop.exception := true.B
      }
        .otherwise
      {
        stq(mem_xcpt_uops(w).stq_idx).bits.uop.exception := true.B
      }
    }
  }



  //------------------------------
  // Issue Someting to Memory
  //
  // A memory op can come from many different places
  // The address either was freshly translated, or we are
  // reading a physical address from the LDQ,STQ, or the HellaCache adapter


  // defaults
  io.dmem.brupdate         := io.core.brupdate
  io.dmem.exception      := io.core.exception
  io.dmem.rob_head_idx   := io.core.rob_head_idx
  io.dmem.rob_pnr_idx    := io.core.rob_pnr_idx

  val dmem_req = Wire(Vec(memWidth, Valid(new BoomDCacheReq)))
  io.dmem.req.valid := dmem_req.map(_.valid).reduce(_||_)
  io.dmem.req.bits  := dmem_req
  val dmem_req_fire = widthMap(w => dmem_req(w).valid && io.dmem.req.fire)

  val s0_executing_loads = WireInit(VecInit((0 until numLdqEntries).map(x=>false.B)))


  for (w <- 0 until memWidth) {
    dmem_req(w).valid := false.B
    dmem_req(w).bits.uop   := NullMicroOp
    dmem_req(w).bits.addr  := 0.U
    dmem_req(w).bits.data  := 0.U
    dmem_req(w).bits.is_hella := false.B

    io.dmem.s1_kill(w) := false.B

    when (will_fire_load_incoming(w)) {
      dmem_req(w).valid      := !exe_tlb_miss(w) && !exe_tlb_uncacheable(w)
      dmem_req(w).bits.addr  := exe_tlb_paddr(w)
      dmem_req(w).bits.uop   := exe_tlb_uop(w)
			dmem_req(w).bits.uop.is_cap := false.B //yh+

      s0_executing_loads(ldq_incoming_idx(w)) := dmem_req_fire(w)
      assert(!ldq_incoming_e(w).bits.executed)

			//yh+begin
      when (dmem_req_fire(w)) {
        printf("[%d] Fire load incoming ldq(%d) addr: %x\n",
                io.core.tsc_reg, exe_tlb_uop(w).ldq_idx, exe_tlb_paddr(w))
      }
			//yh+end
    } .elsewhen (will_fire_load_retry(w)) {
      dmem_req(w).valid      := !exe_tlb_miss(w) && !exe_tlb_uncacheable(w)
      dmem_req(w).bits.addr  := exe_tlb_paddr(w)
      dmem_req(w).bits.uop   := exe_tlb_uop(w)
			dmem_req(w).bits.uop.is_cap := false.B //yh+

      s0_executing_loads(ldq_retry_idx) := dmem_req_fire(w)
      assert(!ldq_retry_e.bits.executed)

			//yh+begin
      when (dmem_req_fire(w)) {
        printf("[%d] Fire load retry ldq(%d) addr: %x\n",
                io.core.tsc_reg, exe_tlb_uop(w).ldq_idx, exe_tlb_paddr(w))
      }
			//yh+end
    } .elsewhen (will_fire_store_commit(w)) {
      dmem_req(w).valid         := true.B
      dmem_req(w).bits.addr     := stq_commit_e.bits.addr.bits
      dmem_req(w).bits.data     := (new freechips.rocketchip.rocket.StoreGen(
                                    stq_commit_e.bits.uop.mem_size, 0.U,
                                    stq_commit_e.bits.data.bits,
                                    coreDataBytes)).data
      dmem_req(w).bits.uop      := stq_commit_e.bits.uop
			dmem_req(w).bits.uop.is_cap := false.B //yh+

      stq_execute_head                     := Mux(dmem_req_fire(w),
                                                WrapInc(stq_execute_head, numStqEntries),
                                                stq_execute_head)

      stq(stq_execute_head).bits.succeeded := false.B

			//yh+begin
      when (dmem_req_fire(w)) {
        printf("[%d] Fire store commit stq(%d) addr: %x data: %x\n",
                io.core.tsc_reg, stq_commit_e.bits.uop.stq_idx,
								stq_commit_e.bits.addr.bits, stq_commit_e.bits.data.bits)
      }
			//yh+end
    } .elsewhen (will_fire_load_wakeup(w)) {
      dmem_req(w).valid      := true.B
      dmem_req(w).bits.addr  := ldq_wakeup_e.bits.addr.bits
      dmem_req(w).bits.uop   := ldq_wakeup_e.bits.uop
			dmem_req(w).bits.uop.is_cap := false.B //yh+

      s0_executing_loads(ldq_wakeup_idx) := dmem_req_fire(w)

      assert(!ldq_wakeup_e.bits.executed && !ldq_wakeup_e.bits.addr_is_virtual)

			//yh+begin
      when (dmem_req_fire(w)) {
        printf("[%d] Fire load wakeup ldq(%d) addr: %x\n",
                io.core.tsc_reg, ldq_wakeup_e.bits.uop.ldq_idx, ldq_wakeup_e.bits.addr.bits)
      }
			//yh+end
    } .elsewhen (will_fire_hella_incoming(w)) {
      assert(hella_state === h_s1)

      dmem_req(w).valid               := !io.hellacache.s1_kill && (!exe_tlb_miss(w) || hella_req.phys)
      dmem_req(w).bits.addr           := exe_tlb_paddr(w)
      dmem_req(w).bits.data           := (new freechips.rocketchip.rocket.StoreGen(
        hella_req.size, 0.U,
        io.hellacache.s1_data.data,
        coreDataBytes)).data
      dmem_req(w).bits.uop.mem_cmd    := hella_req.cmd
      dmem_req(w).bits.uop.mem_size   := hella_req.size
      dmem_req(w).bits.uop.mem_signed := hella_req.signed
      dmem_req(w).bits.is_hella       := true.B
			dmem_req(w).bits.uop.is_cap := false.B //yh+

      hella_paddr := exe_tlb_paddr(w)
    }
      .elsewhen (will_fire_hella_wakeup(w))
    {
      assert(hella_state === h_replay)
      dmem_req(w).valid               := true.B
      dmem_req(w).bits.addr           := hella_paddr
      dmem_req(w).bits.data           := (new freechips.rocketchip.rocket.StoreGen(
        hella_req.size, 0.U,
        hella_data.data,
        coreDataBytes)).data
      dmem_req(w).bits.uop.mem_cmd    := hella_req.cmd
      dmem_req(w).bits.uop.mem_size   := hella_req.size
      dmem_req(w).bits.uop.mem_signed := hella_req.signed
      dmem_req(w).bits.is_hella       := true.B
			dmem_req(w).bits.uop.is_cap := false.B //yh+
    }
		//yh+begin
      .elsewhen (will_fire_cap_check(w))
    {
      dmem_req(w).valid               := !exe_tlb_miss(w) && !exe_tlb_uncacheable(w)
      dmem_req(w).bits.addr           := exe_tlb_paddr(w)
      dmem_req(w).bits.uop            := exe_tlb_uop(w)

      when (dmem_req_fire(w)) {
				printf("[%d] Fire cap check! ", io.core.tsc_reg) 
	      when (can_fire_ldchk_p1(w)) {
          printf("slq(%d) vaddr: %x paddr: %x ", exe_tlb_uop(w).slq_idx, exe_tlb_vaddr(w), exe_tlb_paddr(w))
					assert(exe_tlb_uop(w).slq_idx === cap_check_idx(w))
        } .otherwise {
          printf("ssq(%d) vaddr: %x paddr: %x ", exe_tlb_uop(w).ssq_idx, exe_tlb_vaddr(w), exe_tlb_paddr(w))
					assert(exe_tlb_uop(w).ssq_idx === cap_check_idx(w))
        }
        printf("is_cap: %d uses_ldq: %d uses_stq: %d\n",
                exe_tlb_uop(w).is_cap, exe_tlb_uop(w).uses_ldq, exe_tlb_uop(w).uses_stq)

        assert(exe_tlb_uop(w).is_cap && exe_tlb_uop(w).uses_ldq && !exe_tlb_uop(w).uses_stq)
      }

      when (can_fire_ldchk_p1(w)) {
        slq(cap_check_idx(w)).bits.state := Mux(dmem_req_fire(w), s_wait,
                                         		slq(cap_check_idx(w)).bits.state)
      } .otherwise {
        ssq(cap_check_idx(w)).bits.state := Mux(dmem_req_fire(w), s_wait,
                                         		ssq(cap_check_idx(w)).bits.state)
      }
    }
      .elsewhen (will_fire_cap_store(w))
    {
      dmem_req(w).valid               := !exe_tlb_miss(w) && !exe_tlb_uncacheable(w)
      dmem_req(w).bits.addr           := exe_tlb_paddr(w)
      dmem_req(w).bits.uop            := exe_tlb_uop(w)
      dmem_req(w).bits.uop.uses_stq   := false.B // To prevent store dependency checks
      dmem_req(w).bits.data						:= (new freechips.rocketchip.rocket.StoreGen(
																					3.U, 0.U,
																					cap_store_data(w),
																					coreDataBytes)).data

			scq(cap_store_idx(w)).bits.state := Mux(dmem_req_fire(w),
																					Mux(scq(cap_store_idx(w)).bits.state === s_fire, s_wait, s_wait2),
																					scq(cap_store_idx(w)).bits.state)

      when (dmem_req_fire(w)) {
				printf("[%d] Fire cap store! scq(%d) vaddr: %x paddr: %x data: %x scq_head: %d mem_cmd: %d\n",
								io.core.tsc_reg, exe_tlb_uop(w).scq_idx, exe_tlb_vaddr(w),
								exe_tlb_paddr(w), cap_store_data(w), scq_head, exe_tlb_uop(w).mem_cmd)
      }

			assert(exe_tlb_uop(w).scq_idx === cap_store_idx(w))
      assert(exe_tlb_uop(w).is_cap && !exe_tlb_uop(w).uses_ldq && exe_tlb_uop(w).uses_stq)
		}
		//yh+end

    //-------------------------------------------------------------
    // Write Addr into the LAQ/SAQ
    when (will_fire_load_incoming(w) || will_fire_load_retry(w))
    {
      val ldq_idx = Mux(will_fire_load_incoming(w), ldq_incoming_idx(w), ldq_retry_idx)
      ldq(ldq_idx).bits.addr.valid          := true.B
      ldq(ldq_idx).bits.addr.bits           := Mux(exe_tlb_miss(w), exe_tlb_vaddr(w), exe_tlb_paddr(w))
      ldq(ldq_idx).bits.uop.pdst            := exe_tlb_uop(w).pdst
      ldq(ldq_idx).bits.addr_is_virtual     := exe_tlb_miss(w)
      ldq(ldq_idx).bits.addr_is_uncacheable := exe_tlb_uncacheable(w) && !exe_tlb_miss(w)

      assert(!(will_fire_load_incoming(w) && ldq_incoming_e(w).bits.addr.valid),
        "[lsu] Incoming load is overwriting a valid address")
    }

    when (will_fire_sta_incoming(w) || will_fire_stad_incoming(w) || will_fire_sta_retry(w))
    {
      val stq_idx = Mux(will_fire_sta_incoming(w) || will_fire_stad_incoming(w),
        stq_incoming_idx(w), stq_retry_idx)

      stq(stq_idx).bits.addr.valid := !pf_st(w) // Prevent AMOs from executing!
      stq(stq_idx).bits.addr.bits  := Mux(exe_tlb_miss(w), exe_tlb_vaddr(w), exe_tlb_paddr(w))
      stq(stq_idx).bits.uop.pdst   := exe_tlb_uop(w).pdst // Needed for AMOs
      stq(stq_idx).bits.addr_is_virtual := exe_tlb_miss(w)

      assert(!(will_fire_sta_incoming(w) && stq_incoming_e(w).bits.addr.valid),
        "[lsu] Incoming store is overwriting a valid address")

    }

    //-------------------------------------------------------------
    // Write data into the STQ
    if (w == 0)
      io.core.fp_stdata.ready := !will_fire_std_incoming(w) && !will_fire_stad_incoming(w)
    val fp_stdata_fire = io.core.fp_stdata.fire && (w == 0).B
    when (will_fire_std_incoming(w) || will_fire_stad_incoming(w) || fp_stdata_fire)
    {
      val sidx = Mux(will_fire_std_incoming(w) || will_fire_stad_incoming(w),
        stq_incoming_idx(w),
        io.core.fp_stdata.bits.uop.stq_idx)
      stq(sidx).bits.data.valid := true.B
      stq(sidx).bits.data.bits  := Mux(will_fire_std_incoming(w) || will_fire_stad_incoming(w),
        exe_req(w).bits.data,
        io.core.fp_stdata.bits.data)
      assert(!(stq(sidx).bits.data.valid),
        "[lsu] Incoming store is overwriting a valid data entry")
    }

		//yh+begin
    //-------------------------------------------------------------
    // Write Addr into the SLQ/SSQ
    when (cap_ldchk_incoming(w)) {
			val slq_idx = cap_exe_req(w).bits.uop.slq_idx

			slq(slq_idx).bits.tag				:= cap_exe_req(w).bits.tag
			slq(slq_idx).bits.tagged		:= cap_exe_req(w).bits.tagged
			slq(slq_idx).bits.addr			:= cap_exe_req(w).bits.addr
			//slq(slq_idx).bits.dir				:= cap_exe_req(w).bits.dir
			slq(slq_idx).bits.count 		:= (cap_exe_req(w).bits.num_ways - 1.U)
			slq(slq_idx).bits.state 		:= s_update
			slq(slq_idx).bits.cmt_addr	:= cap_exe_req(w).bits.cmt_addr
			slq(slq_idx).bits.num_ways	:= cap_exe_req(w).bits.num_ways

			//printf("[%d] Exe(ldchk) slq(%d) tag: %x tagged: %d addr: %x dir: %d state: %d mask: %x\n",
			printf("[%d] Exe(ldchk) slq(%d) tag: %x tagged: %d addr: %x state: %d mask: %x\n",
							io.core.tsc_reg, slq_idx, cap_exe_req(w).bits.tag, cap_exe_req(w).bits.tagged,
							//cap_exe_req(w).bits.addr, cap_exe_req(w).bits.dir, s_update, slq(slq_idx).bits.tag_dep_mask)
							cap_exe_req(w).bits.addr, s_update, slq(slq_idx).bits.tag_dep_mask)

			assert(slq(slq_idx).bits.state === s_init)
    }

    when (cap_stchk_incoming(w)) {
			val ssq_idx = cap_exe_req(w).bits.uop.ssq_idx

			ssq(ssq_idx).bits.tag				:= cap_exe_req(w).bits.tag
			ssq(ssq_idx).bits.tagged		:= cap_exe_req(w).bits.tagged
			ssq(ssq_idx).bits.addr			:= cap_exe_req(w).bits.addr
			//ssq(ssq_idx).bits.dir				:= cap_exe_req(w).bits.dir
			ssq(ssq_idx).bits.count 		:= (cap_exe_req(w).bits.num_ways - 1.U)
			ssq(ssq_idx).bits.state 		:= s_update
			ssq(ssq_idx).bits.cmt_addr	:= cap_exe_req(w).bits.cmt_addr
			ssq(ssq_idx).bits.num_ways	:= cap_exe_req(w).bits.num_ways

			//printf("[%d] Exe(stchk) ssq(%d) tag: %x tagged: %d addr: %x dir: %d state: %d mask: %x\n",
			printf("[%d] Exe(stchk) ssq(%d) tag: %x tagged: %d addr: %x state: %d mask: %x\n",
							io.core.tsc_reg, ssq_idx, cap_exe_req(w).bits.tag, cap_exe_req(w).bits.tagged,
							//cap_exe_req(w).bits.addr, cap_exe_req(w).bits.dir, s_update, ssq(ssq_idx).bits.tag_dep_mask)
							cap_exe_req(w).bits.addr, s_update, ssq(ssq_idx).bits.tag_dep_mask)

			assert(ssq(ssq_idx).bits.state === s_init)
    }

    when (cap_capsta_incoming(w)) {
			val scq_idx = cap_exe_req(w).bits.uop.scq_idx

			scq(scq_idx).bits.tag				:= cap_exe_req(w).bits.tag
			scq(scq_idx).bits.tagged		:= cap_exe_req(w).bits.tagged
			scq(scq_idx).bits.addr			:= cap_exe_req(w).bits.addr
			//scq(scq_idx).bits.dir				:= cap_exe_req(w).bits.dir
			scq(scq_idx).bits.count 		:= (cap_exe_req(w).bits.num_ways - 1.U)
			scq(scq_idx).bits.state 		:= s_update
			scq(scq_idx).bits.cmt_addr	:= cap_exe_req(w).bits.cmt_addr
			scq(scq_idx).bits.num_ways	:= cap_exe_req(w).bits.num_ways

			//printf("[%d] Exe(capst) scq(%d) cstr: %d cclr: %d tag: %x tagged: %d addr: %x dir: %d state: %d\n",
			printf("[%d] Exe(capst) scq(%d) cstr: %d cclr: %d tag: %x tagged: %d addr: %x state: %d\n",
							io.core.tsc_reg, scq_idx, cap_exe_req(w).bits.uop.cap_cmd === CAP_CS,
							cap_exe_req(w).bits.uop.cap_cmd === CAP_CC, cap_exe_req(w).bits.tag,
							//cap_exe_req(w).bits.tagged, cap_exe_req(w).bits.addr, cap_exe_req(w).bits.dir, s_update)
							cap_exe_req(w).bits.tagged, cap_exe_req(w).bits.addr, s_update)

			assert(scq(scq_idx).bits.state === s_init)
    }

		when (cap_ldchk_incoming(w) || cap_stchk_incoming(w) || cap_capsta_incoming(w)) {
			printf("[%d] Exe(*) cmt_addr: %x num_ways: %d\n",
							io.core.tsc_reg, cap_exe_req(w).bits.cmt_addr, cap_exe_req(w).bits.num_ways)
		}

    when (cap_capstd_incoming(w)) {
			val scq_idx = cap_exe_req(w).bits.uop.scq_idx

			scq(scq_idx).bits.data.valid	:= true.B
			scq(scq_idx).bits.data.bits		:= cap_exe_req(w).bits.data

      assert(!(scq(scq_idx).bits.data.valid),
        "[lsu] Incoming cap store is overwriting a valid data entry")

			printf("[%d] Data(capst) scq(%d) cstr: %d cclr: %d data: %x\n",
							io.core.tsc_reg, scq_idx, cap_exe_req(w).bits.uop.cap_cmd === CAP_CS,
							cap_exe_req(w).bits.uop.cap_cmd === CAP_CC, cap_exe_req(w).bits.data)
    }
		//yh+end
  }
  val will_fire_stdf_incoming = io.core.fp_stdata.fire
  require (xLen >= fLen) // for correct SDQ size

	//yh+begin
	val fired_dmem_req_ldst = widthMap(w => RegNext(dmem_req_fire(w) && 
																					!dmem_req(w).bits.uop.is_cap))
	val fired_dmem_req_bounds = widthMap(w => RegNext(dmem_req_fire(w) && 
																					dmem_req(w).bits.uop.is_cap))
	var temp_ldst_traffic = ldst_traffic
	var temp_bounds_traffic = bounds_traffic

	for (w <- 0 until memWidth) {
		temp_ldst_traffic = Mux(enableStat & fired_dmem_req_ldst(w),
														temp_ldst_traffic + 1.U, temp_ldst_traffic)
		temp_bounds_traffic = Mux(enableStat & fired_dmem_req_bounds(w),
															temp_bounds_traffic + 1.U, temp_bounds_traffic)
	}

	ldst_traffic := Mux(initDPT, io.core.dpt_csrs.ldst_traffic,
											temp_ldst_traffic)
	bounds_traffic := Mux(initDPT, io.core.dpt_csrs.bounds_traffic,
												temp_bounds_traffic)
	//yh+end

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Cache Access Cycle (Mem)
  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Note the DCache may not have accepted our request

  val exe_req_killed = widthMap(w => IsKilledByBranch(io.core.brupdate, exe_req(w).bits.uop))
  val stdf_killed = IsKilledByBranch(io.core.brupdate, io.core.fp_stdata.bits.uop)

  val fired_load_incoming  = widthMap(w => RegNext(will_fire_load_incoming(w) && !exe_req_killed(w)))
  val fired_stad_incoming  = widthMap(w => RegNext(will_fire_stad_incoming(w) && !exe_req_killed(w)))
  val fired_sta_incoming   = widthMap(w => RegNext(will_fire_sta_incoming (w) && !exe_req_killed(w)))
  val fired_std_incoming   = widthMap(w => RegNext(will_fire_std_incoming (w) && !exe_req_killed(w)))
  val fired_stdf_incoming  = RegNext(will_fire_stdf_incoming && !stdf_killed)
  val fired_sfence         = RegNext(will_fire_sfence)
  val fired_release        = RegNext(will_fire_release)
  val fired_load_retry     = widthMap(w => RegNext(will_fire_load_retry   (w) && !IsKilledByBranch(io.core.brupdate, ldq_retry_e.bits.uop)))
  val fired_sta_retry      = widthMap(w => RegNext(will_fire_sta_retry    (w) && !IsKilledByBranch(io.core.brupdate, stq_retry_e.bits.uop)))
  val fired_store_commit   = RegNext(will_fire_store_commit)
  val fired_load_wakeup    = widthMap(w => RegNext(will_fire_load_wakeup  (w) && !IsKilledByBranch(io.core.brupdate, ldq_wakeup_e.bits.uop)))
  val fired_hella_incoming = RegNext(will_fire_hella_incoming)
  val fired_hella_wakeup   = RegNext(will_fire_hella_wakeup)

  val mem_incoming_uop     = RegNext(widthMap(w => UpdateBrMask(io.core.brupdate, exe_req(w).bits.uop)))
  val mem_ldq_incoming_e   = RegNext(widthMap(w => UpdateBrMask(io.core.brupdate, ldq_incoming_e(w))))
  val mem_stq_incoming_e   = RegNext(widthMap(w => UpdateBrMask(io.core.brupdate, stq_incoming_e(w))))
  val mem_ldq_wakeup_e     = RegNext(UpdateBrMask(io.core.brupdate, ldq_wakeup_e))
  val mem_ldq_retry_e      = RegNext(UpdateBrMask(io.core.brupdate, ldq_retry_e))
  val mem_stq_retry_e      = RegNext(UpdateBrMask(io.core.brupdate, stq_retry_e))
  val mem_ldq_e            = widthMap(w =>
                             Mux(fired_load_incoming(w), mem_ldq_incoming_e(w),
                             Mux(fired_load_retry   (w), mem_ldq_retry_e,
                             Mux(fired_load_wakeup  (w), mem_ldq_wakeup_e, (0.U).asTypeOf(Valid(new LDQEntry))))))
  val mem_stq_e            = widthMap(w =>
                             Mux(fired_stad_incoming(w) ||
                                 fired_sta_incoming (w), mem_stq_incoming_e(w),
                             Mux(fired_sta_retry    (w), mem_stq_retry_e, (0.U).asTypeOf(Valid(new STQEntry)))))
  val mem_stdf_uop         = RegNext(UpdateBrMask(io.core.brupdate, io.core.fp_stdata.bits.uop))


  val mem_tlb_miss             = RegNext(exe_tlb_miss)
  val mem_tlb_uncacheable      = RegNext(exe_tlb_uncacheable)
  val mem_paddr                = RegNext(widthMap(w => dmem_req(w).bits.addr))

  // Task 1: Clr ROB busy bit
  val clr_bsy_valid   = RegInit(widthMap(w => false.B))
  val clr_bsy_rob_idx = Reg(Vec(memWidth, UInt(robAddrSz.W)))
  val clr_bsy_brmask  = Reg(Vec(memWidth, UInt(maxBrCount.W)))

  for (w <- 0 until memWidth) {
    clr_bsy_valid   (w) := false.B
    clr_bsy_rob_idx (w) := 0.U
    clr_bsy_brmask  (w) := 0.U


    when (fired_stad_incoming(w)) {
      clr_bsy_valid   (w) := mem_stq_incoming_e(w).valid           &&
                            !mem_tlb_miss(w)                       &&
                            !mem_stq_incoming_e(w).bits.uop.is_amo &&
                            !IsKilledByBranch(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      clr_bsy_rob_idx (w) := mem_stq_incoming_e(w).bits.uop.rob_idx
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
    } .elsewhen (fired_sta_incoming(w)) {
      clr_bsy_valid   (w) := mem_stq_incoming_e(w).valid            &&
                             mem_stq_incoming_e(w).bits.data.valid  &&
                            !mem_tlb_miss(w)                        &&
                            !mem_stq_incoming_e(w).bits.uop.is_amo  &&
                            !IsKilledByBranch(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      clr_bsy_rob_idx (w) := mem_stq_incoming_e(w).bits.uop.rob_idx
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
    } .elsewhen (fired_std_incoming(w)) {
      clr_bsy_valid   (w) := mem_stq_incoming_e(w).valid                 &&
                             mem_stq_incoming_e(w).bits.addr.valid       &&
                            !mem_stq_incoming_e(w).bits.addr_is_virtual  &&
                            !mem_stq_incoming_e(w).bits.uop.is_amo       &&
                            !IsKilledByBranch(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
      clr_bsy_rob_idx (w) := mem_stq_incoming_e(w).bits.uop.rob_idx
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_incoming_e(w).bits.uop)
    } .elsewhen (fired_sfence(w)) {
      clr_bsy_valid   (w) := (w == 0).B // SFence proceeds down all paths, only allow one to clr the rob
      clr_bsy_rob_idx (w) := mem_incoming_uop(w).rob_idx
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_incoming_uop(w))
    } .elsewhen (fired_sta_retry(w)) {
      clr_bsy_valid   (w) := mem_stq_retry_e.valid            &&
                             mem_stq_retry_e.bits.data.valid  &&
                            !mem_tlb_miss(w)                  &&
                            !mem_stq_retry_e.bits.uop.is_amo  &&
                            !IsKilledByBranch(io.core.brupdate, mem_stq_retry_e.bits.uop)
      clr_bsy_rob_idx (w) := mem_stq_retry_e.bits.uop.rob_idx
      clr_bsy_brmask  (w) := GetNewBrMask(io.core.brupdate, mem_stq_retry_e.bits.uop)
    }

    io.core.clr_bsy(w).valid := clr_bsy_valid(w) &&
                               !IsKilledByBranch(io.core.brupdate, clr_bsy_brmask(w)) &&
                               !io.core.exception && !RegNext(io.core.exception) && !RegNext(RegNext(io.core.exception))
    io.core.clr_bsy(w).bits  := clr_bsy_rob_idx(w)
  }

  val stdf_clr_bsy_valid   = RegInit(false.B)
  val stdf_clr_bsy_rob_idx = Reg(UInt(robAddrSz.W))
  val stdf_clr_bsy_brmask  = Reg(UInt(maxBrCount.W))
  stdf_clr_bsy_valid   := false.B
  stdf_clr_bsy_rob_idx := 0.U
  stdf_clr_bsy_brmask  := 0.U
  when (fired_stdf_incoming) {
    val s_idx = mem_stdf_uop.stq_idx
    stdf_clr_bsy_valid   := stq(s_idx).valid                 &&
                            stq(s_idx).bits.addr.valid       &&
                            !stq(s_idx).bits.addr_is_virtual &&
                            !stq(s_idx).bits.uop.is_amo      &&
                            !IsKilledByBranch(io.core.brupdate, mem_stdf_uop)
    stdf_clr_bsy_rob_idx := mem_stdf_uop.rob_idx
    stdf_clr_bsy_brmask  := GetNewBrMask(io.core.brupdate, mem_stdf_uop)
  }



  io.core.clr_bsy(memWidth).valid := stdf_clr_bsy_valid &&
                                    !IsKilledByBranch(io.core.brupdate, stdf_clr_bsy_brmask) &&
                                    !io.core.exception && !RegNext(io.core.exception) && !RegNext(RegNext(io.core.exception))
  io.core.clr_bsy(memWidth).bits  := stdf_clr_bsy_rob_idx



  // Task 2: Do LD-LD. ST-LD searches for ordering failures
  //         Do LD-ST search for forwarding opportunities
  // We have the opportunity to kill a request we sent last cycle. Use it wisely!

  // We translated a store last cycle
  val do_st_search = widthMap(w => (fired_stad_incoming(w) || fired_sta_incoming(w) || fired_sta_retry(w)) && !mem_tlb_miss(w))
  // We translated a load last cycle
  val do_ld_search = widthMap(w => ((fired_load_incoming(w) || fired_load_retry(w)) && !mem_tlb_miss(w)) ||
                     fired_load_wakeup(w))
  // We are making a local line visible to other harts
  val do_release_search = widthMap(w => fired_release(w))

  // Store addrs don't go to memory yet, get it from the TLB response
  // Load wakeups don't go through TLB, get it through memory
  // Load incoming and load retries go through both

  val lcam_addr  = widthMap(w => Mux(fired_stad_incoming(w) || fired_sta_incoming(w) || fired_sta_retry(w),
                                     RegNext(exe_tlb_paddr(w)),
                                     Mux(fired_release(w), RegNext(io.dmem.release.bits.address),
                                         mem_paddr(w))))
  val lcam_uop   = widthMap(w => Mux(do_st_search(w), mem_stq_e(w).bits.uop,
                                 Mux(do_ld_search(w), mem_ldq_e(w).bits.uop, NullMicroOp)))

  val lcam_mask  = widthMap(w => GenByteMask(lcam_addr(w), lcam_uop(w).mem_size))
  val lcam_st_dep_mask = widthMap(w => mem_ldq_e(w).bits.st_dep_mask)
  val lcam_is_release = widthMap(w => fired_release(w))
  val lcam_ldq_idx  = widthMap(w =>
                      Mux(fired_load_incoming(w), mem_incoming_uop(w).ldq_idx,
                      Mux(fired_load_wakeup  (w), RegNext(ldq_wakeup_idx),
                      Mux(fired_load_retry   (w), RegNext(ldq_retry_idx), 0.U))))
  val lcam_stq_idx  = widthMap(w =>
                      Mux(fired_stad_incoming(w) ||
                          fired_sta_incoming (w), mem_incoming_uop(w).stq_idx,
                      Mux(fired_sta_retry    (w), RegNext(stq_retry_idx), 0.U)))

  val can_forward = WireInit(widthMap(w =>
    Mux(fired_load_incoming(w) || fired_load_retry(w), !mem_tlb_uncacheable(w),
      !ldq(lcam_ldq_idx(w)).bits.addr_is_uncacheable)))

  // Mask of stores which we conflict on address with
  val ldst_addr_matches    = WireInit(widthMap(w => VecInit((0 until numStqEntries).map(x=>false.B))))
  // Mask of stores which we can forward from
  val ldst_forward_matches = WireInit(widthMap(w => VecInit((0 until numStqEntries).map(x=>false.B))))

  val failed_loads     = WireInit(VecInit((0 until numLdqEntries).map(x=>false.B))) // Loads which we will report as failures (throws a mini-exception)
  val nacking_loads    = WireInit(VecInit((0 until numLdqEntries).map(x=>false.B))) // Loads which are being nacked by dcache in the next stage

  val s1_executing_loads = RegNext(s0_executing_loads)
  val s1_set_execute     = WireInit(s1_executing_loads)

  val mem_forward_valid   = Wire(Vec(memWidth, Bool()))
  val mem_forward_ldq_idx = lcam_ldq_idx
  val mem_forward_ld_addr = lcam_addr
  val mem_forward_stq_idx = Wire(Vec(memWidth, UInt(log2Ceil(numStqEntries).W)))

  val wb_forward_valid    = RegNext(mem_forward_valid)
  val wb_forward_ldq_idx  = RegNext(mem_forward_ldq_idx)
  val wb_forward_ld_addr  = RegNext(mem_forward_ld_addr)
  val wb_forward_stq_idx  = RegNext(mem_forward_stq_idx)

  for (i <- 0 until numLdqEntries) {
    val l_valid = ldq(i).valid
    val l_bits  = ldq(i).bits
    val l_addr  = ldq(i).bits.addr.bits
    val l_mask  = GenByteMask(l_addr, l_bits.uop.mem_size)

    val l_forwarders      = widthMap(w => wb_forward_valid(w) && wb_forward_ldq_idx(w) === i.U)
    val l_is_forwarding   = l_forwarders.reduce(_||_)
    val l_forward_stq_idx = Mux(l_is_forwarding, Mux1H(l_forwarders, wb_forward_stq_idx), l_bits.forward_stq_idx)


    val block_addr_matches = widthMap(w => lcam_addr(w) >> blockOffBits === l_addr >> blockOffBits)
    val dword_addr_matches = widthMap(w => block_addr_matches(w) && lcam_addr(w)(blockOffBits-1,3) === l_addr(blockOffBits-1,3))
    val mask_match   = widthMap(w => (l_mask & lcam_mask(w)) === l_mask)
    val mask_overlap = widthMap(w => (l_mask & lcam_mask(w)).orR)

    // Searcher is a store
    for (w <- 0 until memWidth) {

      when (do_release_search(w) &&
            l_valid              &&
            l_bits.addr.valid    &&
            block_addr_matches(w)) {
        // This load has been observed, so if a younger load to the same address has not
        // executed yet, this load must be squashed
        ldq(i).bits.observed := true.B
      } .elsewhen (do_st_search(w)                                                                                                &&
                   l_valid                                                                                                        &&
                   l_bits.addr.valid                                                                                              &&
                   (l_bits.executed || l_bits.succeeded || l_is_forwarding)                                                       &&
                   !l_bits.addr_is_virtual                                                                                        &&
                   l_bits.st_dep_mask(lcam_stq_idx(w))                                                                            &&
                   dword_addr_matches(w)                                                                                          &&
                   mask_overlap(w)) {

        val forwarded_is_older = IsOlder(l_forward_stq_idx, lcam_stq_idx(w), l_bits.youngest_stq_idx)
        // We are older than this load, which overlapped us.
        when (!l_bits.forward_std_val || // If the load wasn't forwarded, it definitely failed
          ((l_forward_stq_idx =/= lcam_stq_idx(w)) && forwarded_is_older)) { // If the load forwarded from us, we might be ok
          ldq(i).bits.order_fail := true.B
          failed_loads(i)        := true.B
        }
      } .elsewhen (do_ld_search(w)            &&
                   l_valid                    &&
                   l_bits.addr.valid          &&
                   !l_bits.addr_is_virtual    &&
                   dword_addr_matches(w)      &&
                   mask_overlap(w)) {
        val searcher_is_older = IsOlder(lcam_ldq_idx(w), i.U, ldq_head)
        when (searcher_is_older) {
          when ((l_bits.executed || l_bits.succeeded || l_is_forwarding) &&
                !s1_executing_loads(i) && // If the load is proceeding in parallel we don't need to kill it
                l_bits.observed) {        // Its only a ordering failure if the cache line was observed between the younger load and us
            ldq(i).bits.order_fail := true.B
            failed_loads(i)        := true.B
          }
        } .elsewhen (lcam_ldq_idx(w) =/= i.U) {
          // The load is older, and either it hasn't executed, it was nacked, or it is ignoring its response
          // we need to kill ourselves, and prevent forwarding
          val older_nacked = nacking_loads(i) || RegNext(nacking_loads(i))
          when (!(l_bits.executed || l_bits.succeeded) || older_nacked) {
            s1_set_execute(lcam_ldq_idx(w))    := false.B
            io.dmem.s1_kill(w)                 := RegNext(dmem_req_fire(w))
            can_forward(w)                     := false.B
          }
        }
      }
    }
  }

  for (i <- 0 until numStqEntries) {
    val s_addr = stq(i).bits.addr.bits
    val s_uop  = stq(i).bits.uop
    val dword_addr_matches = widthMap(w =>
                             ( stq(i).bits.addr.valid      &&
                              !stq(i).bits.addr_is_virtual &&
                              (s_addr(corePAddrBits-1,3) === lcam_addr(w)(corePAddrBits-1,3))))
    val write_mask = GenByteMask(s_addr, s_uop.mem_size)
    for (w <- 0 until memWidth) {
      when (do_ld_search(w) && stq(i).valid && lcam_st_dep_mask(w)(i)) {
        when (((lcam_mask(w) & write_mask) === lcam_mask(w)) && !s_uop.is_fence && dword_addr_matches(w) && can_forward(w))
        {
          ldst_addr_matches(w)(i)            := true.B
          ldst_forward_matches(w)(i)         := true.B
          io.dmem.s1_kill(w)                 := RegNext(dmem_req_fire(w))
          s1_set_execute(lcam_ldq_idx(w))    := false.B
        }
          .elsewhen (((lcam_mask(w) & write_mask) =/= 0.U) && dword_addr_matches(w))
        {
          ldst_addr_matches(w)(i)            := true.B
          io.dmem.s1_kill(w)                 := RegNext(dmem_req_fire(w))
          s1_set_execute(lcam_ldq_idx(w))    := false.B
        }
          .elsewhen (s_uop.is_fence || s_uop.is_amo)
        {
          ldst_addr_matches(w)(i)            := true.B
          io.dmem.s1_kill(w)                 := RegNext(dmem_req_fire(w))
          s1_set_execute(lcam_ldq_idx(w))    := false.B
        }
      }
    }
  }

  // Set execute bit in LDQ
  for (i <- 0 until numLdqEntries) {
    when (s1_set_execute(i)) { ldq(i).bits.executed := true.B }
  }

  // Find the youngest store which the load is dependent on
  val forwarding_age_logic = Seq.fill(memWidth) { Module(new ForwardingAgeLogic(numStqEntries)) }
  for (w <- 0 until memWidth) {
    forwarding_age_logic(w).io.addr_matches    := ldst_addr_matches(w).asUInt
    forwarding_age_logic(w).io.youngest_st_idx := lcam_uop(w).stq_idx
  }
  val forwarding_idx = widthMap(w => forwarding_age_logic(w).io.forwarding_idx)

  // Forward if st-ld forwarding is possible from the writemask and loadmask
  mem_forward_valid       := widthMap(w =>
                                  (ldst_forward_matches(w)(forwarding_idx(w))        &&
                                 !IsKilledByBranch(io.core.brupdate, lcam_uop(w))    &&
                                 !io.core.exception && !RegNext(io.core.exception)))
  mem_forward_stq_idx     := forwarding_idx

  // Avoid deadlock with a 1-w LSU prioritizing load wakeups > store commits
  // On a 2W machine, load wakeups and store commits occupy separate pipelines,
  // so only add this logic for 1-w LSU
  if (memWidth == 1) {
    // Wakeups may repeatedly find a st->ld addr conflict and fail to forward,
    // repeated wakeups may block the store from ever committing
    // Disallow load wakeups 1 cycle after this happens to allow the stores to drain
    when (RegNext(ldst_addr_matches(0).reduce(_||_) && !mem_forward_valid(0))) {
      block_load_wakeup := true.B
    }

    // If stores remain blocked for 15 cycles, block load wakeups to get a store through
    val store_blocked_counter = Reg(UInt(4.W))
    when (will_fire_store_commit(0) || !can_fire_store_commit(0)) {
      store_blocked_counter := 0.U
    } .elsewhen (can_fire_store_commit(0) && !will_fire_store_commit(0)) {
      store_blocked_counter := Mux(store_blocked_counter === 15.U, store_blocked_counter + 1.U, 15.U)
    }
    when (store_blocked_counter === 15.U) {
      block_load_wakeup := true.B
    }
  }


  // Task 3: Clr unsafe bit in ROB for succesful translations
  //         Delay this a cycle to avoid going ahead of the exception broadcast
  //         The unsafe bit is cleared on the first translation, so no need to fire for load wakeups
  for (w <- 0 until memWidth) {
    io.core.clr_unsafe(w).valid := RegNext((do_st_search(w) || do_ld_search(w)) && !fired_load_wakeup(w)) && false.B
    io.core.clr_unsafe(w).bits  := RegNext(lcam_uop(w).rob_idx)
  }

  // detect which loads get marked as failures, but broadcast to the ROB the oldest failing load
  // TODO encapsulate this in an age-based  priority-encoder
  //   val l_idx = AgePriorityEncoder((Vec(Vec.tabulate(numLdqEntries)(i => failed_loads(i) && i.U >= laq_head)
  //   ++ failed_loads)).asUInt)
  val temp_bits = (VecInit(VecInit.tabulate(numLdqEntries)(i =>
    failed_loads(i) && i.U >= ldq_head) ++ failed_loads)).asUInt
  val l_idx = PriorityEncoder(temp_bits)

  // one exception port, but multiple causes!
  // - 1) the incoming store-address finds a faulting load (it is by definition younger)
  // - 2) the incoming load or store address is excepting. It must be older and thus takes precedent.
  val r_xcpt_valid = RegInit(false.B)
  val r_xcpt       = Reg(new Exception)

  val ld_xcpt_valid = failed_loads.reduce(_|_)
  val ld_xcpt_uop   = ldq(Mux(l_idx >= numLdqEntries.U, l_idx - numLdqEntries.U, l_idx)).bits.uop

  //yh-val use_mem_xcpt = (mem_xcpt_valid && IsOlder(mem_xcpt_uop.rob_idx, ld_xcpt_uop.rob_idx, io.core.rob_head_idx)) || !ld_xcpt_valid

  //yh-val xcpt_uop = Mux(use_mem_xcpt, mem_xcpt_uop, ld_xcpt_uop)

  //yh-r_xcpt_valid := (ld_xcpt_valid || mem_xcpt_valid) &&
  //yh-                 !io.core.exception &&
  //yh-                 !IsKilledByBranch(io.core.brupdate, xcpt_uop)

	//yh+begin
  //val use_mem_xcpt = (mem_xcpt_valid && IsOlder(mem_xcpt_uop.rob_idx, ld_xcpt_uop.rob_idx, io.core.rob_head_idx)) || !ld_xcpt_valid
  val use_mem_xcpt = mem_xcpt_valid && (IsOlder(mem_xcpt_uop.rob_idx, ld_xcpt_uop.rob_idx, io.core.rob_head_idx) || !ld_xcpt_valid)
  val use_ld_xcpt = ld_xcpt_valid
  //val use_mem_xcpt = ((mem_xcpt_valid && IsOlder(mem_xcpt_uop.rob_idx, ld_xcpt_uop.rob_idx, io.core.rob_head_idx) &&
	//											IsOlder(mem_xcpt_uop.rob_idx, dpt_xcpt_uop.rob_idx, io.core.rob_head_idx)) || !(ld_xcpt_valid || dpt_xcpt_valid))
  //val use_ld_xcpt = (ld_xcpt_valid && IsOlder(ld_xcpt_uop.rob_idx, dpt_xcpt_uop.rob_idx, io.core.rob_head_idx)) || !dpt_xcpt_valid
	val xcpt_uop = Mux(use_mem_xcpt, mem_xcpt_uop,
											Mux(use_ld_xcpt, ld_xcpt_uop, dpt_xcpt_uop))

  r_xcpt_valid := ((ld_xcpt_valid || mem_xcpt_valid || dpt_xcpt_valid) &&
                   !io.core.exception &&
                   !IsKilledByBranch(io.core.brupdate, xcpt_uop))
	//yh+end
  r_xcpt.uop         := xcpt_uop
  r_xcpt.uop.br_mask := GetNewBrMask(io.core.brupdate, xcpt_uop)
  //yh-r_xcpt.cause       := Mux(use_mem_xcpt, mem_xcpt_cause, MINI_EXCEPTION_MEM_ORDERING)
  //yh-r_xcpt.badvaddr    := mem_xcpt_vaddr // TODO is there another register we can use instead?
	//yh+begin
  r_xcpt.cause       := Mux(use_mem_xcpt, mem_xcpt_cause, 
														Mux(use_ld_xcpt, MINI_EXCEPTION_MEM_ORDERING, dpt_xcpt_cause))
  r_xcpt.badvaddr    := Mux(use_mem_xcpt || use_ld_xcpt, mem_xcpt_vaddr, dpt_xcpt_addr)

	when (r_xcpt_valid) {
		printf("[%d] Found r_xcpt_valid! rob(%d) badvaddr: %x use_mem_xcpt: %d use_ld_xcpt: %d dpt_xcpt_valid: %d\n",
						io.core.tsc_reg, r_xcpt.uop.rob_idx, r_xcpt.badvaddr, use_mem_xcpt, use_ld_xcpt, dpt_xcpt_valid)
	}
	//yh+end

  io.core.lxcpt.valid := r_xcpt_valid && !io.core.exception && !IsKilledByBranch(io.core.brupdate, r_xcpt.uop)
  io.core.lxcpt.bits  := r_xcpt

  // Task 4: Speculatively wakeup loads 1 cycle before they come back
  for (w <- 0 until memWidth) {
    io.core.spec_ld_wakeup(w).valid := enableFastLoadUse.B          &&
                                       fired_load_incoming(w)       &&
                                       !mem_incoming_uop(w).fp_val  &&
                                       mem_incoming_uop(w).pdst =/= 0.U
    io.core.spec_ld_wakeup(w).bits  := mem_incoming_uop(w).pdst
  }


  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // Writeback Cycle (St->Ld Forwarding Path)
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Handle Memory Responses and nacks
  //----------------------------------
  for (w <- 0 until memWidth) {
    io.core.exe(w).iresp.valid := false.B
    io.core.exe(w).fresp.valid := false.B
  }

  val dmem_resp_fired = WireInit(widthMap(w => false.B))

  for (w <- 0 until memWidth) {
    // Handle nacks
    when (io.dmem.nack(w).valid)
    {
      // We have to re-execute this!
      when (io.dmem.nack(w).bits.is_hella)
      {
        assert(hella_state === h_wait || hella_state === h_dead)
      }
        .elsewhen (io.dmem.nack(w).bits.uop.uses_ldq)
      {
        assert(ldq(io.dmem.nack(w).bits.uop.ldq_idx).bits.executed)
        ldq(io.dmem.nack(w).bits.uop.ldq_idx).bits.executed  := false.B
        nacking_loads(io.dmem.nack(w).bits.uop.ldq_idx) := true.B

				//yh+begin
				printf("[%d] Received LD-NACK ldq(%d) slq(%d)\n",
								io.core.tsc_reg, io.dmem.nack(w).bits.uop.ldq_idx, io.dmem.nack(w).bits.uop.slq_idx)
				//yh+end
      }
        .otherwise
        //.elsewhen (io.dmem.nack(w).bits.uop.uses_stq) //yh+
      {
        assert(io.dmem.nack(w).bits.uop.uses_stq)
        when (IsOlder(io.dmem.nack(w).bits.uop.stq_idx, stq_execute_head, stq_head)) {
          stq_execute_head := io.dmem.nack(w).bits.uop.stq_idx
        }

				//yh+begin
				printf("[%d] Received ST-NACK stq(%d) ssq(%d)\n",
								io.core.tsc_reg, io.dmem.nack(w).bits.uop.stq_idx, io.dmem.nack(w).bits.uop.ssq_idx)
				//yh+end
      }
    }
    // Handle the response
    when (io.dmem.resp(w).valid)
    {
      when (io.dmem.resp(w).bits.uop.uses_ldq)
      {
        assert(!io.dmem.resp(w).bits.is_hella)
        val ldq_idx = io.dmem.resp(w).bits.uop.ldq_idx
        val send_iresp = ldq(ldq_idx).bits.uop.dst_rtype === RT_FIX
        val send_fresp = ldq(ldq_idx).bits.uop.dst_rtype === RT_FLT

        io.core.exe(w).iresp.bits.uop  := ldq(ldq_idx).bits.uop
        io.core.exe(w).fresp.bits.uop  := ldq(ldq_idx).bits.uop
        io.core.exe(w).iresp.valid     := send_iresp
        io.core.exe(w).iresp.bits.data := io.dmem.resp(w).bits.data
        io.core.exe(w).fresp.valid     := send_fresp
        io.core.exe(w).fresp.bits.data := io.dmem.resp(w).bits.data

        assert(send_iresp ^ send_fresp)
        dmem_resp_fired(w) := true.B

        ldq(ldq_idx).bits.succeeded      := io.core.exe(w).iresp.valid || io.core.exe(w).fresp.valid
        ldq(ldq_idx).bits.debug_wb_data  := io.dmem.resp(w).bits.data
				//yh+begin
				assert(!io.dmem.resp(w).bits.uop.is_cap)
				printf("[%d] Received LD-RESP ldq(%d) slq(%d)\n",
								io.core.tsc_reg, io.dmem.resp(w).bits.uop.ldq_idx, io.dmem.resp(w).bits.uop.slq_idx)
				//yh+end
      }
        .elsewhen (io.dmem.resp(w).bits.uop.uses_stq)
      {
        assert(!io.dmem.resp(w).bits.is_hella)
        stq(io.dmem.resp(w).bits.uop.stq_idx).bits.succeeded := true.B
        when (io.dmem.resp(w).bits.uop.is_amo) {
          dmem_resp_fired(w) := true.B
          io.core.exe(w).iresp.valid     := true.B
          io.core.exe(w).iresp.bits.uop  := stq(io.dmem.resp(w).bits.uop.stq_idx).bits.uop
          io.core.exe(w).iresp.bits.data := io.dmem.resp(w).bits.data

          stq(io.dmem.resp(w).bits.uop.stq_idx).bits.debug_wb_data := io.dmem.resp(w).bits.data
        }
				//yh+begin
				assert(!io.dmem.resp(w).bits.uop.is_cap)
				printf("[%d] Received ST-RESP stq(%d) ssq(%d)\n",
								io.core.tsc_reg, io.dmem.resp(w).bits.uop.stq_idx, io.dmem.resp(w).bits.uop.ssq_idx)
				//yh+end
      }
    }


    when (dmem_resp_fired(w) && wb_forward_valid(w))
    {
      // Twiddle thumbs. Can't forward because dcache response takes precedence
    }
      .elsewhen (!dmem_resp_fired(w) && wb_forward_valid(w))
    {
      val f_idx       = wb_forward_ldq_idx(w)
      val forward_uop = ldq(f_idx).bits.uop
      val stq_e       = stq(wb_forward_stq_idx(w))
      val data_ready  = stq_e.bits.data.valid
      val live        = !IsKilledByBranch(io.core.brupdate, forward_uop)
      val storegen = new freechips.rocketchip.rocket.StoreGen(
                                stq_e.bits.uop.mem_size, stq_e.bits.addr.bits,
                                stq_e.bits.data.bits, coreDataBytes)
      val loadgen  = new freechips.rocketchip.rocket.LoadGen(
                                forward_uop.mem_size, forward_uop.mem_signed,
                                wb_forward_ld_addr(w),
                                storegen.data, false.B, coreDataBytes)

      io.core.exe(w).iresp.valid := (forward_uop.dst_rtype === RT_FIX) && data_ready && live
      io.core.exe(w).fresp.valid := (forward_uop.dst_rtype === RT_FLT) && data_ready && live
      io.core.exe(w).iresp.bits.uop  := forward_uop
      io.core.exe(w).fresp.bits.uop  := forward_uop
      io.core.exe(w).iresp.bits.data := loadgen.data
      io.core.exe(w).fresp.bits.data := loadgen.data

      when (data_ready && live) {
        ldq(f_idx).bits.succeeded := data_ready
        ldq(f_idx).bits.forward_std_val := true.B
        ldq(f_idx).bits.forward_stq_idx := wb_forward_stq_idx(w)

        ldq(f_idx).bits.debug_wb_data   := loadgen.data
      }
			//yh+begin
			printf("[%d] Forward load ldq(%d)\n", io.core.tsc_reg, f_idx)
			//yh+end
    }

		//yh+begin
    // Handle capability nacks
    when (io.dmem.cap_nack(w).valid) {
			val cap_nack_uop = io.dmem.cap_nack(w).bits.uop

      when (cap_nack_uop.cap_cmd === CAP_LD) {
				val slq_idx = cap_nack_uop.slq_idx
        assert(slq(slq_idx).bits.state === s_wait)
        slq(slq_idx).bits.state := s_fire

        printf("[%d] Received CAP_LD-NACK! slq(%d)\n",
                io.core.tsc_reg, io.dmem.cap_nack(w).bits.uop.slq_idx)
      } .elsewhen (cap_nack_uop.cap_cmd === CAP_ST) {
				val ssq_idx = cap_nack_uop.ssq_idx
        assert(ssq(ssq_idx).bits.state === s_wait)
        ssq(ssq_idx).bits.state := s_fire

        printf("[%d] Received CAP_ST-NACK! ssq(%d)\n",
                io.core.tsc_reg, io.dmem.cap_nack(w).bits.uop.ssq_idx)
			} .otherwise {
				val scq_idx = cap_nack_uop.scq_idx
        scq(scq_idx).bits.state := Mux(scq(scq_idx).bits.state === s_wait, s_fire, s_fire2)

        printf("[%d] Received CAP_C*-NACK! scq(%d)\n",
                io.core.tsc_reg, io.dmem.cap_nack(w).bits.uop.scq_idx)
        assert(scq(scq_idx).bits.state === s_wait || scq(scq_idx).bits.state === s_wait2)
			}
		}

    // Handle the capability response
    when (io.dmem.cap_resp(w).valid) {
      val slq_idx = io.dmem.cap_resp(w).bits.uop.slq_idx
      val ssq_idx = io.dmem.cap_resp(w).bits.uop.ssq_idx
      val scq_idx = io.dmem.cap_resp(w).bits.uop.scq_idx
			val is_ldst = !io.dmem.cap_resp(w).bits.uop.cap_cmd(1)
			val is_load = (io.dmem.cap_resp(w).bits.uop.cap_cmd === CAP_LD)
			val is_store = (io.dmem.cap_resp(w).bits.uop.cap_cmd === CAP_ST)
			val is_cstr = (io.dmem.cap_resp(w).bits.uop.cap_cmd === CAP_CS)
			val is_cclr = (io.dmem.cap_resp(w).bits.uop.cap_cmd === CAP_CC)

			val addr = Mux(is_load, slq(slq_idx).bits.addr, ssq(ssq_idx).bits.addr)
			val resp_lbnd = (io.dmem.resp(w).bits.dataBeats(vaddrBits-1,0) & bounds_margin)
			//val resp_ubnd = Mux(is_load, io.dmem.resp(w).bits.dataBeats(xLen+vaddrBits-1,xLen) + 1.U,
      //                    io.dmem.resp(w).bits.dataBeats(xLen+vaddrBits-1,xLen))
			val resp_ubnd = (io.dmem.resp(w).bits.dataBeats(xLen+vaddrBits-1,xLen) & bounds_margin)
			val resp_data = io.dmem.resp(w).bits.data(vaddrBits-1,0)

			// Perform capability checks
			val cap_chk = (is_ldst && (resp_lbnd(vaddrBits-1,0) <= addr(vaddrBits-1,0) &&
																	addr(vaddrBits-1,0) <= resp_ubnd(vaddrBits-1,0)))
																	//addr(vaddrBits-1,0) < resp_ubnd(vaddrBits-1,0)))
			val occ_chk = ((is_cstr && resp_data(vaddrBits-1,0) === 0.U) ||
											(is_cclr && resp_data(vaddrBits-1,0) === scq(scq_idx).bits.addr(vaddrBits-1,0)) ||
											  (scq(scq_idx).bits.state === s_wait2))
			val pass = (cap_chk | occ_chk)
			val fail = (!pass &&
									((is_load && slq(slq_idx).bits.count === 0.U) |
									(is_store && ssq(ssq_idx).bits.count === 0.U) |
									(!is_ldst && scq(scq_idx).bits.count === 0.U)))
			val decr = (!pass && !fail)

			printf("[%d] is_load: %d is_store: %d is_cstr: %d is_cclr: %d\n", io.core.tsc_reg, is_load, is_store, is_cstr, is_cclr)

			val next_state = Mux(cap_chk, s_done,
												Mux(occ_chk, Mux(scq(scq_idx).bits.state === s_wait, s_fire2, s_done),
												Mux(decr, s_fire, Mux(!is_cstr && suppressFault, s_done, s_fail))))
												//Mux(decr, s_fire, s_fail)))

			when (cap_chk | occ_chk) { printf("[%d] Passed ", io.core.tsc_reg) }
			.elsewhen (fail) { printf("[%d] Failed ", io.core.tsc_reg) }
			.otherwise { printf("[%d] Decrement Count ", io.core.tsc_reg) }

      when (is_load) {
				slq(slq_idx).bits.state := next_state
				// TODO just use WrapDecWay?
				//TODO slq(slq_idx).bits.way := Mux(decr, WrapIncOrDecWay(slq(slq_idx).bits.dir, slq(slq_idx).bits.way, num_ways),
				//TODO 															slq(slq_idx).bits.way)
				slq(slq_idx).bits.way := Mux(decr, WrapIncWay(slq(slq_idx).bits.way, slq(slq_idx).bits.num_ways), slq(slq_idx).bits.way)
				slq(slq_idx).bits.count := Mux(decr, slq(slq_idx).bits.count - 1.U, slq(slq_idx).bits.count)
				
				printf("slq(%d) addr: %x resp_lbnd: %x resp_ubnd: %x next_state: %d\n", slq_idx, addr, resp_lbnd, resp_ubnd, next_state)
				assert(slq(slq_idx).bits.state === s_wait)
      } .elsewhen (is_store) {
				ssq(ssq_idx).bits.state := next_state
				// TODO just use WrapDecWay?
				//TODO ssq(ssq_idx).bits.way := Mux(decr, WrapIncOrDecWay(ssq(ssq_idx).bits.dir, ssq(ssq_idx).bits.way, num_ways),
				//TODO 															ssq(ssq_idx).bits.way)
				ssq(ssq_idx).bits.way := Mux(decr, WrapIncWay(ssq(ssq_idx).bits.way, ssq(ssq_idx).bits.num_ways), ssq(ssq_idx).bits.way)
				ssq(ssq_idx).bits.count := Mux(decr, ssq(ssq_idx).bits.count - 1.U, ssq(ssq_idx).bits.count)

				printf("ssq(%d) addr: %x resp_lbnd: %x resp_ubnd: %x next_state: %d\n", ssq_idx, addr, resp_lbnd, resp_ubnd, next_state)
				assert(ssq(ssq_idx).bits.state === s_wait)
			} .otherwise {
				scq(scq_idx).bits.state := next_state
				//TODO scq(scq_idx).bits.way := Mux(decr, WrapIncOrDecWay(scq(scq_idx).bits.dir, scq(scq_idx).bits.way, num_ways),
				//TODO 															scq(scq_idx).bits.way)
				scq(scq_idx).bits.way := Mux(decr, WrapIncWay(scq(scq_idx).bits.way, scq(scq_idx).bits.num_ways), scq(scq_idx).bits.way)
				scq(scq_idx).bits.count := Mux(decr, scq(scq_idx).bits.count - 1.U, scq(scq_idx).bits.count)

				printf("scq(%d) addr: %x resp_data: %x next_state: %d\n",
								scq_idx, addr, resp_data, next_state)
				assert(scq(scq_idx).bits.state === s_wait || scq(scq_idx).bits.state === s_wait2)
			}

			// Update ccache
			ccache_write_lbnd_val(w) := (cap_chk | (occ_chk & scq(scq_idx).bits.state === s_wait))
			ccache_write_ubnd_val(w) := (cap_chk | (occ_chk & scq(scq_idx).bits.state === s_wait2))
			ccache_write_tag(w) := Mux(is_load, slq(slq_idx).bits.tag,
															Mux(is_store, ssq(ssq_idx).bits.tag, scq(scq_idx).bits.tag))
			ccache_write_lbnd_data(w) := Mux(occ_chk, Mux(is_cclr, 0.U, scq(scq_idx).bits.addr(vaddrBits-1,0)), resp_lbnd)
			ccache_write_ubnd_data(w) := Mux(occ_chk, Mux(is_cclr, 0.U,
                                      scq(scq_idx).bits.addr(vaddrBits-1,0) + scq(scq_idx).bits.data.bits(vaddrBits-1,0)), resp_ubnd)

			// Clear needCC
			//clr_needCC_valid(memWidth+w)		:= (pass || (fail && is_cclr))
			clr_needCC_valid(memWidth+w)		:= (next_state === s_done)
			clr_needCC_rob_idx(memWidth+w)	:= io.dmem.cap_resp(w).bits.uop.rob_idx
			clr_needCC_brmask(memWidth+w)		:= GetNewBrMask(io.core.brupdate, io.dmem.cap_resp(w).bits.uop)

			//when (!decr && !(fail && is_cstr)) {
			when (next_state === s_done) {
				printf("[%d] Clear needCC(2) rob(%d)\n", io.core.tsc_reg, io.dmem.cap_resp(w).bits.uop.rob_idx)
			}

			//update_shb_valid(w) := (is_cstr & next_state === s_done)
			update_shb_valid(w) := ((is_cstr || is_cclr) && next_state === s_done)
			//update_chb_valid(w) := (is_cclr & next_state === s_done)
			//update_hb_tag(w)		:= scq(scq_idx).bits.tag
			update_hb_tag(w)		:= scq(scq_idx).bits.tag(tagWidth/2-1,0)
			//update_hb_way(w)		:= WrapIncWay(scq(scq_idx).bits.way, scq(scq_idx).bits.num_ways)
			update_hb_way(w)		:= Mux(is_cstr, scq(scq_idx).bits.way,
															WrapIncWay(scq(scq_idx).bits.way, scq(scq_idx).bits.num_ways))
		}
		//yh+end
  }

  // Initially assume the speculative load wakeup failed
  io.core.ld_miss         := RegNext(io.core.spec_ld_wakeup.map(_.valid).reduce(_||_))
  val spec_ld_succeed = widthMap(w =>
    !RegNext(io.core.spec_ld_wakeup(w).valid) ||
    (io.core.exe(w).iresp.valid &&
      io.core.exe(w).iresp.bits.uop.ldq_idx === RegNext(mem_incoming_uop(w).ldq_idx)
    )
  ).reduce(_&&_)
  when (spec_ld_succeed) {
    io.core.ld_miss := false.B
  }

  //-------------------------------------------------------------
  // Kill speculated entries on branch mispredict
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  // Kill stores
  val st_brkilled_mask = Wire(Vec(numStqEntries, Bool()))
  for (i <- 0 until numStqEntries)
  {
    st_brkilled_mask(i) := false.B

    when (stq(i).valid)
    {
      stq(i).bits.uop.br_mask := GetNewBrMask(io.core.brupdate, stq(i).bits.uop.br_mask)

      when (IsKilledByBranch(io.core.brupdate, stq(i).bits.uop))
      {
        stq(i).valid           := false.B
        stq(i).bits.addr.valid := false.B
        stq(i).bits.data.valid := false.B
        st_brkilled_mask(i)    := true.B
				//yh+begin
				printf("[%d] Misprediction inits stq(%d)\n", io.core.tsc_reg, i.U)
				//yh+end
      }
    }

    assert (!(IsKilledByBranch(io.core.brupdate, stq(i).bits.uop) && stq(i).valid && stq(i).bits.committed),
      "Branch is trying to clear a committed store.")
  }

  // Kill loads
  for (i <- 0 until numLdqEntries)
  {
    when (ldq(i).valid)
    {
      ldq(i).bits.uop.br_mask := GetNewBrMask(io.core.brupdate, ldq(i).bits.uop.br_mask)
      when (IsKilledByBranch(io.core.brupdate, ldq(i).bits.uop))
      {
        ldq(i).valid           := false.B
        ldq(i).bits.addr.valid := false.B
				//yh+begin
				printf("[%d] Misprediction inits ldq(%d)\n", io.core.tsc_reg, i.U)
				//yh+end
      }
    }
  }

  //yh+begin
  // Kill SLQ entries
  for (i <- 0 until numSlqEntries)
  {
    when (slq(i).valid)
    {
      slq(i).bits.uop.br_mask := GetNewBrMask(io.core.brupdate, slq(i).bits.uop.br_mask)

      when (IsKilledByBranch(io.core.brupdate, slq(i).bits.uop))
      {
        slq(i).valid           := false.B
				slq(i).bits.state			 := s_init
				printf("[%d] Misprediction inits slq(%d)\n", io.core.tsc_reg, i.U)
      }
    }
  }

  // Kill SSQ entries
  for (i <- 0 until numSsqEntries)
  {
    when (ssq(i).valid)
    {
      ssq(i).bits.uop.br_mask := GetNewBrMask(io.core.brupdate, ssq(i).bits.uop.br_mask)

      when (IsKilledByBranch(io.core.brupdate, ssq(i).bits.uop))
      {
        ssq(i).valid           := false.B
				ssq(i).bits.state			 := s_init
				printf("[%d] Misprediction inits ssq(%d)\n", io.core.tsc_reg, i.U)
      }
    }
  }

  // Kill SCQ entries
  val sc_brkilled_mask = Wire(Vec(numScqEntries, Bool()))
  for (i <- 0 until numScqEntries)
  {
    sc_brkilled_mask(i) := false.B

    when (scq(i).valid)
    {
      scq(i).bits.uop.br_mask := GetNewBrMask(io.core.brupdate, scq(i).bits.uop.br_mask)

      when (IsKilledByBranch(io.core.brupdate, scq(i).bits.uop))
      {
        scq(i).valid           := false.B
        sc_brkilled_mask(i)    := true.B
				scq(i).bits.data.valid := false.B
				scq(i).bits.state			 := s_init
				printf("[%d] Misprediction inits scq(%d)\n", io.core.tsc_reg, i.U)
      }
    }

    //assert (!(IsKilledByBranch(io.core.brupdate, scq(i).bits.uop) && scq(i).valid && scq(i).bits.committed),
    //  "Branch is trying to clear a committed store check.")
  }
  //yh+end

  //-------------------------------------------------------------
  when (io.core.brupdate.b2.mispredict && !io.core.exception)
  {
    stq_tail := io.core.brupdate.b2.uop.stq_idx
    ldq_tail := io.core.brupdate.b2.uop.ldq_idx

    //yh+begin
    slq_tail := io.core.brupdate.b2.uop.slq_idx
    ssq_tail := io.core.brupdate.b2.uop.ssq_idx
    scq_tail := io.core.brupdate.b2.uop.scq_idx

		printf("[%d] Misprediction occurred stq_idx: %d ldq_idx: %d slq_idx: %d ssq_idx: %d scq_idx: %d\n",
						io.core.tsc_reg, io.core.brupdate.b2.uop.stq_idx,
						io.core.brupdate.b2.uop.ldq_idx,
            io.core.brupdate.b2.uop.slq_idx,
            io.core.brupdate.b2.uop.ssq_idx,
            io.core.brupdate.b2.uop.scq_idx)
    //yh+end
  }

  //-------------------------------------------------------------
  //-------------------------------------------------------------
  // dequeue old entries on commit
  //-------------------------------------------------------------
  //-------------------------------------------------------------

  var temp_stq_commit_head = stq_commit_head
  var temp_ldq_head        = ldq_head
	//yh+begin
  var temp_num_store = num_store
  var temp_num_load = num_load
	//yh+end
  for (w <- 0 until coreWidth)
  {
    //yh-val commit_store = io.core.commit.valids(w) && io.core.commit.uops(w).uses_stq
    //yh+begin
    val commit_store = (io.core.commit.valids(w) && io.core.commit.uops(w).uses_stq &&
                        !io.core.commit.uops(w).is_cap)
    //yh+end
    val commit_load  = io.core.commit.valids(w) && io.core.commit.uops(w).uses_ldq
    val idx = Mux(commit_store, temp_stq_commit_head, temp_ldq_head)
    when (commit_store)
    {
      stq(idx).bits.committed := true.B
      //yh+begin
      assert(!io.core.commit.uops(w).is_cap)
      printf("[%d] Commit stq(%d)\n", io.core.tsc_reg, idx)
      //yh+end
    } .elsewhen (commit_load) {
      assert (ldq(idx).valid, "[lsu] trying to commit an un-allocated load entry.")
      assert ((ldq(idx).bits.executed || ldq(idx).bits.forward_std_val) && ldq(idx).bits.succeeded ,
        "[lsu] trying to commit an un-executed load entry.")

      ldq(idx).valid                 := false.B
      ldq(idx).bits.addr.valid       := false.B
      ldq(idx).bits.executed         := false.B
      ldq(idx).bits.succeeded        := false.B
      ldq(idx).bits.order_fail       := false.B
      ldq(idx).bits.forward_std_val  := false.B
			//yh+begin
      printf("[%d] Commit ldq(%d)\n", io.core.tsc_reg, idx)
			//yh+end
    }

    if (MEMTRACE_PRINTF) {
      when (commit_store || commit_load) {
        val uop    = Mux(commit_store, stq(idx).bits.uop, ldq(idx).bits.uop)
        val addr   = Mux(commit_store, stq(idx).bits.addr.bits, ldq(idx).bits.addr.bits)
        val stdata = Mux(commit_store, stq(idx).bits.data.bits, 0.U)
        val wbdata = Mux(commit_store, stq(idx).bits.debug_wb_data, ldq(idx).bits.debug_wb_data)
        printf("MT %x %x %x %x %x %x %x\n",
          io.core.tsc_reg, uop.uopc, uop.mem_cmd, uop.mem_size, addr, stdata, wbdata)
      }
    }

    temp_stq_commit_head = Mux(commit_store,
                               WrapInc(temp_stq_commit_head, numStqEntries),
                               temp_stq_commit_head)

    temp_ldq_head        = Mux(commit_load,
                               WrapInc(temp_ldq_head, numLdqEntries),
                               temp_ldq_head)

		//yh+begin
		temp_num_store = Mux(enableStat & commit_store, temp_num_store + 1.U, temp_num_store)
		temp_num_load = Mux(enableStat & commit_load, temp_num_load + 1.U, temp_num_load)
		when (commit_store || commit_load) {
			printf("[%d] temp_num_load: %x\n", io.core.tsc_reg, temp_num_load)
		}
		//yh+end
  }
  stq_commit_head := temp_stq_commit_head
  ldq_head        := temp_ldq_head

  //yh+begin
  num_store := Mux(initStat, io.core.dpt_csrs.num_store, temp_num_store)
  num_load := Mux(initStat, io.core.dpt_csrs.num_load, temp_num_load)

  for (i <- 0 until numSlqEntries) {
    when (slq(i).valid && slq(i).bits.uop.rob_idx === io.core.rob_head_idx) {
      slq(i).bits.is_rob_head := true.B

      when (!slq(i).bits.is_rob_head) {
        printf("[%d] Set slq(%d) is_rob_head\n", io.core.tsc_reg, i.U)
      }
    }
  }

  for (i <- 0 until numSsqEntries) {
    when (ssq(i).valid && ssq(i).bits.uop.rob_idx === io.core.rob_head_idx) {
      ssq(i).bits.is_rob_head := true.B

      when (!ssq(i).bits.is_rob_head) {
        printf("[%d] Set ssq(%d) is_rob_head\n", io.core.tsc_reg, i.U)
      }
    }
  }
  
  for (i <- 0 until numScqEntries) {
    when (scq(i).valid && scq(i).bits.uop.rob_idx === io.core.rob_head_idx) {
      scq(i).bits.is_rob_head := true.B

      when (!scq(i).bits.is_rob_head) {
        printf("[%d] Set scq(%d) is_rob_head\n", io.core.tsc_reg, i.U)
      }
    }
  }
  //yh+end

  // store has been committed AND successfully sent data to memory
  when (stq(stq_head).valid && stq(stq_head).bits.committed)
  {
    when (stq(stq_head).bits.uop.is_fence && !io.dmem.ordered) {
      io.dmem.force_order := true.B
      store_needs_order   := true.B
    }
    clear_store := Mux(stq(stq_head).bits.uop.is_fence, io.dmem.ordered,
                                                        stq(stq_head).bits.succeeded)
		//yh+begin
		when (clear_store) {
			printf("[%d] Clear store stq(%d), rob(%d)\n", io.core.tsc_reg, stq_head, stq(stq_head).bits.uop.rob_idx)
		}
		//yh+end
  }

  when (clear_store)
  {
    stq(stq_head).valid           := false.B
    stq(stq_head).bits.addr.valid := false.B
    stq(stq_head).bits.data.valid := false.B
    stq(stq_head).bits.succeeded  := false.B
    stq(stq_head).bits.committed  := false.B

    stq_head := WrapInc(stq_head, numStqEntries)
    when (stq(stq_head).bits.uop.is_fence)
    {
      stq_execute_head := WrapInc(stq_execute_head, numStqEntries)
    }
  }

	//yh+begin
  var temp_slq_head = slq_head
  var temp_ssq_head = ssq_head

  var temp_num_tagged_load = num_tagged_load
  var temp_num_load_hit = num_load_hit
  var temp_num_tagged_store = num_tagged_store
  var temp_num_store_hit = num_store_hit
  var temp_num_slq_itr = num_slq_itr
  var temp_num_ssq_itr = num_ssq_itr

  for (w <- 0 until coreWidth)
  {
	  val commit_ldchk = (io.core.commit.valids(w) && io.core.commit.uops(w).uses_ldq &&
                        io.core.commit.uops(w).needCC)

		when (commit_ldchk) {
      assert (slq(temp_slq_head).valid, "[lsu] trying to commit an un-allocated SLQ entry.")
      assert (slq(temp_slq_head).bits.state === s_done,
							"[lsu] trying to commit an un-executed SLQ entry.")

      slq(temp_slq_head).valid      := false.B
      slq(temp_slq_head).bits.state	:= s_init
      printf("[%d] Commit slq(%d) num_ways: %d count: %d\n",
							io.core.tsc_reg, temp_slq_head, slq(temp_slq_head).bits.num_ways, slq(temp_slq_head).bits.count)
		}

		temp_num_tagged_load = Mux(enableStat & commit_ldchk & slq(temp_slq_head).bits.tagged,
															temp_num_tagged_load + 1.U, temp_num_tagged_load)
		temp_num_load_hit = Mux(enableStat & commit_ldchk & slq(temp_slq_head).bits.ccache_hit,
															temp_num_load_hit + 1.U, temp_num_load_hit)
		temp_num_slq_itr = Mux(enableStat & commit_ldchk & slq(temp_slq_head).bits.tagged,
															temp_num_slq_itr + (slq(temp_slq_head).bits.num_ways - slq(temp_slq_head).bits.count),
															temp_num_slq_itr)
    temp_slq_head        = Mux(commit_ldchk, WrapInc(temp_slq_head, numSlqEntries), temp_slq_head)

    val commit_stchk = (io.core.commit.valids(w) && io.core.commit.uops(w).uses_stq &&
                        io.core.commit.uops(w).needCC && !io.core.commit.uops(w).is_cap)

		when (commit_stchk) {
      assert (ssq(temp_ssq_head).valid, "[lsu] trying to commit an un-allocated SSQ entry.")
      assert (ssq(temp_ssq_head).bits.state === s_done,
							"[lsu] trying to commit an un-executed SSQ entry.")

      ssq(temp_ssq_head).valid      := false.B
      ssq(temp_ssq_head).bits.state	:= s_init
      printf("[%d] Commit ssq(%d) num_ways: %d count: %d\n",
							io.core.tsc_reg, temp_ssq_head, ssq(temp_ssq_head).bits.num_ways, ssq(temp_ssq_head).bits.count)
		}

		temp_num_tagged_store = Mux(enableStat & commit_stchk & ssq(temp_ssq_head).bits.tagged,
																temp_num_tagged_store + 1.U, temp_num_tagged_store)
		temp_num_store_hit 		= Mux(enableStat & commit_stchk & ssq(temp_ssq_head).bits.ccache_hit,
																temp_num_store_hit + 1.U, temp_num_store_hit)
		temp_num_ssq_itr = Mux(enableStat & commit_stchk & ssq(temp_ssq_head).bits.tagged,
															temp_num_ssq_itr + (ssq(temp_ssq_head).bits.num_ways - ssq(temp_ssq_head).bits.count),
															temp_num_ssq_itr)
    temp_ssq_head         = Mux(commit_stchk, WrapInc(temp_ssq_head, numSsqEntries), temp_ssq_head)
	}

	slq_head := temp_slq_head
	ssq_head := temp_ssq_head

  num_tagged_load := Mux(initStat, io.core.dpt_csrs.num_tagged_load, temp_num_tagged_load)
  num_tagged_store := Mux(initStat, io.core.dpt_csrs.num_tagged_store, temp_num_tagged_store)
  num_load_hit := Mux(initStat, io.core.dpt_csrs.num_load_hit, temp_num_load_hit)
  num_store_hit := Mux(initStat, io.core.dpt_csrs.num_store_hit, temp_num_store_hit)
  num_slq_itr := Mux(initStat, io.core.dpt_csrs.num_slq_itr, temp_num_slq_itr)
  num_ssq_itr := Mux(initStat, io.core.dpt_csrs.num_ssq_itr, temp_num_ssq_itr)

	when (update_shb_valid(0)) {
    //val tidx = update_hb_tag(0)(tagWidth/2-1,0)
    val tidx = update_hb_tag(0)
		val way = update_hb_way(0)

		//shb_meta(tidx) := update_hb_tag(0)(tagWidth-1,tagWidth/2)
		shb_data(tidx) := way
		printf("[%d] Update SHB[%x]: %d\n", io.core.tsc_reg, tidx, way)
	}

	//when (update_chb_valid(0)) {
  //  val tidx = update_hb_tag(0)(tagWidth/2-1,0)
	//	//TODO val way = WrapIncOrDecWay(update_hb_dir(0), update_hb_way(0), num_ways)
	//	val way = update_hb_way(0)

	//	chb_meta(tidx) := update_hb_tag(0)(tagWidth-1,tagWidth/2)
	//	chb_data(tidx) := way
	//	printf("[%d] Update CHB[%x]: %d\n", io.core.tsc_reg, tidx, way)
	//}

	when (enableDPT) {
		printf("[%d] slq_head: %d slq_tail: %d ssq_head: %d ssq_tail: %d scq_head: %d scq_tail: %d\n",
						io.core.tsc_reg, slq_head, slq_tail, ssq_head, ssq_tail, scq_head, scq_tail)
		printf("[%d] ldq_head: %d ldq_tail: %d stq_head: %d stq_tail: %d\n",
						io.core.tsc_reg, ldq_head, ldq_tail, stq_head, stq_tail)
	}
  //yh+end

  // -----------------------
  // Hellacache interface
  // We need to time things like a HellaCache would
  io.hellacache.req.ready := false.B
  io.hellacache.s2_nack   := false.B
  io.hellacache.s2_xcpt   := (0.U).asTypeOf(new rocket.HellaCacheExceptions)
  io.hellacache.resp.valid := false.B
  when (hella_state === h_ready) {
    io.hellacache.req.ready := true.B
    when (io.hellacache.req.fire) {
      hella_req   := io.hellacache.req.bits
      hella_state := h_s1
    }
  } .elsewhen (hella_state === h_s1) {
    can_fire_hella_incoming(memWidth-1) := true.B

    hella_data := io.hellacache.s1_data
    hella_xcpt := dtlb.io.resp(memWidth-1)

    when (io.hellacache.s1_kill) {
      when (will_fire_hella_incoming(memWidth-1) && dmem_req_fire(memWidth-1)) {
        hella_state := h_dead
      } .otherwise {
        hella_state := h_ready
      }
    } .elsewhen (will_fire_hella_incoming(memWidth-1) && dmem_req_fire(memWidth-1)) {
      hella_state := h_s2
    } .otherwise {
      hella_state := h_s2_nack
    }
  } .elsewhen (hella_state === h_s2_nack) {
    io.hellacache.s2_nack := true.B
    hella_state := h_ready
  } .elsewhen (hella_state === h_s2) {
    io.hellacache.s2_xcpt := hella_xcpt
    when (io.hellacache.s2_kill || hella_xcpt.asUInt =/= 0.U) {
      hella_state := h_dead
    } .otherwise {
      hella_state := h_wait
    }
  } .elsewhen (hella_state === h_wait) {
    for (w <- 0 until memWidth) {
      when (io.dmem.resp(w).valid && io.dmem.resp(w).bits.is_hella) {
        hella_state := h_ready

        io.hellacache.resp.valid       := true.B
        io.hellacache.resp.bits.addr   := hella_req.addr
        io.hellacache.resp.bits.tag    := hella_req.tag
        io.hellacache.resp.bits.cmd    := hella_req.cmd
        io.hellacache.resp.bits.signed := hella_req.signed
        io.hellacache.resp.bits.size   := hella_req.size
        io.hellacache.resp.bits.data   := io.dmem.resp(w).bits.data
      } .elsewhen (io.dmem.nack(w).valid && io.dmem.nack(w).bits.is_hella) {
        hella_state := h_replay
      }
    }
  } .elsewhen (hella_state === h_replay) {
    can_fire_hella_wakeup(memWidth-1) := true.B

    when (will_fire_hella_wakeup(memWidth-1) && dmem_req_fire(memWidth-1)) {
      hella_state := h_wait
    }
  } .elsewhen (hella_state === h_dead) {
    for (w <- 0 until memWidth) {
      when (io.dmem.resp(w).valid && io.dmem.resp(w).bits.is_hella) {
        hella_state := h_ready
      }
    }
  }

  //-------------------------------------------------------------
  // Exception / Reset

  // for the live_store_mask, need to kill stores that haven't been committed
  val st_exc_killed_mask = WireInit(VecInit((0 until numStqEntries).map(x=>false.B)))

  when (reset.asBool || io.core.exception)
  {
    ldq_head := 0.U
    ldq_tail := 0.U

    when (reset.asBool)
    {
      stq_head := 0.U
      stq_tail := 0.U
      stq_commit_head  := 0.U
      stq_execute_head := 0.U

      for (i <- 0 until numStqEntries)
      {
        stq(i).valid           := false.B
        stq(i).bits.addr.valid := false.B
        stq(i).bits.data.valid := false.B
        stq(i).bits.uop        := NullMicroOp

				//yh+begin
				printf("[%d] Reset inits stq(%d)\n", io.core.tsc_reg, i.U)
				//yh+end
      }
    }
      .otherwise // exception
    {
      stq_tail := stq_commit_head

      for (i <- 0 until numStqEntries)
      {
        when (!stq(i).bits.committed && !stq(i).bits.succeeded)
        {
          stq(i).valid           := false.B
          stq(i).bits.addr.valid := false.B
          stq(i).bits.data.valid := false.B
          st_exc_killed_mask(i)  := true.B

					//yh+begin
					printf("[%d] Exception inits stq(%d)\n", io.core.tsc_reg, i.U)
					//yh+end
        }
      }
    }

    for (i <- 0 until numLdqEntries)
    {
      ldq(i).valid           := false.B
      ldq(i).bits.addr.valid := false.B
      ldq(i).bits.executed   := false.B

      //yh+begin
      printf("[%d] Exception inits ldq(%d)\n", io.core.tsc_reg, i.U)
      //yh+end
    }
  }

  //yh+begin
  when (reset.asBool || io.core.exception) {
		slq_head := 0.U
		slq_tail := 0.U

		for (i <- 0 until numSlqEntries) {
			slq(i).valid           := false.B
			slq(i).bits.state			 := s_init
			printf("[%d] Reset inits slq(%d)\n", io.core.tsc_reg, i.U)
		}

		ssq_head := 0.U
		ssq_tail := 0.U

		for (i <- 0 until numSsqEntries) {
			ssq(i).valid           := false.B
			ssq(i).bits.state			 := s_init
			printf("[%d] Reset inits ssq(%d)\n", io.core.tsc_reg, i.U)
		}

		scq_head := 0.U
		scq_tail := 0.U

		for (i <- 0 until numScqEntries) {
			scq(i).valid           := false.B
			scq(i).bits.data.valid := false.B
			scq(i).bits.state			 := s_init
			printf("[%d] Reset inits scq(%d)\n", io.core.tsc_reg, i.U)
		}
  }
  //yh+end

  //-------------------------------------------------------------
  // Live Store Mask
  // track a bit-array of stores that are alive
  // (could maybe be re-produced from the stq_head/stq_tail, but need to know include spec_killed entries)

  // TODO is this the most efficient way to compute the live store mask?
  live_store_mask := next_live_store_mask &
                    ~(st_brkilled_mask.asUInt) &
                    ~(st_exc_killed_mask.asUInt)

  //yh+begin
  live_tag_mask := Mux(reset.asBool || io.core.exception, 0.U,
										(next_live_tag_mask &
                    ~(sc_brkilled_mask.asUInt))) //TODO can remove sc_exc?

	// Init all buffers upon context switch
	for (i <- 0 until 256) {
		when (initDPT) {
			//shb_meta(i) := 0.U
			shb_data(i) := 0.U
			//chb_meta(i) := 0.U
			//chb_data(i) := 0.U
			ccache_meta(i) := 0.U
			ccache_lbnd(i) := 0.U
			ccache_ubnd(i) := 0.U
		}
	}
  //yh+end
}

/**
 * Object to take an address and generate an 8-bit mask of which bytes within a
 * double-word.
 */
object GenByteMask
{
   def apply(addr: UInt, size: UInt): UInt =
   {
      val mask = Wire(UInt(8.W))
      mask := MuxCase(255.U(8.W), Array(
                   (size === 0.U) -> (1.U(8.W) << addr(2,0)),
                   (size === 1.U) -> (3.U(8.W) << (addr(2,1) << 1.U)),
                   (size === 2.U) -> Mux(addr(2), 240.U(8.W), 15.U(8.W)),
                   (size === 3.U) -> 255.U(8.W)))
      mask
   }
}

/**
 * ...
 */
class ForwardingAgeLogic(num_entries: Int)(implicit p: Parameters) extends BoomModule()(p)
{
   val io = IO(new Bundle
   {
      val addr_matches    = Input(UInt(num_entries.W)) // bit vector of addresses that match
                                                       // between the load and the SAQ
      val youngest_st_idx = Input(UInt(stqAddrSz.W)) // needed to get "age"

      val forwarding_val  = Output(Bool())
      val forwarding_idx  = Output(UInt(stqAddrSz.W))
   })

   // generating mask that zeroes out anything younger than tail
   val age_mask = Wire(Vec(num_entries, Bool()))
   for (i <- 0 until num_entries)
   {
      age_mask(i) := true.B
      when (i.U >= io.youngest_st_idx) // currently the tail points PAST last store, so use >=
      {
         age_mask(i) := false.B
      }
   }

   // Priority encoder with moving tail: double length
   val matches = Wire(UInt((2*num_entries).W))
   matches := Cat(io.addr_matches & age_mask.asUInt,
                  io.addr_matches)

   val found_match = Wire(Bool())
   found_match       := false.B
   io.forwarding_idx := 0.U

   // look for youngest, approach from the oldest side, let the last one found stick
   for (i <- 0 until (2*num_entries))
   {
      when (matches(i))
      {
         found_match := true.B
         io.forwarding_idx := (i % num_entries).U
      }
   }

   io.forwarding_val := found_match
}
