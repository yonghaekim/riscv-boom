//**************************************************************************
// Execution Units
//--------------------------------------------------------------------------
//
// Christopher Celio
// 2013 Apr 27
//
// The issue window schedules micro-ops onto a specific execution pipeline
// A given execution pipeline may contain multiple functional units; one or two
// read ports, and one or more writeports.
//
// TODO provide an easier way to describe which functional units you want
// instead of providing a ton of hand-written variants.
// FPU+ALU+MUL+DIV+MEM, etc.


package BOOM
{

import Chisel._
import Node._

import FUCode._
import uncore.constants.MemoryOpConstants._
import rocket.BuildFPU

class ExeUnitResp(data_width: Int) extends BOOMCoreBundle
{
   val uop = new MicroOp()
   val data = Bits(width = data_width)
   val fflags = new ValidIO(new FFlagsResp) // write fflags to ROB
   override def clone = new ExeUnitResp(data_width).asInstanceOf[this.type]
}

class FFlagsResp extends BOOMCoreBundle
{
   val uop = new MicroOp()
   val flags = Bits(width=rocket.FPConstants.FLAGS_SZ)
}

class ExecutionUnitIo(num_rf_read_ports: Int
                     , num_rf_write_ports: Int
                     , num_bypass_ports: Int
                     , data_width: Int
                     ) extends Bundle with BOOMCoreParameters
{
   // describe which functional units we support (used by the issue window)
   val fu_types = Bits(OUTPUT, FUC_SZ)

   val req     = (new DecoupledIO(new FuncUnitReq(data_width))).flip
   val resp    = Vec.fill(num_rf_write_ports) { (new DecoupledIO(new ExeUnitResp(data_width))) }
   val bypass  = new BypassData(num_bypass_ports, data_width).asOutput()

   val brinfo  = new BrResolutionInfo().asInput()

   // only used by the branch unit
   val br_unit = new BranchUnitResp().asOutput
   val get_rob_pc = new Bundle
   {
      val rob_idx = UInt(OUTPUT, ROB_ADDR_SZ)
      val curr_pc = UInt(INPUT, xLen)
      val next_val= Bool(INPUT)
      val next_pc = UInt(INPUT, xLen)
   }

   // only used by the fpu unit
   val fcsr_rm = Bits(INPUT, rocket.FPConstants.RM_SZ)

   // only used by the mem unit
   val lsu_io = new LoadStoreUnitIo(DECODE_WIDTH)
   val dmem   = new DCMemPortIo()
   val com_handling_exc = Bool(INPUT)
}

abstract class ExecutionUnit(val num_rf_read_ports: Int
                            , val num_rf_write_ports: Int
                            , val num_bypass_stages: Int
                            , val data_width: Int
                            , val num_variable_write_ports: Int = 0
                            , var bypassable: Boolean           = false
                            , val is_mem_unit: Boolean          = false
                            , var uses_csr_wport: Boolean       = false
                            ,     is_branch_unit: Boolean       = false
                            , val has_fpu       : Boolean       = false // can return fflags
                            ) extends Module with BOOMCoreParameters
{
   val io = new ExecutionUnitIo(num_rf_read_ports, num_rf_write_ports
                               , num_bypass_stages, data_width)

   val uses_rf_wport = false

   if (!has_fpu)
   {
      io.resp.map(_.bits.fflags.valid := Bool(false))
   }

   def num_bypass_ports: Int = num_bypass_stages
   def has_branch_unit : Boolean = is_branch_unit
   def is_bypassable   : Boolean = bypassable
}

 
class ALUExeUnit(is_branch_unit: Boolean = false
                , shares_csr_wport: Boolean = false
                ) extends ExecutionUnit(num_rf_read_ports = 2
                                       , num_rf_write_ports = 1
                                       , num_bypass_stages = 1
                                       , data_width = 64 // TODO need to use xLen here
                                       , bypassable = true
                                       , is_mem_unit = false
                                       , uses_csr_wport = shares_csr_wport
                                       , is_branch_unit = is_branch_unit
                                       )
{
   io.fu_types := FU_ALU |
                  FU_CNTR |
                  (Mux(Bool(shares_csr_wport), FU_CSR, Bits(0))) |
                  (Mux(Bool(is_branch_unit), FU_BRU, Bits(0)))


   val alu = Module(new ALUUnit(is_branch_unit = is_branch_unit))
   alu.io.req <> io.req
   io.resp(0) <> alu.io.resp

   alu.io.brinfo <> io.brinfo
   io.bypass <> alu.io.bypass

   // branch unit is embedded inside the ALU
   if (is_branch_unit)
   {
      io.br_unit <> alu.io.br_unit
      alu.io.get_rob_pc <> io.get_rob_pc
   }
   else
   {
      io.br_unit.brinfo.valid := Bool(false)
   }

}

// TODO how to combine with above aluexeunit? 
class ALUMulExeUnit(is_branch_unit: Boolean = false
                , shares_csr_wport: Boolean = false
                ) extends ExecutionUnit(num_rf_read_ports = 2
                                       , num_rf_write_ports = 1
                                       , num_bypass_stages = 3
                                       , data_width = 64 // TODO need to use xLen here
                                       , bypassable = true
                                       , is_mem_unit = false
                                       , uses_csr_wport = shares_csr_wport
                                       , is_branch_unit = is_branch_unit
                                       )
{
   io.fu_types := FU_ALU |
                  FU_CNTR |
                  FU_MUL |
                  (Mux(Bool(shares_csr_wport), FU_CSR, Bits(0))) |
                  (Mux(Bool(is_branch_unit), FU_BRU, Bits(0)))


   // ALU Unit -------------------------------
   val alu = Module(new ALUUnit(is_branch_unit = is_branch_unit, num_stages=IMUL_STAGES))
   alu.io.req.valid         := io.req.valid &&
                                   ((io.req.bits.uop.fu_code === FU_ALU) ||
                                   (io.req.bits.uop.fu_code === FU_BRU) ||
                                   (io.req.bits.uop.fu_code === FU_CSR) ||
                                   (io.req.bits.uop.fu_code === FU_CNTR))
   alu.io.req.bits.uop      := io.req.bits.uop
   alu.io.req.bits.kill     := io.req.bits.kill
   alu.io.req.bits.rs1_data := io.req.bits.rs1_data
   alu.io.req.bits.rs2_data := io.req.bits.rs2_data
   alu.io.brinfo <> io.brinfo
   io.bypass <> alu.io.bypass

   // branch unit is embedded inside the ALU
   if (is_branch_unit)
   {
      io.br_unit <> alu.io.br_unit
      alu.io.get_rob_pc <> io.get_rob_pc
   }
   else
   {
      io.br_unit.brinfo.valid := Bool(false)
   }
 
   // Pipelined, IMul Unit ------------------
   val imul = Module(new PipelinedMulUnit(IMUL_STAGES))
   imul.io.req.valid := io.req.valid && io.req.bits.uop.fu_code_is(FU_MUL)
   imul.io.req.bits.uop      := io.req.bits.uop
   imul.io.req.bits.rs1_data := io.req.bits.rs1_data
   imul.io.req.bits.rs2_data := io.req.bits.rs2_data
   imul.io.req.bits.kill     := io.req.bits.kill
   imul.io.brinfo <> io.brinfo

   // Outputs (Write Port #0)  ---------------
   io.resp(0).valid     := alu.io.resp.valid || imul.io.resp.valid
   io.resp(0).bits.uop  := Mux(imul.io.resp.valid, imul.io.resp.bits.uop, alu.io.resp.bits.uop)
   io.resp(0).bits.data := Mux(imul.io.resp.valid, imul.io.resp.bits.data, alu.io.resp.bits.data)
    
}

class FPUALUMulExeUnit(is_branch_unit: Boolean = false
                , shares_csr_wport: Boolean = false
                ) extends ExecutionUnit(num_rf_read_ports = 3
                                       , num_rf_write_ports = 1
                                       , num_bypass_stages = 3 // TODO FPU LATENCY ADAM
                                       , data_width = 65
                                       , bypassable = true
                                       , is_mem_unit = false
                                       , uses_csr_wport = shares_csr_wport
                                       , is_branch_unit = is_branch_unit
                                       , has_fpu = true
                                       )
{
   io.fu_types := FU_ALU |
                  FU_CNTR |
                  FU_FPU |
                  FU_MUL |
                  (Mux(Bool(shares_csr_wport), FU_CSR, Bits(0))) |
                  (Mux(Bool(is_branch_unit), FU_BRU, Bits(0)))


   // ALU Unit -------------------------------
   val alu = Module(new ALUUnit(is_branch_unit = is_branch_unit, num_stages=3)) // TODO FPU LATENCY
   alu.io.req.valid         := io.req.valid &&
                                   ((io.req.bits.uop.fu_code === FU_ALU) ||
                                   (io.req.bits.uop.fu_code === FU_BRU) ||
                                   (io.req.bits.uop.fu_code === FU_CSR) ||
                                   (io.req.bits.uop.fu_code === FU_CNTR))
   alu.io.req.bits.uop      := io.req.bits.uop
   alu.io.req.bits.kill     := io.req.bits.kill
   alu.io.req.bits.rs1_data := io.req.bits.rs1_data
   alu.io.req.bits.rs2_data := io.req.bits.rs2_data

   alu.io.brinfo <> io.brinfo
   io.bypass <> alu.io.bypass

   // branch unit is embedded inside the ALU
   if (is_branch_unit)
   {
      io.br_unit <> alu.io.br_unit
      alu.io.get_rob_pc <> io.get_rob_pc
   }
   else
   {
      io.br_unit.brinfo.valid := Bool(false)
   }
 
   // Pipelined, IMul Unit ------------------
   val imul = Module(new PipelinedMulUnit(IMUL_STAGES))
   imul.io.req.valid := io.req.valid && io.req.bits.uop.fu_code_is(FU_MUL)
   imul.io.req.bits.uop      := io.req.bits.uop
   imul.io.req.bits.rs1_data := io.req.bits.rs1_data
   imul.io.req.bits.rs2_data := io.req.bits.rs2_data
   imul.io.req.bits.kill     := io.req.bits.kill
   imul.io.brinfo <> io.brinfo

   // FPU Unit -----------------------

   val fpu = Module(new FPUUnit())
   fpu.io.req.valid           := io.req.valid &&
                                 io.req.bits.uop.fu_code === FU_FPU
   fpu.io.req.bits.uop        := io.req.bits.uop
   fpu.io.req.bits.rs1_data   := io.req.bits.rs1_data
   fpu.io.req.bits.rs2_data   := io.req.bits.rs2_data
   fpu.io.req.bits.rs3_data   := io.req.bits.rs3_data
   fpu.io.req.bits.kill       := io.req.bits.kill
   fpu.io.fcsr_rm             := io.fcsr_rm
   // TODO use bundle interfacing

   fpu.io.brinfo <> io.brinfo

   // Outputs (Write Port #0)  ---------------

   io.resp(0).valid     := alu.io.resp.valid || fpu.io.resp.valid || imul.io.resp.valid
   io.resp(0).bits.uop  := Mux(fpu.io.resp.valid,  fpu.io.resp.bits.uop, 
                           Mux(imul.io.resp.valid, imul.io.resp.bits.uop,
                                                   alu.io.resp.bits.uop))
   io.resp(0).bits.data := Mux(fpu.io.resp.valid,  fpu.io.resp.bits.data,
                           Mux(imul.io.resp.valid, imul.io.resp.bits.data,
                                                   alu.io.resp.bits.data))

   // TODO is there a way to override a single signal in a bundle?
//   io.resp(0).bits.fflags <> fpu.io.resp.bits.fflags
   io.resp(0).bits.fflags.valid      := fpu.io.resp.valid
   io.resp(0).bits.fflags.bits.uop   := fpu.io.resp.bits.fflags.bits.uop
   io.resp(0).bits.fflags.bits.flags := fpu.io.resp.bits.fflags.bits.flags

   assert (PopCount(List(alu,fpu,imul).map(_.io.resp.valid)) <= UInt(1)
      , "ALU,IMUL, and/or FPU are fighting over the write port.")

}


class MulDExeUnit extends ExecutionUnit(num_rf_read_ports = 2
                                       , num_rf_write_ports = 1
                                       , num_bypass_stages = 0
                                       , data_width = 64 // TODO need to use xLen here
                                       , num_variable_write_ports = 1
                                       )
{
   val muldiv_busy = Bool()
   io.fu_types := Mux(!muldiv_busy, FU_MUL | FU_DIV, Bits(0))

   val muldiv = Module(new MulDivUnit())
   muldiv.io.req <> io.req

   io.resp(0) <> muldiv.io.resp
   io.resp(0).ready := Bool(true)

   muldiv.io.brinfo <> io.brinfo
   io.bypass <> muldiv.io.bypass

   muldiv_busy := !muldiv.io.req.ready || (io.req.valid)
}
// TODO listed as FIFOs, but not using ready signal


class ALUDivExeUnit(is_branch_unit: Boolean = false
                    , shares_csr_wport: Boolean = false
                    ) extends ExecutionUnit(num_rf_read_ports = 2
                                           , num_rf_write_ports = 1
                                           , num_bypass_stages = 1
                                           , data_width = 64 // TODO need to use xprlen
                                           , num_variable_write_ports = 1
                                           , bypassable = true
                                           , is_mem_unit = false
                                           , uses_csr_wport = shares_csr_wport
                                           , is_branch_unit = is_branch_unit
                                           )
{
   val muldiv_busy = Bool()
   io.fu_types := (FU_ALU |
                  FU_CNTR |
                  (Mux(!muldiv_busy, FU_DIV, Bits(0))) |
                  (Mux(Bool(shares_csr_wport), FU_CSR, Bits(0))) |
                  (Mux(Bool(is_branch_unit), FU_BRU, Bits(0))))


   // ALU Unit -------------------------------
   val alu = Module(new ALUUnit(is_branch_unit = is_branch_unit))
   alu.io.req.valid         := io.req.valid &&
                                    ((io.req.bits.uop.fu_code === FU_ALU) ||
                                     (io.req.bits.uop.fu_code === FU_BRU) ||
                                     (io.req.bits.uop.fu_code === FU_CSR) ||
                                     (io.req.bits.uop.fu_code === FU_CNTR))
   alu.io.req.bits.uop      := io.req.bits.uop
   alu.io.req.bits.kill     := io.req.bits.kill
   alu.io.req.bits.rs1_data := io.req.bits.rs1_data
   alu.io.req.bits.rs2_data := io.req.bits.rs2_data

   // branch unit is embedded inside the ALU
   if (is_branch_unit)
   {
      io.br_unit <> alu.io.br_unit
      alu.io.get_rob_pc <> io.get_rob_pc
   }
   else
   {
      io.br_unit.brinfo.valid := Bool(false)
   }


   // Mul/Div/Rem Unit -----------------------
   val muldiv = Module(new MulDivUnit())

   muldiv.io.req.valid           := io.req.valid &&
                                    (io.req.bits.uop.fu_code === FU_DIV)
   muldiv.io.req.bits.uop        := io.req.bits.uop
   muldiv.io.req.bits.rs1_data   := io.req.bits.rs1_data
   muldiv.io.req.bits.rs2_data   := io.req.bits.rs2_data
   muldiv.io.brinfo              := io.brinfo
   muldiv.io.req.bits.kill       := io.req.bits.kill

   muldiv.io.resp.ready := !alu.io.resp.valid // share write port with the ALU

   muldiv_busy := !muldiv.io.req.ready || (io.req.valid && io.req.bits.uop.fu_code_is(FU_DIV))

   // Branch Resolution ------------------------

   alu.io.brinfo <> io.brinfo
   muldiv.io.brinfo <> io.brinfo

   // Bypassing --------------------------------
   // (only the ALU is bypassable)

   io.bypass <> alu.io.bypass

   // Outputs ----------------------------------
   // hook up responses

   io.resp(0).valid := alu.io.resp.valid || muldiv.io.resp.valid
   io.resp(0).bits.uop         := Mux(alu.io.resp.valid, alu.io.resp.bits.uop,
                                                         muldiv.io.resp.bits.uop)
   io.resp(0).bits.data        := Mux(alu.io.resp.valid, alu.io.resp.bits.data,
                                                         muldiv.io.resp.bits.data)

}


class MemExeUnit extends ExecutionUnit(num_rf_read_ports = 2 // TODO make this 1, requires MemAddrCalcUnit to accept store data on rs1_data port
                                      , num_rf_write_ports = 1
                                      , num_bypass_stages = 0
                                      , data_width = 65 // TODO need to know if params(BuildFPU).isEmpty here
                                      , num_variable_write_ports = 1
                                      , bypassable = false
                                      , is_mem_unit = true)
{
   io.fu_types := FU_MEM

   // Perform address calculation
   val maddrcalc = Module(new MemAddrCalcUnit())
   maddrcalc.io.req <> io.req

   maddrcalc.io.brinfo <> io.brinfo
   io.bypass <> maddrcalc.io.bypass  // TODO this is not where the bypassing should occur from, is there any bypassing happening?!

   val lsu = Module(new LoadStoreUnit(DECODE_WIDTH))

   // TODO does this interface have to be so verbose? for the LSU connections
   // we want "lsu.io <> io.lsu_io"
   lsu.io.dec_st_vals       := io.lsu_io.dec_st_vals
   lsu.io.dec_ld_vals       := io.lsu_io.dec_ld_vals
   lsu.io.dec_uops          := io.lsu_io.dec_uops


   lsu.io.commit_store_mask := io.lsu_io.commit_store_mask
   lsu.io.commit_load_mask  := io.lsu_io.commit_load_mask

   lsu.io.brinfo            := io.brinfo
   lsu.io.exception         := io.lsu_io.exception
   lsu.io.nack              <> io.dmem.nack
   lsu.io.counters          <> io.lsu_io.counters

   io.lsu_io.new_ldq_idx := lsu.io.new_ldq_idx
   io.lsu_io.new_stq_idx := lsu.io.new_stq_idx
   io.lsu_io.laq_full := lsu.io.laq_full
   io.lsu_io.stq_full := lsu.io.stq_full
   io.lsu_io.lsu_clr_bsy_valid := lsu.io.lsu_clr_bsy_valid // HACK TODO need a better way to clear the busy bits in the ROB
   io.lsu_io.lsu_clr_bsy_rob_idx := lsu.io.lsu_clr_bsy_rob_idx // HACK TODO need a better way to clear the busy bits in the rob
   io.lsu_io.lsu_fencei_rdy := lsu.io.lsu_fencei_rdy
   io.lsu_io.debug := lsu.io.debug

   // enqueue addresses,st-data at the end of Execute
   lsu.io.exe_resp <> maddrcalc.io.resp

   lsu.io.ptw <> io.lsu_io.ptw
   lsu.io.xcpt <> io.lsu_io.xcpt

   // HellaCache Req
   lsu.io.dmem_req_ready := io.dmem.req.ready
   lsu.io.dmem_is_ordered:= io.dmem.ordered


   // TODO get rid of com_handling and guard with an assert?
   io.dmem.req.valid     := Mux(io.com_handling_exc && lsu.io.memreq_uop.is_load, Bool(false),
                                                                              lsu.io.memreq_val)
   io.dmem.req.bits.addr  := lsu.io.memreq_addr
   io.dmem.req.bits.data  := lsu.io.memreq_wdata
   io.dmem.req.bits.uop   := lsu.io.memreq_uop
   io.dmem.req.bits.kill  := lsu.io.memreq_kill // load kill request sent to memory

   // I should be timing forwarding to coincide with dmem resps, so I'm not clobbering
   //anything....
   val memresp_val    = Mux(io.com_handling_exc && io.dmem.resp.bits.uop.is_load, Bool(false),
                                                lsu.io.forward_val || io.dmem.resp.valid)
   val memresp_rf_wen = (io.dmem.resp.valid && (io.dmem.resp.bits.uop.mem_cmd === M_XRD || io.dmem.resp.bits.uop.is_amo)) ||  // TODO should I refactor this to use is_load?
                           lsu.io.forward_val
   val memresp_uop    = Mux(lsu.io.forward_val, lsu.io.forward_uop,
                                                io.dmem.resp.bits.uop)

   var memresp_data:Bits = null
   if (params(BuildFPU).isEmpty)
   {
      memresp_data = Mux(lsu.io.forward_val, lsu.io.forward_data
                                           , io.dmem.resp.bits.data_subword)
   }
   else
   {
      //recode FP values
      val typ = io.dmem.resp.bits.typ
      val load_single = typ === MT_W || typ === MT_WU
      val rec_s = hardfloat.floatNToRecodedFloatN(io.dmem.resp.bits.data, 23, 9)
      val rec_d = hardfloat.floatNToRecodedFloatN(io.dmem.resp.bits.data, 52, 12)
      val fp_load_data_recoded = Mux(load_single, Cat(SInt(-1, 32), rec_s), rec_d)

      memresp_data = Mux(lsu.io.forward_val, lsu.io.forward_data
                   , Mux(memresp_uop.fp_val, fp_load_data_recoded
                                           , io.dmem.resp.bits.data_subword))
   }



   lsu.io.memresp.valid := memresp_val
   lsu.io.memresp.bits  := memresp_uop


   // Hook up loads to the response
   io.resp(0).valid := memresp_val
   io.resp(0).bits.uop := memresp_uop
   io.resp(0).bits.uop.ctrl.rf_wen := memresp_rf_wen
   io.resp(0).bits.data := memresp_data
}


class ALUMulDMemExeUnit(is_branch_unit: Boolean = false
                       , shares_csr_wport: Boolean = false
                       , use_slow_mul: Boolean = false
                       ) extends ExecutionUnit(num_rf_read_ports = 2
                                              , num_rf_write_ports = 2
                                              , num_bypass_stages = 1
                                              , data_width = 65 // TODO need to use params(BuildFPU).isEmpty here
                                              , num_variable_write_ports = 1
                                              , bypassable = true
                                              , is_mem_unit = true
                                              , uses_csr_wport = shares_csr_wport
                                              , is_branch_unit = is_branch_unit)
{
   val muldiv_busy = Bool()
   io.fu_types := (FU_ALU |
                  FU_CNTR |
                  FU_MEM |
                  (Mux(!muldiv_busy, FU_DIV, Bits(0))) |
                  (Mux(!muldiv_busy && Bool(use_slow_mul), FU_MUL, Bits(0))) |
                  (Mux(Bool(shares_csr_wport), FU_CSR, Bits(0))) |
                  (Mux(Bool(is_branch_unit), FU_BRU, Bits(0))))


   val memresp_val = Bool()


   // ALU Unit -------------------------------
   val alu = Module(new ALUUnit(is_branch_unit = true))
   alu.io.req.valid         := io.req.valid &&
                                    ((io.req.bits.uop.fu_code === FU_ALU) ||
                                     (io.req.bits.uop.fu_code === FU_BRU) ||
                                     (io.req.bits.uop.fu_code === FU_CSR) ||
                                     (io.req.bits.uop.fu_code === FU_CNTR))
   alu.io.req.bits.uop      := io.req.bits.uop
   alu.io.req.bits.kill     := io.req.bits.kill
   alu.io.req.bits.rs1_data := io.req.bits.rs1_data
   alu.io.req.bits.rs2_data := io.req.bits.rs2_data

   // branch unit is embedded inside the ALU
   if (is_branch_unit)
   {
      io.br_unit <> alu.io.br_unit
      alu.io.get_rob_pc <> io.get_rob_pc
   }
   else
   {
      io.br_unit.brinfo.valid := Bool(false)
   }

   // Outputs ----------------------------------
   // hook up responses

   io.resp(0) <> alu.io.resp


   // Mul/Div/Rem Unit -----------------------
   val muldiv = Module(new MulDivUnit())

   muldiv.io.req.valid           := io.req.valid &&
                                    (io.req.bits.uop.fu_code === FU_DIV)
   muldiv.io.req.bits.uop        := io.req.bits.uop
   muldiv.io.req.bits.rs1_data   := io.req.bits.rs1_data
   muldiv.io.req.bits.rs2_data   := io.req.bits.rs2_data
   muldiv.io.brinfo              := io.brinfo
   muldiv.io.req.bits.kill       := io.req.bits.kill

   muldiv.io.resp.ready := !memresp_val //share write port with the memory

   muldiv_busy := !muldiv.io.req.ready || (io.req.valid && io.req.bits.uop.fu_code === FU_DIV)

   // Branch Resolution ------------------------

   alu.io.brinfo <> io.brinfo
   muldiv.io.brinfo <> io.brinfo

   // Bypassing --------------------------------
   // (only the ALU is bypassable)

   io.bypass <> alu.io.bypass

   // Perform address calculation
   val maddrcalc = Module(new MemAddrCalcUnit())
   maddrcalc.io.req <> io.req

   maddrcalc.io.brinfo <> io.brinfo

   val lsu = Module(new LoadStoreUnit(DECODE_WIDTH))

   lsu.io.dec_st_vals       := io.lsu_io.dec_st_vals
   lsu.io.dec_ld_vals       := io.lsu_io.dec_ld_vals
   lsu.io.dec_uops          := io.lsu_io.dec_uops


   lsu.io.commit_store_mask := io.lsu_io.commit_store_mask
   lsu.io.commit_load_mask  := io.lsu_io.commit_load_mask

   lsu.io.brinfo            := io.brinfo
   lsu.io.exception         := io.lsu_io.exception
   lsu.io.nack              <> io.dmem.nack
   lsu.io.counters          <> io.lsu_io.counters

   io.lsu_io.new_ldq_idx := lsu.io.new_ldq_idx
   io.lsu_io.new_stq_idx := lsu.io.new_stq_idx
   io.lsu_io.laq_full := lsu.io.laq_full
   io.lsu_io.stq_full := lsu.io.stq_full
   io.lsu_io.lsu_clr_bsy_valid := lsu.io.lsu_clr_bsy_valid // HACK TODO need a better way to clear the busy bits in the ROB
   io.lsu_io.lsu_clr_bsy_rob_idx := lsu.io.lsu_clr_bsy_rob_idx // HACK TODO need a better way to clear the busy bits in the rob
   io.lsu_io.lsu_fencei_rdy := lsu.io.lsu_fencei_rdy
   io.lsu_io.debug := lsu.io.debug

   // enqueue addresses,st-data at the end of Execute
   lsu.io.exe_resp <> maddrcalc.io.resp

   lsu.io.ptw <> io.lsu_io.ptw
   lsu.io.xcpt <> io.lsu_io.xcpt

   // HellaCache Req
   lsu.io.dmem_req_ready := io.dmem.req.ready
   lsu.io.dmem_is_ordered:= io.dmem.ordered

   io.dmem.req.valid     := Mux(io.com_handling_exc && lsu.io.memreq_uop.is_load, Bool(false),
                                                                              lsu.io.memreq_val)
   io.dmem.req.bits.addr  := lsu.io.memreq_addr
   io.dmem.req.bits.data  := lsu.io.memreq_wdata
   io.dmem.req.bits.uop   := lsu.io.memreq_uop
   io.dmem.req.bits.kill  := lsu.io.memreq_kill // load kill request sent to memory

   // I'm timing forwarding to coincide with dmem resps, so I'm not clobbering
   //anything....
   memresp_val    := Mux(io.com_handling_exc && io.dmem.resp.bits.uop.is_load, Bool(false),
                                                lsu.io.forward_val || io.dmem.resp.valid)


   val memresp_rf_wen = (io.dmem.resp.valid && (io.dmem.resp.bits.uop.mem_cmd === M_XRD || io.dmem.resp.bits.uop.is_amo)) ||
                           lsu.io.forward_val
   val memresp_uop    = Mux(lsu.io.forward_val, lsu.io.forward_uop,
                                                io.dmem.resp.bits.uop)

   var memresp_data:Bits = null
   if (params(BuildFPU).isEmpty)
   {
      memresp_data = Mux(lsu.io.forward_val, lsu.io.forward_data
                                           , io.dmem.resp.bits.data_subword)
   }
   else
   {
      //recode FP values
      val typ = io.dmem.resp.bits.typ
      val load_single = typ === MT_W || typ === MT_WU
      val rec_s = hardfloat.floatNToRecodedFloatN(io.dmem.resp.bits.data, 23, 9)
      val rec_d = hardfloat.floatNToRecodedFloatN(io.dmem.resp.bits.data, 52, 12)
      val fp_load_data_recoded = Mux(load_single, Cat(SInt(-1, 32), rec_s), rec_d)

      memresp_data = Mux(lsu.io.forward_val, lsu.io.forward_data
                   , Mux(memresp_uop.fp_val, fp_load_data_recoded
                                           , io.dmem.resp.bits.data_subword))
   }

   lsu.io.memresp.valid := memresp_val
   lsu.io.memresp.bits  := memresp_uop


   // Hook up loads and multiplies to the 2nd write port
   io.resp(1).valid                := memresp_val || muldiv.io.resp.valid
   io.resp(1).bits.uop             := Mux(memresp_val, memresp_uop, muldiv.io.resp.bits.uop)
   io.resp(1).bits.uop.ctrl.rf_wen := Mux(memresp_val, memresp_rf_wen, muldiv.io.resp.bits.uop.ctrl.rf_wen)
   io.resp(1).bits.data            := Mux(memresp_val, memresp_data, muldiv.io.resp.bits.data)
}

// TODO add the FPU as an input flag to prevent too many separate classes here?
class FPUALUMulDMemExeUnit(is_branch_unit: Boolean = false
                       , shares_csr_wport: Boolean = false
                       , use_slow_mul    : Boolean = false
                       ) extends ExecutionUnit(num_rf_read_ports = 3
                                              , num_rf_write_ports = 2
                                              , num_bypass_stages = 3 // TODO FPU_LATENCY Adam
                                              , data_width = 65
                                              , num_variable_write_ports = 1
                                              , bypassable = true
                                              , is_mem_unit = true
                                              , uses_csr_wport = shares_csr_wport
                                              , is_branch_unit = is_branch_unit
                                              , has_fpu = true
                                             )
{
   require(!params(BuildFPU).isEmpty)

   val muldiv_busy = Bool()
   io.fu_types := (FU_ALU |
                  FU_CNTR |
                  FU_MEM |
                  FU_FPU |
                  (Mux(!muldiv_busy, FU_DIV, Bits(0))) |
                  (Mux(!muldiv_busy && Bool(use_slow_mul), FU_MUL, Bits(0))) |
                  (Mux(Bool(!use_slow_mul), FU_MUL, Bits(0))) |
                  (Mux(Bool(shares_csr_wport), FU_CSR, Bits(0))) |
                  (Mux(Bool(is_branch_unit), FU_BRU, Bits(0))))


   val memresp_val = Bool()


   // ALU Unit -------------------------------
   val alu = Module(new ALUUnit(is_branch_unit = true, num_stages=3)) // TODO FPU LATENCY
   alu.io.req.valid         := io.req.valid &&
                                    (io.req.bits.uop.fu_code_is(FU_ALU) ||
                                     io.req.bits.uop.fu_code_is(FU_BRU) ||
                                     io.req.bits.uop.fu_code_is(FU_CSR) ||
                                     io.req.bits.uop.fu_code_is(FU_CNTR))
   alu.io.req.bits.uop      := io.req.bits.uop
   alu.io.req.bits.kill     := io.req.bits.kill
   alu.io.req.bits.rs1_data := io.req.bits.rs1_data
   alu.io.req.bits.rs2_data := io.req.bits.rs2_data

   alu.io.brinfo <> io.brinfo

   // branch unit is embedded inside the ALU
   if (is_branch_unit)
   {
      io.br_unit <> alu.io.br_unit
      alu.io.get_rob_pc <> io.get_rob_pc
   }
   else
   {
      // TODO CODEREVIEW how does the ALU handle other things being branch units?
      io.br_unit.brinfo.valid := Bool(false)
   }

   // FPU Unit -----------------------

   val fpu = Module(new FPUUnit())
   fpu.io.req.valid           := io.req.valid &&
                                 io.req.bits.uop.fu_code_is(FU_FPU)
   fpu.io.req.bits.uop        := io.req.bits.uop
   fpu.io.req.bits.rs1_data   := io.req.bits.rs1_data
   fpu.io.req.bits.rs2_data   := io.req.bits.rs2_data
   fpu.io.req.bits.rs3_data   := io.req.bits.rs3_data
   fpu.io.req.bits.kill       := io.req.bits.kill
   fpu.io.fcsr_rm             := io.fcsr_rm
   fpu.io.brinfo <> io.brinfo
   // TODO use bundle interfacing

   // Outputs (Write Port #0)  ---------------

   io.resp(0).valid     := alu.io.resp.valid || fpu.io.resp.valid
   io.resp(0).bits.uop  := Mux(fpu.io.resp.valid, fpu.io.resp.bits.uop, alu.io.resp.bits.uop)
   io.resp(0).bits.data := Mux(fpu.io.resp.valid, fpu.io.resp.bits.data, alu.io.resp.bits.data)

//   io.resp(0).bits.fflags <> fpu.io.resp.bits.fflags
   io.resp(0).bits.fflags.valid      := fpu.io.resp.valid
   io.resp(0).bits.fflags.bits.uop   := fpu.io.resp.bits.fflags.bits.uop
   io.resp(0).bits.fflags.bits.flags := fpu.io.resp.bits.fflags.bits.flags

   assert (!(alu.io.resp.valid && fpu.io.resp.valid)
      , "ALU and FPU are fighting over the write port.")

   // Pipelined, IMul Unit -----------------------
   val imul = Module(new PipelinedMulUnit(IMUL_STAGES))
   imul.io.req.valid := io.req.valid && (io.req.bits.uop.fu_code_is(FU_MUL) && Bool(!use_slow_mul))
   imul.io.req.bits.uop      := io.req.bits.uop
   imul.io.req.bits.rs1_data := io.req.bits.rs1_data
   imul.io.req.bits.rs2_data := io.req.bits.rs2_data
   imul.io.req.bits.kill     := io.req.bits.kill
   imul.io.brinfo <> io.brinfo
   
   // Mul/Div/Rem Unit -----------------------
   val muldiv = Module(new MulDivUnit())

   muldiv.io.req.valid           := io.req.valid &&
                                    (io.req.bits.uop.fu_code_is(FU_DIV) ||
                                    (io.req.bits.uop.fu_code_is(FU_MUL) && Bool(use_slow_mul)))
   muldiv.io.req.bits.uop        := io.req.bits.uop
   muldiv.io.req.bits.rs1_data   := io.req.bits.rs1_data
   muldiv.io.req.bits.rs2_data   := io.req.bits.rs2_data
   muldiv.io.req.bits.kill       := io.req.bits.kill

   muldiv.io.brinfo <> io.brinfo

   muldiv.io.resp.ready := !memresp_val //share write port with the memory

   muldiv_busy := !muldiv.io.req.ready || 
                  (io.req.valid && (io.req.bits.uop.fu_code_is(FU_DIV) ||
                                   (io.req.bits.uop.fu_code_is(FU_MUL) && Bool(use_slow_mul))))


   // Bypassing --------------------------------
   // (only the ALU is bypassable)

   io.bypass <> alu.io.bypass

   // Perform address calculation
   val maddrcalc = Module(new MemAddrCalcUnit())
   maddrcalc.io.req <> io.req

   maddrcalc.io.brinfo <> io.brinfo

   val lsu = Module(new LoadStoreUnit(DECODE_WIDTH))

   lsu.io.dec_st_vals       := io.lsu_io.dec_st_vals
   lsu.io.dec_ld_vals       := io.lsu_io.dec_ld_vals
   lsu.io.dec_uops          := io.lsu_io.dec_uops


   lsu.io.commit_store_mask := io.lsu_io.commit_store_mask
   lsu.io.commit_load_mask  := io.lsu_io.commit_load_mask

   lsu.io.brinfo            := io.brinfo
   lsu.io.exception         := io.lsu_io.exception
   lsu.io.nack              <> io.dmem.nack
   lsu.io.counters          <> io.lsu_io.counters

   io.lsu_io.new_ldq_idx := lsu.io.new_ldq_idx
   io.lsu_io.new_stq_idx := lsu.io.new_stq_idx
   io.lsu_io.laq_full := lsu.io.laq_full
   io.lsu_io.stq_full := lsu.io.stq_full
   io.lsu_io.lsu_clr_bsy_valid := lsu.io.lsu_clr_bsy_valid // HACK TODO need a better way to clear the busy bits in the ROB
   io.lsu_io.lsu_clr_bsy_rob_idx := lsu.io.lsu_clr_bsy_rob_idx // HACK TODO need a better way to clear the busy bits in the rob
   io.lsu_io.lsu_fencei_rdy := lsu.io.lsu_fencei_rdy
   io.lsu_io.debug := lsu.io.debug

   // enqueue addresses,st-data at the end of Execute
   lsu.io.exe_resp <> maddrcalc.io.resp

   lsu.io.ptw <> io.lsu_io.ptw
   lsu.io.xcpt <> io.lsu_io.xcpt

   // HellaCache Req
   lsu.io.dmem_req_ready := io.dmem.req.ready
   lsu.io.dmem_is_ordered:= io.dmem.ordered

   io.dmem.req.valid     := Mux(io.com_handling_exc && lsu.io.memreq_uop.is_load, Bool(false),
                                                                              lsu.io.memreq_val)
   io.dmem.req.bits.addr  := lsu.io.memreq_addr
   io.dmem.req.bits.data  := lsu.io.memreq_wdata
   io.dmem.req.bits.uop   := lsu.io.memreq_uop
   io.dmem.req.bits.kill  := lsu.io.memreq_kill // load kill request sent to memory

   // I'm timing forwarding to coincide with dmem resps, so I'm not clobbering
   //anything....
   memresp_val := Mux(io.com_handling_exc && io.dmem.resp.bits.uop.is_load, Bool(false),
                                               lsu.io.forward_val || io.dmem.resp.valid)


   val memresp_rf_wen = (io.dmem.resp.valid && (io.dmem.resp.bits.uop.mem_cmd === M_XRD || io.dmem.resp.bits.uop.is_amo)) ||
                           lsu.io.forward_val
   val memresp_uop    = Mux(lsu.io.forward_val, lsu.io.forward_uop,
                                                io.dmem.resp.bits.uop)

   var memresp_data:Bits = null
   if (params(BuildFPU).isEmpty)
   {
      memresp_data = Mux(lsu.io.forward_val, lsu.io.forward_data
                                           , io.dmem.resp.bits.data_subword)
   }
   else
   {
      // TODO CODE REVIEW throwing resources to try and salvage critical path...
      //recode FP values
      // I'm doing this twice for two different paths (cache path and forwarding path)!
      val typ = io.dmem.resp.bits.typ
      val load_single = typ === MT_W || typ === MT_WU
      val rec_s = hardfloat.floatNToRecodedFloatN(io.dmem.resp.bits.data, 23, 9)
      val rec_d = hardfloat.floatNToRecodedFloatN(io.dmem.resp.bits.data, 52, 12)
      val fp_load_data_recoded = Mux(load_single, Cat(SInt(-1, 32), rec_s), rec_d)

      val typ_f = lsu.io.forward_uop.mem_typ
      val load_single_f = typ_f === MT_W || typ_f === MT_WU
      val rec_s_f = hardfloat.floatNToRecodedFloatN(lsu.io.forward_data, 23, 9)
      val rec_d_f = hardfloat.floatNToRecodedFloatN(lsu.io.forward_data, 52, 12)
      val fp_load_data_recoded_forwarded = Mux(load_single_f, Cat(SInt(-1,32), rec_s_f), rec_d_f)

      memresp_data = Mux(lsu.io.forward_val && !lsu.io.forward_uop.fp_val, lsu.io.forward_data,
                     Mux(lsu.io.forward_val && lsu.io.forward_uop.fp_val,  fp_load_data_recoded_forwarded,
                     Mux(io.dmem.resp.bits.uop.fp_val,                     fp_load_data_recoded,
                                                                           io.dmem.resp.bits.data_subword)))
   }

   lsu.io.memresp.valid := memresp_val
   lsu.io.memresp.bits  := memresp_uop

   // Hook up loads and multiplies to the 2nd write port
   io.resp(1).valid                := memresp_val || muldiv.io.resp.valid
   io.resp(1).bits.uop             := Mux(memresp_val, memresp_uop, muldiv.io.resp.bits.uop)
   io.resp(1).bits.uop.ctrl.rf_wen := Mux(memresp_val, memresp_rf_wen, muldiv.io.resp.bits.uop.ctrl.rf_wen)  // TODO get rid of this, it should come from the thing below
   io.resp(1).bits.data            := Mux(memresp_val, memresp_data, muldiv.io.resp.bits.data)
   io.resp(1).bits.fflags.valid    := Bool(false)
   io.resp(1).bits.fflags.bits.uop := NullMicroOp
   io.resp(1).bits.fflags.bits.flags:= Bits(0)
}


}
