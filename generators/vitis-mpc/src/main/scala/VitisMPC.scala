package vitis_mpc

import chisel3._
import chisel3.util._
import chisel3.experimental.{IntParam, BaseModule}
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.prci._
import freechips.rocketchip.subsystem.{BaseSubsystem, PBUS}
import org.chipsalliance.cde.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.UIntIsOneOf
import freechips.rocketchip.interrupts._

case class VitisMPCTrackerParams(
  address: BigInt = 0x4000,
  axi_control_data_width: Int = 32,
  axi_control_addr_width: Int = 7,
  axi_data_width: Int = 32,
  axi_control_wstrb_width: Int = (32/8),
  axi_wstrb_width: Int = (32/8))

case object VitisMPCTrackerKey extends Field[Option[VitisMPCTrackerParams]](None)

class VitisMPCTracker(params: VitisMPCTrackerParams)(implicit p: Parameters) extends LazyModule {
  val dtsdevice = new SimpleDevice("vitis_mpc", Seq("vitis_mpc"))

  val cfg_axi_node = AXI4SlaveNode(Seq(AXI4SlavePortParameters(
    slaves = Seq(AXI4SlaveParameters(
    address       = Seq(AddressSet(params.address, 4096-1)),
    resources     = dtsdevice.reg,
    regionType    = RegionType.UNCACHED,
    executable    = false,
    supportsWrite = TransferSizes(1, 32),
    supportsRead  = TransferSizes(1, 32),
    interleavedId = Some(0))),
  beatBytes = 4)))
  
  val cfg_tl_node = cfg_axi_node := AXI4Buffer () := LazyModule(new TLToAXI4()).node
  val int_node = IntSourceNode(IntSourcePortSimple(num = 1, resources = dtsdevice.int))

  lazy val module = new LazyModuleImp(this) {
    val bb_mpctracker = Module(new tracking(params))
    val (s, _) = cfg_axi_node.in(0)

    bb_mpctracker.io.ap_clk := clock
    bb_mpctracker.io.ap_rst_n := reset

    bb_mpctracker.io.s_axi_control.AWVALID := s.aw.bits.id
    s.aw.ready := bb_mpctracker.io.s_axi_control.AWREADY
    bb_mpctracker.io.s_axi_control.AWADDR := s.aw.bits.addr

    bb_mpctracker.io.s_axi_control.WVALID := s.w.valid
    s.w.ready := bb_mpctracker.io.s_axi_control.WREADY
    bb_mpctracker.io.s_axi_control.WDATA := s.w.bits.data
    bb_mpctracker.io.s_axi_control.WSTRB := s.w.bits.strb

    bb_mpctracker.io.s_axi_control.ARVALID := s.ar.valid
    s.ar.ready := bb_mpctracker.io.s_axi_control.ARREADY
    bb_mpctracker.io.s_axi_control.ARADDR := s.ar.bits.addr

    s.r.valid := bb_mpctracker.io.s_axi_control.RVALID
    bb_mpctracker.io.s_axi_control.RREADY := s.r.ready
    s.r.bits.data := bb_mpctracker.io.s_axi_control.RDATA
    s.r.bits.resp:= bb_mpctracker.io.s_axi_control.RRESP

    s.b.valid := bb_mpctracker.io.s_axi_control.BVALID
    bb_mpctracker.io.s_axi_control.BREADY := s.b.ready
    s.b.bits.resp := bb_mpctracker.io.s_axi_control.BRESP

    val (io_int, _) = int_node.out(0)
    io_int(0) := bb_mpctracker.io.interrupt
  }
}

class tracking(params: VitisMPCTrackerParams) extends BlackBox with HasBlackBoxPath {
  val io = IO(new Bundle {
    val ap_clk = Input(Clock())
    val ap_rst_n = Input(Bool())
    val s_axi_control = new Bundle {
        val AWVALID = Input(Bool())
        val AWREADY = Output(Bool())
        val AWADDR = Input(UInt(params.axi_control_addr_width.W))
        val WVALID = Input(Bool())
        val WREADY = Output(Bool())
        val WDATA = Input(UInt(params.axi_control_data_width.W))
        val WSTRB = Input(UInt(params.axi_control_wstrb_width.W))
        val ARVALID = Input(Bool())
        val ARREADY = Output(Bool())
        val ARADDR = Input(UInt(params.axi_control_addr_width.W))
        val RVALID = Output(Bool())
        val RREADY = Input(Bool())
        val RDATA = Output(UInt(params.axi_control_data_width.W))
        val RRESP = Output(UInt(2.W))
        val BVALID = Output(Bool())
        val BREADY = Input(Bool())
        val BRESP = Output(UInt(2.W))
    }
    val interrupt = Output(Bool())
  })
  val chipyardDir = System.getProperty("user.dir")
  val vitisVsrcDir = s"$chipyardDir/generators/vitis-mpc/src/main/resources/vsrc"

//   val makeStr = s"make -C $vitisVsrcDir default"
//   val preproc = makeStr
//   require (preproc.! == 0, "Failed to run pre-processing step")

  addPath(s"$vitisVsrcDir/vitis_tracking.preprocessed.v")
}

trait CanHavePeripheryVitisMPCTracker { this: BaseSubsystem =>
  
  private val pbus = locateTLBusWrapper(PBUS)

  p(VitisMPCTrackerKey).map { params =>

    val pbus = locateTLBusWrapper(PBUS)
    val vitismpcDomain = pbus.generateSynchronousDomain

    vitismpcDomain {
      val vitisMPCTracker = LazyModule(new VitisMPCTracker(params))
      pbus.coupleTo("vitismpctracker_cfg") {vitisMPCTracker.cfg_tl_node := TLFragmenter(4, pbus.blockBytes, holdFirstDeny = true) := TLWidthWidget(pbus.beatBytes) := _ }
      ibus.fromSync := vitisMPCTracker.int_node
    }
  }
}

class WithVitisMPCTracker(useAXI4: Boolean = false, useBlackBox: Boolean = false) extends Config((site, here, up) => {
  case VitisMPCTrackerKey => Some(VitisMPCTrackerParams())
})
