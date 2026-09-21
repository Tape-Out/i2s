"""i2s 的行为测试台：SD 自环，按 UM11732（Rev. 3.0）拆线上的字。

判据不靠被测件自己的接收端——收发两头错得一样（比如都低位先出）自环照样对得上。测试台
另写一个解码器，只看线上的 SCK、WS、SD，照规范原文拆字：
  · 3.1：高位先出，接收端在 SCK 上升沿锁存
  · 3.2：WS = 0 是左声道、1 是右声道；WS 在 MSB 之前一个时钟周期变化
于是在某个上升沿第一次看到 WS 变了，这一位是上一个字的 LSB；下一位起是新字的 MSB。拆出来的
左右两个字、以及字长，必须等于写进 txl、txr 的低 len 位与 len。

另查三条线上的事：SCK 高低各 div + 1 拍（表 3 要各不少于 0.35T，这里是一半）；SD 与 WS 只在
SCK 下降沿那一拍变（表 3 的 tdtr 以上升沿为参照，下降沿变数据天然满足）；关掉 en，SCK 不再跳。

认矩阵：`rx` 开着再查被测件自己收到的 rxl、rxr。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
knobs = cfg.get("knobs", {})
rx_on = bool(knobs.get("rx", True))

CTRL, DIV, TXL, TXR, RXL, RXR, STATUS = 0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18
# 左声道最高位为一（二补码的负数），右声道为零，两个字的位图互不对称
LEFT, RIGHT = 0x9A5C3E71, 0x2D6B81F4


def mask(v, n):
    return v & ((1 << n) - 1)


def rounds():
    parts = []
    for n in (16, 24, 32):
        left, right = mask(LEFT, n), mask(RIGHT, n)
        rx = f"""
    rd(8'h{RXL:02X});
    action if (rdR != 32'h{left:08X}) begin $display("FAIL at len {n} rxl reads %08h, want {left:08x}", rdR); bad <= True; end endaction
    rd(8'h{RXR:02X});
    action if (rdR != 32'h{right:08X}) begin $display("FAIL at len {n} rxr reads %08h, want {right:08x}", rdR); bad <= True; end endaction""" if rx_on else ""
        parts.append(f"""
    // 字长 {n}
    wr(8'h{CTRL:02X}, 32'h{(n << 8) | 1:08X});
    delay(1200);
    action
      Bool wrong = False;
      if (gotL[1] != 32'h{left:08X}) begin $display("FAIL at len {n} the wire carries left %08h, want {left:08x}", gotL[1]); wrong = True; end
      if (gotR[1] != 32'h{right:08X}) begin $display("FAIL at len {n} the wire carries right %08h, want {right:08x}", gotR[1]); wrong = True; end
      if (gotN[1] != {n}) begin $display("FAIL at len {n} a word on the wire is %0d bits", gotN[1]); wrong = True; end
      if (wrong) bad <= True;
    endaction{rx}""")
    return "".join(parts)


verdict = ("words on the wire are MSB first with WS low for left and one clock of delay, at 16, 24 and 32 bits; "
           "SCK has an even duty cycle, SD and WS change only on the falling edge, the clock stops when disabled, "
           "and the frame flag drives irq" + (", and the receiver reads back both channels" if rx_on else ""))

TEMPLATE = r'''package I2s@L@Tb;

// 由 htest/mki2stb.py 生成，勿手改。这一点：rx=@RXON@

import StmtFSM::*;
import ConfigReg::*;
import RegIf::*;
import I2s::*;

(* synthesize *)
module mkI2s@L@Tb(Empty);
  I2sIfc#(8, 32) d <- mkI2s(I2sCfg { rx: @RX@ });

  rule loop;
    d.pins.sd_in(d.pins.sd_o);
  endrule

  // ---- 线上解码器：照 UM11732 3.1、3.2 拆字 ----
  Reg#(Bit#(1))  sckP <- mkReg(0);
  Reg#(Bit#(1))  sdP  <- mkReg(0);
  Reg#(Bit#(1))  wsP  <- mkReg(0);
  Reg#(Bit#(1))  mws  <- mkReg(0);
  Reg#(Bit#(32)) acc  <- mkReg(0);
  Reg#(UInt#(8)) n    <- mkReg(0);
  Reg#(Bit#(32)) gotL[2] <- mkCReg(2, 0);
  Reg#(Bit#(32)) gotR[2] <- mkCReg(2, 0);
  Reg#(UInt#(8)) gotN[2] <- mkCReg(2, 0);
  Reg#(UInt#(16)) hiRun <- mkReg(0);
  Reg#(UInt#(16)) loRun <- mkReg(0);
  Reg#(UInt#(16)) hiLen[2] <- mkCReg(2, 0);
  Reg#(UInt#(16)) loLen[2] <- mkCReg(2, 0);
  Reg#(Bool)     offEdge[2] <- mkCReg(2, False);
  Reg#(UInt#(32)) rises[2] <- mkCReg(2, 0);

  rule decode;
    Bit#(1) sck = d.pins.sck;
    Bit#(1) sd  = d.pins.sd_o;
    Bit#(1) ws  = d.pins.ws;
    sckP <= sck; sdP <= sd; wsP <= ws;
    Bool rise = sckP == 0 && sck == 1;
    Bool fall = sckP == 1 && sck == 0;
    // SD 与 WS 只许在 SCK 下降沿那一拍变
    if ((sd != sdP || ws != wsP) && !fall) offEdge[0] <= True;
    // 四个分支互斥：上升沿、下降沿、其余高电平、其余低电平各自计数
    if (rise) begin
      loLen[0] <= loRun; loRun <= 0; hiRun <= 1; rises[0] <= rises[0] + 1;
      Bit#(32) a = {acc[30:0], sd};
      if (ws != mws) begin
        if (mws == 0) gotL[0] <= a; else gotR[0] <= a;
        gotN[0] <= n + 1;
        acc <= 0; n <= 0; mws <= ws;
      end else begin
        acc <= a; n <= n + 1;
      end
    end else if (fall) begin
      hiLen[0] <= hiRun; hiRun <= 0; loRun <= 1;
    end else if (sck == 1) hiRun <= hiRun + 1;
    else loRun <= loRun + 1;
  endrule

  // ---- 命令序列 ----
  Reg#(Bool)      bad  <- mkReg(False);
  Reg#(Bit#(32))  rdR  <- mkReg(0);
  Reg#(UInt#(32)) mark <- mkReg(0);
  Reg#(UInt#(32)) cyc  <- mkConfigReg(0);

  function Action wr(Bit#(8) a, Bit#(32) v) = action
    let _ <- d.regs.access(RegReq { addr: a, write: True, wdata: v, wstrb: 4'hF });
  endaction;

  function Action rd(Bit#(8) a) = action
    let x <- d.regs.access(RegReq { addr: a, write: False, wdata: 0, wstrb: 4'hF });
    rdR <= x.rdata;
  endaction;

  Stmt test = seq
    wr(8'h@DIV@, 1);
    wr(8'h@TXL@, 32'h@LEFT@);
    wr(8'h@TXR@, 32'h@RIGHT@);
@ROUNDS@

    // SCK 高低各 div + 1 = 2 拍；SD、WS 从没在下降沿之外变过
    action
      Bool wrong = False;
      if (hiLen[1] != 2 || loLen[1] != 2) begin $display("FAIL SCK is high %0d and low %0d cycles, want 2 and 2", hiLen[1], loLen[1]); wrong = True; end
      if (offEdge[1]) begin $display("FAIL SD or WS changed away from a falling SCK edge"); wrong = True; end
      if (wrong) bad <= True;
    endaction

    // 运行中把分频改小：计数器已经越过新的 div 也不许卡住。计数在 SCK 翻转那一拍清零，
    // 所以对齐到一次上升沿再等两拍，div 为 7 时计数一定已经大于 1，这时改成 1。
    // 起初写的是「连着三次各晚一拍」，三次都碰巧落在计数 0 或 1 上，旧写法照样绿
    wr(8'h@DIV@, 7);
    delay(40);
    mark <= rises[1];
    await(rises[1] != mark);
    delay(2);
    wr(8'h@DIV@, 1);
    mark <= rises[1];
    delay(40);
    action if (rises[1] - mark < 5) begin $display("FAIL after div is lowered while running, SCK rises %0d times in 40 cycles", rises[1] - mark); bad <= True; end endaction

    // 帧标志与中断
    rd(8'h@STATUS@);
    action if (rdR[0] != 1) begin $display("FAIL the frame flag is not set while frames go out"); bad <= True; end endaction
    action if (d.irq) begin $display("FAIL irq rises with the frame flag set but ien off"); bad <= True; end endaction
    wr(8'h@CTRL@, 32'h00002003);
    action if (!d.irq) begin $display("FAIL irq stays low with the frame flag set and ien on"); bad <= True; end endaction

    // 关掉：SCK 不再跳
    wr(8'h@CTRL@, 32'h00002000);
    wr(8'h@STATUS@, 1);
    delay(8);
    mark <= rises[1];
    delay(200);
    action if (rises[1] != mark) begin $display("FAIL SCK still rises %0d times after en is cleared", rises[1] - mark); bad <= True; end endaction
    action if (d.irq) begin $display("FAIL irq stays high after the frame flag is cleared with the clock stopped"); bad <= True; end endaction
  endseq;

  FSM fsm <- mkFSM(test);
  Reg#(Bool) started <- mkReg(False);

  rule go (!started);
    started <= True;
    fsm.start;
  endrule

  rule tick_;
    cyc <= cyc + 1;
    if (cyc > 200000) begin
      $display("TIMEOUT");
      $finish(1);
    end
  endrule

  rule fin (started && fsm.done);
    if (bad) $display("FAILED");
    else $display("PASS i2s: @VERDICT@");
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
'''

txt = (TEMPLATE.replace("@L@", label)
       .replace("@RXON@", str(rx_on))
       .replace("@RX@", "True" if rx_on else "False")
       .replace("@ROUNDS@", rounds())
       .replace("@VERDICT@", verdict)
       .replace("@LEFT@", f"{LEFT:08X}").replace("@RIGHT@", f"{RIGHT:08X}")
       .replace("@CTRL@", f"{CTRL:02X}").replace("@DIV@", f"{DIV:02X}")
       .replace("@TXL@", f"{TXL:02X}").replace("@TXR@", f"{TXR:02X}")
       .replace("@STATUS@", f"{STATUS:02X}"))

(out / f"I2s{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  i2s 行为测试台就位：rx={rx_on}")
