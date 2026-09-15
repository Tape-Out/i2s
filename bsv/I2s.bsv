package I2s;

// I2S 主机：一条规则分频出 SCK，下降沿发、上升沿收，照 UM11732（Rev. 3.0）3.1 与 3.2：
// 高位先出；接收端在上升沿锁存；WS = 0 是左声道；WS 在 MSB 之前一个时钟周期变化。

import RegIf::*;
import I2sRegs::*;

typedef struct {
  Bool rx;
} I2sCfg;

interface I2sPins;
  (* always_ready, result = "sck" *)  method Bit#(1) sck;
  (* always_ready, result = "ws" *)   method Bit#(1) ws;
  (* always_ready, result = "sd_o" *) method Bit#(1) sd_o;
  (* always_ready, always_enabled, prefix = "" *)
  method Action sd_in((* port = "sd_i" *) Bit#(1) v);
endinterface

interface I2sIfc#(numeric type aw, numeric type dw);
  interface RegIf#(aw, dw) regs;
  interface I2sPins         pins;
  (* always_ready *) method Bool irq;
endinterface

module mkI2s#(I2sCfg cfg)(I2sIfc#(aw, dw))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, 1, dw),
              Add#(_c, 6, dw), Add#(_d, 16, dw), Add#(_e, 32, dw));

  I2sRegsIfc#(aw, dw) r <- mkI2sRegs(I2sRegsCfg { rx: cfg.rx });

  // 恒零的只读寄存器：特性关掉时占位，综合器整片消掉
  function Reg#(t) roReg(t v) = interface Reg;
    method t _read = v;
    method Action _write(t x) = noAction;
  endinterface;

  Wire#(Bit#(1)) sdIn <- mkBypassWire;

  Reg#(Bit#(16)) cnt  <- mkReg(0);
  Reg#(Bit#(1))  sckR <- mkReg(0);
  Reg#(Bit#(1))  wsR  <- mkReg(0);
  Reg#(Bit#(1))  sdR  <- mkReg(0);
  Reg#(Bit#(1))  chan <- mkReg(1);     // 线上这个字属于哪个声道
  Reg#(UInt#(6)) bitn <- mkReg(31);    // 线上这一位是字里的第几位，0 是 MSB
  Reg#(Bit#(32)) txSh <- mkReg(0);     // 这个字还没发的位，靠高位对齐

  Reg#(Bit#(32)) rxSh = roReg(0);
  Reg#(Bit#(1))  rwsP = roReg(0);
  Reg#(Bit#(32)) rxlR = roReg(0);
  Reg#(Bit#(32)) rxrR = roReg(0);
  if (cfg.rx) begin
    rxSh <- mkReg(0);
    rwsP <- mkReg(0);
    rxlR <- mkReg(0);
    rxrR <- mkReg(0);
  end

  UInt#(6) len = unpack(r.ctrl_len);

  rule run;
    if (r.ctrl_en == 0) begin
      cnt  <= 0;
      sckR <= 0;
    // 判「小于」不判「不等于」：运行中把 div 改小，计数已经越过新值时，不等于要等 16 位回绕
    end else if (cnt < r.div) begin
      cnt <= cnt + 1;
    end else begin
      cnt <= 0;
      if (sckR == 0) begin
        // 上升沿：接收端锁存。WS 与上一次锁存的不同，这一位就是上一个字的 LSB
        sckR <= 1;
        if (cfg.rx) begin
          Bit#(32) a = {rxSh[30:0], sdIn};
          if (wsR != rwsP) begin
            if (rwsP == 0) rxlR <= a; else rxrR <= a;
            rxSh <= 0;
          end else
            rxSh <= a;
          rwsP <= wsR;
        end
      end else begin
        // 下降沿：发送端换数据。上一位是 LSB 就装下一个字；这一位是 LSB 就同拍翻转 WS，
        // WS 于是比下一个字的 MSB 早一个时钟周期。改小字长时位号可能已经越过 LSB，所以判「不小于」
        sckR <= 0;
        if (bitn + 1 >= len) begin
          Bit#(1)  nc = ~chan;
          Bit#(32) w  = ((nc == 0) ? r.txl : r.txr) << (32 - pack(len));
          sdR  <= w[31];
          txSh <= w << 1;
          bitn <= 0;
          chan <= nc;
          if (chan == 1) r.status_frame_set(1);
        end else begin
          sdR  <= txSh[31];
          txSh <= txSh << 1;
          bitn <= bitn + 1;
          if (bitn + 2 == len) wsR <= ~chan;
        end
      end
    end
  endrule

  rule show;
    r.rxl_in(rxlR);
    r.rxr_in(rxrR);
  endrule

  interface regs = r.regs;
  interface I2sPins pins;
    method Bit#(1) sck = sckR;
    method Bit#(1) ws = wsR;
    method Bit#(1) sd_o = sdR;
    method Action sd_in(Bit#(1) v); sdIn._write(v); endmethod
  endinterface
  method Bool irq = r.status_frame == 1 && r.ctrl_ien == 1;
endmodule

endpackage
