# i2s

I2S controller.

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Simulated. The controller is the bus master: it generates SCK and WS and sends the stereo frame in `txl` and `txr` over and over, 16, 24 or 32 bits a word. The word format follows sections 3.1 and 3.2 of NXP UM11732, *I2S bus specification*, Rev. 3.0: MSB first, WS low for the left channel and changing one clock before the MSB, data changing on the falling edge of SCK and latched on the rising edge. With `rx` on it also samples SD on the rising edge and keeps the last left and right words in `rxl` and `rxr`.

The whole IP is one rule in Bluespec SystemVerilog. What makes it right is the cycle on which SCK toggles, data changes and data is latched; there is no algorithm in it worth writing in Bluespec Haskell.

The testbench loops SD back and decodes the wire with a decoder of its own that looks only at SCK, WS and SD, so a sender and a receiver that are wrong in the same way cannot pass together. It checks the left and right words and the word length at 16, 24 and 32 bits, the received words when `rx` is on, equal high and low SCK times, SD and WS changing only on a falling SCK, the frame flag and the interrupt against `ien`, SCK stopping once `en` is cleared, and SCK still running after `div` is lowered while the divider has already counted past the new value.

| `rx` | off | on |
| :--: | --: | --: |
| Area, um2 | 2697 | 3803 |

## Registers

| Offset | Register | Fields |
| :--: | :-- | :-- |
| 0x00 | `ctrl` | `en`, `ien`, `len` (16, 24 or 32; a write of any other value keeps the old length) |
| 0x04 | `div` | SCK half period in clock cycles, minus one (3 at reset) |
| 0x08 | `txl` | left sample to send, the low `len` bits |
| 0x0C | `txr` | right sample to send, the low `len` bits |
| 0x10 | `rxl` | last left sample received (with `rx`) |
| 0x14 | `rxr` | last right sample received (with `rx`) |
| 0x18 | `status` | `frame`: both channels of a frame have gone out (write 1 to clear) |

Pins are `sck`, `ws` and `sd_o` out and `sd_i` in. Slave mode, where SCK and WS come from outside, is not implemented, and neither are TDM, the left- and right-justified formats or MCLK.

## License

Mulan PSL v2.
