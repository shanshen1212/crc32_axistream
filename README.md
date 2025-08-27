# CRC32 10 Gb/s Pipeline (AXI4-Stream, 4+4 Byte Pipelined, No LUT)

A timing-friendly, **table-free** CRC-32 engine that sustains **64-bit @ 156.25 MHz** (≈ **10 Gb/s** line rate) on AXI4-Stream.  
Each 64-bit beat is processed in **two 4-byte stages (4+4)** to cut the critical path, so timing closure is much easier than a naïve single-cycle 8-byte CRC.

> **CRC flavor**: IEEE 802.3 / Ethernet reflected CRC-32  
> Polynomial (LSB-first): `0xEDB88320` (reflection of `0x04C11DB7`)  
> Seed: `0xFFFF_FFFF` · Final XOR: invert (`~crc`) · Per-byte bit order: **LSB-first**


---

## Features

- **Throughput**: one 64-bit beat **every clock** at 156.25 MHz (≈ 10 Gb/s).
- **Low timing pressure**: 8 bytes handled as **4+4** (two CRC stages).
- **Latency**: CRC pipeline **2 cycles**; CRC result is presented on the same cycle as the output `tlast`.
- **AXI4-Stream pass-through** with full `tvalid/tready` backpressure handling.
- **`tkeep`**: any pattern (contiguous or sparse) is supported.
- **No ROM/LUT tables**: portable Verilog-2001; simple to synthesize on any FPGA/ASIC.
- **Config**: `crc_init` (seed), `crc_enable` (bypass CRC computation without stalling data).


---

## Why Ethernet CRC?

- **Interoperability**: matches the most widely used 32-bit CRC in networking and storage (Ethernet, ZIP/PNG, many libraries).
- **Verification**: plenty of reference implementations and test vectors exist.
- **Hardware-friendly**: reflected (LSB-first) pipeline maps cleanly to iterative byte processing.


---


- **Stage A** processes bytes 0..3; **Stage B** processes bytes 4..7.  
- A two-stage **data/keep/valid/last** pipeline aligns the AXI stream with the 2-cycle CRC pipeline.  
- The CRC “running state” is carried across beats to ensure **correct cross-beat linking**.  
- On the output beat whose `tlast=1`, `m_axis_tuser` carries the **inverted** final CRC (Ethernet convention).  
  On non-last beats, `m_axis_tuser` is `32'h0`.


---

## AXI4-Stream Interface

### Slave (input)
- `s_axis_tdata [63:0]` — input data (byte 0 = `tdata[7:0]`).
- `s_axis_tkeep [7:0]` — per-byte valid mask (`keep[0]` ↔ `tdata[7:0]`).
- `s_axis_tvalid` — input beat is valid.
- `s_axis_tready` — core can accept a beat (follows output backpressure).
- `s_axis_tlast` — last beat of a packet.

### Master (output)
- `m_axis_tdata [63:0]` — pass-through data, aligned to CRC pipeline.
- `m_axis_tkeep [7:0]` — pass-through keep.
- `m_axis_tvalid` — output beat is valid.
- `m_axis_tready` — downstream ready.
- `m_axis_tlast` — last beat of the packet.
- `m_axis_tuser [31:0]` — **CRC32** of the packet (only valid on the last beat; otherwise 0).

### Configuration
- `crc_init [31:0]` — initial seed (recommend `32'hFFFF_FFFF`).
- `crc_enable` — when `0`, CRC is bypassed (stream still passes through).


---

## Timing & Throughput

- **Clock**: 156.25 MHz (6.4 ns period) typical for 10G-class streams.  
- **Throughput**: 64 b per cycle ⇒ ≈10.0 Gb/s sustained when `tvalid&tready` are continuously asserted.  
- **Critical path**: 4 × `crc8_lsb` per stage (vs 8 × `crc8_lsb` in a single-stage design).  
- **Latency**: 2 cycles CRC pipeline + pass-through alignment (the provided RTL aligns these so the CRC appears with the correct `tlast`).

> If your device still struggles at 156.25 MHz, the same structure scales:  
> – Drop to **78.125 MHz** for ≈5 Gb/s with zero RTL changes, or  
> – Further pipeline the byte sequence (e.g., **2+2+2+2**).


---

## `tkeep` & byte order

- `tkeep[i]` corresponds to `tdata[(8*i+7) : (8*i)]` (i.e., `keep[0]` ↔ LSB byte).
- Non-contiguous `tkeep` patterns are supported; inactive bytes are skipped in the CRC loop.
- The internal byte update is **LSB-first** per byte (required by the reflected polynomial).

**Examples**

| `tkeep`        | Valid bytes                              |
|----------------|-------------------------------------------|
| `8'b1111_1111` | all 8 bytes                               |
| `8'b0000_1111` | bytes 0..3 only                           |
| `8'b0101_0101` | bytes 0,2,4,6 (sparse)                    |
| `8'b0000_0001` | byte 0 only                               |


---

## How the CRC is produced

- The CRC state is initialized to `crc_init` at the **first beat of a packet**.  
- For each beat:
  - **Stage A** updates CRC with bytes 0..3 (respecting `tkeep[3:0]`).
  - **Stage B** continues with bytes 4..7 (respecting `tkeep[7:4]`).
  - The result becomes the **running CRC** for the next beat.
- On the beat where `tlast=1`, the core outputs `~CRC` on `m_axis_tuser` (Ethernet convention).  
  The running CRC is reset to `crc_init` for the next packet.


---

## Integration Notes

- **Reset**: `rst_n` is async active-low; deassert synchronously in your system where possible.
- **Backpressure**: `s_axis_tready` follows output availability; the core preserves order and doesn’t drop beats.
- **Bypass mode**: When `crc_enable=0`, the stream passes through; `m_axis_tuser` remains `0`.
- **Endianness**: The mapping is byte-wise as described; the algorithm itself is bit-reflected (LSB-first per byte).


---

## File List

- `crc32_10gbps_pipeline.v` — DUT (Verilog-2001 RTL, two-stage 4+4 pipeline).
- `crc32_10gbps_tb.v` — Verilog testbench (self-checking.

