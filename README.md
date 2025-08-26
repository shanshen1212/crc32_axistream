# CRC32 10Gbps Pipeline (AXI4-Stream)

A hardware module that computes **Ethernet CRC-32** over an **AXI4-Stream** at **10 Gb/s** using a **64-bit parallel core** plus a **3-stage alignment pipeline**.  
Consumes up to 8 bytes per beat @ 156.25 MHz and outputs the final CRC on the **last beat** via `m_axis_tuser[31:0]`. Data itself is transparently forwarded.

- CRC mode: RefIn=1, RefOut=1, **poly=0x04C11DB7** (reflected form **0xEDB88320**), **Init=crc_init** (default `0xFFFF_FFFF`), **XorOut=0xFFFF_FFFF**.
- Implementation: **LSB-first**, right-shift LFSR; per-beat update honors **`TKEEP`**.

---

## Features

- AXI4-Stream **slave** input and **master** output
- **64-bit** datapath @ 156.25 MHz (10 GbE line rate)
- Correct **cross-beat CRC chaining** for multi-beat packets
- `TKEEP`-aware (partial final beat & non-contiguous masks supported)
- **3-stage** alignment pipeline (clean timing to the master side)
- Proper backpressure handling (`TREADY` propagation)
- CRC enable bypass (`crc_enable=0` → transparent pass-through, `tuser=0`)

---

## Ports

### Clock & Reset
- `clk` — clock (posedge)
- `rst_n` — async reset, **active-low**

### AXI4-Stream Slave (input)
- `s_axis_tdata[63:0]` — data (byte0=`[7:0]` … byte7=`[63:56]`)
- `s_axis_tkeep[7:0]` — byte-valid mask; `TKEEP[i]` ↔ `TDATA[8*i+7:8*i]`
- `s_axis_tvalid` — input valid
- `s_axis_tlast` — last beat of packet
- `s_axis_tready` — module ready

### AXI4-Stream Master (output)
- `m_axis_tdata[63:0]` — forwarded data (delayed)
- `m_axis_tkeep[7:0]` — forwarded keep
- `m_axis_tvalid` — output valid
- `m_axis_tlast` — forwarded last
- `m_axis_tuser[31:0]` — **final CRC32** on the **last beat** (`~CRC`, Ethernet)
- `m_axis_tready` — downstream ready

### Configuration
- `crc_init[31:0]` — CRC init per packet (default `32'hFFFF_FFFF`)
- `crc_enable` — 1: compute/output CRC; 0: bypass (no CRC updates)

---

## How It Works

### Handshake & Flow
- A beat **fires** when `TVALID && TREADY` is high.
- `s_axis_tready = (!m_axis_tvalid) || m_axis_tready` to propagate backpressure.

### Per-Beat CRC
- Running state: `crc_state`.
- Combinational next:  
  ```systemverilog
  next_crc = crc64_update(crc_state, s_axis_tdata, s_axis_tkeep);
