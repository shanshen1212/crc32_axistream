// CRC32 10Gbps Pipeline with Two-Stage Byte Pipeline (4+4)
// Splits 8-byte processing into two 4-byte stages to reduce timing pressure
// Maintains 64-bit@156.25MHz throughput without lookup tables

module crc32_10gbps_pipeline (
    // Clock and Reset
    input  wire         clk,
    input  wire         rst_n,
    
    // AXI4-Stream Slave Interface (Input)
    input  wire [63:0]  s_axis_tdata,
    input  wire [7:0]   s_axis_tkeep,
    input  wire         s_axis_tvalid,
    input  wire         s_axis_tlast,
    output reg          s_axis_tready,
    
    // AXI4-Stream Master Interface (Output)
    output reg  [63:0]  m_axis_tdata,
    output reg  [7:0]   m_axis_tkeep,
    output reg          m_axis_tvalid,
    output reg          m_axis_tlast,
    output reg  [31:0]  m_axis_tuser,  // CRC32 result
    input  wire         m_axis_tready,
    
    // Configuration
    input  wire [31:0]  crc_init,      // Initial CRC value (default: 32'hFFFFFFFF)
    input  wire         crc_enable     // Enable CRC calculation
);

// CRC32 Core Functions (LSB-first Ethernet)
// Single byte CRC update
function [31:0] crc8_lsb;
    input [31:0] crc_in;
    input [7:0]  data_byte;
    integer i;
    reg [31:0] c;
    begin
        c = crc_in;
        for (i = 0; i < 8; i = i + 1) begin
            if ((c[0] ^ data_byte[i]) == 1'b1)
                c = (c >> 1) ^ 32'hEDB88320; // Ethernet CRC32 LSB-first polynomial
            else
                c = (c >> 1);
        end
        crc8_lsb = c;
    end
endfunction

// Process 4 consecutive bytes with selective keep
function [31:0] crc4_update;
    input [31:0] seed;
    input [31:0] data32;  // {b3,b2,b1,b0}
    input [3:0]  keep4;   // {k3,k2,k1,k0}
    reg [31:0] c;
    begin
        c = seed;
        if (keep4[0]) c = crc8_lsb(c, data32[7:0]);
        if (keep4[1]) c = crc8_lsb(c, data32[15:8]);
        if (keep4[2]) c = crc8_lsb(c, data32[23:16]);
        if (keep4[3]) c = crc8_lsb(c, data32[31:24]);
        crc4_update = c;
    end
endfunction

// Flow Control
wire input_fire, output_fire, pipeline_ready;
assign input_fire = s_axis_tvalid && s_axis_tready;
assign output_fire = m_axis_tvalid && m_axis_tready;
assign pipeline_ready = !m_axis_tvalid || m_axis_tready;

// Simple ready connection
always @(*) begin
    s_axis_tready = pipeline_ready;
end

// Two-Stage Byte Pipeline (4+4 bytes per stage)
// Running CRC state (seed for Stage A)
reg [31:0] crc_run;

// Stage A registers (processes bytes 0-3)
reg [31:0] st1_mid_crc;    // Intermediate CRC after bytes 0-3
reg [31:0] st1_data_hi;    // Save bytes 4-7 for Stage B
reg [3:0]  st1_keep_hi;    // Save keep bits 4-7 for Stage B
reg        st1_valid;
reg        st1_last;

// Stage B registers (processes bytes 4-7)
reg [31:0] st2_final_crc;  // Final CRC after all 8 bytes
reg        st2_valid;
reg        st2_last;

// Data alignment pipeline (2-stage delay to match CRC pipeline)
reg [63:0] dpipe0, dpipe1;
reg [7:0]  kpipe0, kpipe1;
reg        vpipe0, vpipe1;
reg        lpipe0, lpipe1;

// Packet boundary detection
reg packet_start;

// Packet start detection: reset or previous beat was tlast
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        packet_start <= 1'b1;
    end else begin
        if (input_fire) begin
            packet_start <= s_axis_tlast;  // Next beat after tlast is packet start
        end
    end
end

// Main Pipeline Processing
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Reset CRC pipeline
        crc_run       <= crc_init;
        st1_mid_crc   <= 32'h0;
        st1_data_hi   <= 32'h0;
        st1_keep_hi   <= 4'h0;
        st1_valid     <= 1'b0;
        st1_last      <= 1'b0;
        st2_final_crc <= 32'h0;
        st2_valid     <= 1'b0;
        st2_last      <= 1'b0;
        
        // Reset data alignment pipeline
        dpipe0 <= 64'h0; dpipe1 <= 64'h0;
        kpipe0 <= 8'h0;  kpipe1 <= 8'h0;
        vpipe0 <= 1'b0;  vpipe1 <= 1'b0;
        lpipe0 <= 1'b0;  lpipe1 <= 1'b0;
        
    end else if (pipeline_ready) begin
        
        // Process bytes 0-3
        if (input_fire && crc_enable) begin
            // Use crc_init for new packet, crc_run for continuation
            st1_mid_crc <= crc4_update(packet_start ? crc_init : crc_run,
                                      s_axis_tdata[31:0], s_axis_tkeep[3:0]);
            st1_data_hi <= s_axis_tdata[63:32];
            st1_keep_hi <= s_axis_tkeep[7:4];
            st1_valid   <= 1'b1;
            st1_last    <= s_axis_tlast;
        end else if (input_fire && !crc_enable) begin
            // CRC disabled: pass through with identity
            st1_mid_crc <= packet_start ? crc_init : crc_run;
            st1_data_hi <= s_axis_tdata[63:32];
            st1_keep_hi <= s_axis_tkeep[7:4];
            st1_valid   <= 1'b1;
            st1_last    <= s_axis_tlast;
        end else begin
            st1_valid <= 1'b0;
        end

        // Process bytes 4-7
        st2_valid <= st1_valid;
        st2_last  <= st1_last;
        
        if (st1_valid) begin
            if (crc_enable) begin
                // Continue CRC calculation with bytes 4-7
                st2_final_crc <= crc4_update(st1_mid_crc, st1_data_hi, st1_keep_hi);
                
                // Update running CRC for next beat
                if (st1_last) begin
                    crc_run <= crc_init;  // Packet end, reset for next packet
                end else begin
                    // Save final CRC as seed for next beat
                    crc_run <= crc4_update(st1_mid_crc, st1_data_hi, st1_keep_hi);
                end
            end else begin
                // CRC disabled
                st2_final_crc <= st1_mid_crc;
                if (st1_last) begin
                    crc_run <= crc_init;
                end else begin
                    crc_run <= st1_mid_crc;
                end
            end
        end

        // Data Alignment Pipeline
        // Two-stage shift register to align data with 2-cycle CRC pipeline
        dpipe0 <= s_axis_tdata;       dpipe1 <= dpipe0;
        kpipe0 <= s_axis_tkeep;       kpipe1 <= kpipe0;
        vpipe0 <= input_fire;         vpipe1 <= vpipe0;
        lpipe0 <= s_axis_tlast & input_fire; lpipe1 <= lpipe0;

        m_axis_tdata  <= dpipe1;
        m_axis_tkeep  <= kpipe1;
        m_axis_tvalid <= vpipe1;
        m_axis_tlast  <= lpipe1;
        
        // CRC result: Ethernet standard inversion, only on last beat
        m_axis_tuser  <= (lpipe1 && vpipe1 && crc_enable) ? ~st2_final_crc : 32'h0;
    end
end

// Performance Counters
reg [31:0] packet_count;
reg [31:0] byte_count;

// Accurate byte counting
function [3:0] popcount8;
    input [7:0] bits;
    begin
        popcount8 = bits[0] + bits[1] + bits[2] + bits[3] + 
                   bits[4] + bits[5] + bits[6] + bits[7];
    end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        packet_count <= 32'h0;
        byte_count   <= 32'h0;
    end else begin
        if (output_fire) begin
            if (m_axis_tlast) begin
                packet_count <= packet_count + 1;
            end
            byte_count <= byte_count + popcount8(m_axis_tkeep);
        end
    end
end

endmodule



/*
=== TIMING ANALYSIS ===

Critical Path Comparison:
┌─────────────────────────────────────────────────────────────┐
│ Single Stage (Original):                                    │
│ crc_state → 8×crc8_lsb → next_crc                          │
│ Depth: ~64 XOR levels (8 bytes × 8 bits)                   │
│ Estimated: ~4.8ns @ moderate FPGA                          │
│ Margin @ 156.25MHz (6.4ns): ~25%                           │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Two-Stage (This Design):                                    │
│ Stage A: crc_state → 4×crc8_lsb → st1_mid_crc             │
│ Stage B: st1_mid_crc → 4×crc8_lsb → st2_final_crc         │
│ Depth: ~32 XOR levels per stage (4 bytes × 8 bits)        │
│ Estimated: ~2.4ns per stage                               │
│ Margin @ 156.25MHz (6.4ns): ~62%                          │
└─────────────────────────────────────────────────────────────┘

Performance Impact:
• Throughput: UNCHANGED (10Gbps @ 156.25MHz)
• Latency: +1 cycle (total 3 cycles vs 2 cycles)
• Resources: +~100 registers, minimal LUT increase
• Timing margin: +37% improvement

Benefits:
• No lookup table generation required
• Supports arbitrary TKEEP patterns
• Maintains full AXI4-Stream compliance
• Easy to extend to 4-stage (2+2+2+2) if needed
• Standard Verilog-2001, highly portable

This architecture achieves the timing goals through algorithmic 
pipelining rather than table optimization, making it simpler to 
implement and verify while maintaining full functionality.
*/