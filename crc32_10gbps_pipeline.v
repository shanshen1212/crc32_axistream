// CRC32 10Gbps Pipeline Processor - Fixed Cross-Beat Linking
// Uses 64-bit parallel CRC core + alignment pipeline
// Supports proper multi-beat packet CRC calculation

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

// Flow control signals
wire input_fire, output_fire, pipeline_ready;
assign input_fire = s_axis_tvalid && s_axis_tready;
assign output_fire = m_axis_tvalid && m_axis_tready;
assign pipeline_ready = !m_axis_tvalid || m_axis_tready;

// Ready signal - not gated by crc_enable
always @(*) begin
    s_axis_tready = pipeline_ready;
end

// =====================================================
// 64-bit Parallel CRC32 Core (Combinational)
// =====================================================

// CRC32-Ethernet polynomial (LSB-first): 0xEDB88320
function [31:0] crc8_lsb;
    input [31:0] crc_in;
    input [7:0] data_byte;
    integer i;
    reg [31:0] crc_temp;
    begin
        crc_temp = crc_in;
        for (i = 0; i < 8; i = i + 1) begin
            if ((crc_temp[0] ^ data_byte[i]) == 1'b1)
                crc_temp = (crc_temp >> 1) ^ 32'hEDB88320;
            else
                crc_temp = crc_temp >> 1;
        end
        crc8_lsb = crc_temp;
    end
endfunction

// 64-bit parallel CRC update function
function [31:0] crc64_update;
    input [31:0] crc_seed;
    input [63:0] data_in;
    input [7:0]  keep_in;
    reg [31:0] crc_temp;
    begin
        crc_temp = crc_seed;
        
        // Process byte 0
        if (keep_in[0])
            crc_temp = crc8_lsb(crc_temp, data_in[7:0]);
        
        // Process byte 1
        if (keep_in[1])
            crc_temp = crc8_lsb(crc_temp, data_in[15:8]);
        
        // Process byte 2
        if (keep_in[2])
            crc_temp = crc8_lsb(crc_temp, data_in[23:16]);
        
        // Process byte 3
        if (keep_in[3])
            crc_temp = crc8_lsb(crc_temp, data_in[31:24]);
        
        // Process byte 4
        if (keep_in[4])
            crc_temp = crc8_lsb(crc_temp, data_in[39:32]);
        
        // Process byte 5
        if (keep_in[5])
            crc_temp = crc8_lsb(crc_temp, data_in[47:40]);
        
        // Process byte 6
        if (keep_in[6])
            crc_temp = crc8_lsb(crc_temp, data_in[55:48]);
        
        // Process byte 7
        if (keep_in[7])
            crc_temp = crc8_lsb(crc_temp, data_in[63:56]);
        
        crc64_update = crc_temp;
    end
endfunction

// =====================================================
// CRC State Management (Correct Cross-Beat Linking)
// =====================================================

reg [31:0] crc_state;        // Running CRC state
reg        packet_start;     // First beat of packet detection
wire [31:0] next_crc_comb;   // Next CRC after processing current beat

// Parallel CRC calculation (combinational)
assign next_crc_comb = crc_enable ? 
                       crc64_update(crc_state, s_axis_tdata, s_axis_tkeep) : 
                       crc_state;

// Packet start detection
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        packet_start <= 1'b1;
    end else begin
        if (input_fire) begin
            packet_start <= s_axis_tlast;  // Next beat after tlast is packet start
        end
    end
end

// CRC state management - CORRECT cross-beat linking
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        crc_state <= crc_init;
    end else begin
        if (input_fire) begin
            if (packet_start) begin
                // Initialize CRC for new packet, then update with current beat
                crc_state <= crc_enable ? 
                            crc64_update(crc_init, s_axis_tdata, s_axis_tkeep) :
                            crc_init;
            end else if (s_axis_tlast) begin
                // Last beat of packet - prepare for next packet
                crc_state <= crc_init;
            end else begin
                // Continue with calculated CRC for next beat
                crc_state <= next_crc_comb;
            end
        end
    end
end

// =====================================================
// 3-Stage Alignment Pipeline (Data + CRC Result)
// =====================================================

// Pipeline stage registers
reg [63:0]  pipe_data  [2:0];
reg [7:0]   pipe_keep  [2:0];
reg         pipe_valid [2:0];
reg         pipe_last  [2:0];
reg [31:0]  pipe_crc   [2:0];   // CRC results aligned with data

// Pipeline shift register
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Initialize all pipeline stages
        pipe_data[0]  <= 64'h0; pipe_data[1]  <= 64'h0; pipe_data[2]  <= 64'h0;
        pipe_keep[0]  <= 8'h0;  pipe_keep[1]  <= 8'h0;  pipe_keep[2]  <= 8'h0;
        pipe_valid[0] <= 1'b0;  pipe_valid[1] <= 1'b0;  pipe_valid[2] <= 1'b0;
        pipe_last[0]  <= 1'b0;  pipe_last[1]  <= 1'b0;  pipe_last[2]  <= 1'b0;
        pipe_crc[0]   <= 32'h0; pipe_crc[1]   <= 32'h0; pipe_crc[2]   <= 32'h0;
    end else begin
        if (pipeline_ready) begin
            // Shift pipeline stages
            pipe_data[2]  <= pipe_data[1];
            pipe_data[1]  <= pipe_data[0];
            pipe_data[0]  <= s_axis_tdata;
            
            pipe_keep[2]  <= pipe_keep[1];
            pipe_keep[1]  <= pipe_keep[0];
            pipe_keep[0]  <= s_axis_tkeep;
            
            pipe_valid[2] <= pipe_valid[1];
            pipe_valid[1] <= pipe_valid[0];
            pipe_valid[0] <= input_fire;
            
            pipe_last[2]  <= pipe_last[1];
            pipe_last[1]  <= pipe_last[0];
            pipe_last[0]  <= s_axis_tlast && input_fire;
            
            // Align CRC result with data
            pipe_crc[2]   <= pipe_crc[1];
            pipe_crc[1]   <= pipe_crc[0];
            pipe_crc[0]   <= next_crc_comb;  // CRC after processing this beat
        end
    end
end

// =====================================================
// Output Stage
// =====================================================

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        m_axis_tdata  <= 64'h0;
        m_axis_tkeep  <= 8'h0;
        m_axis_tvalid <= 1'b0;
        m_axis_tlast  <= 1'b0;
        m_axis_tuser  <= 32'h0;
    end else begin
        if (pipeline_ready) begin
            m_axis_tdata  <= pipe_data[2];
            m_axis_tkeep  <= pipe_keep[2];
            m_axis_tvalid <= pipe_valid[2];
            m_axis_tlast  <= pipe_last[2];
            // Final CRC result on last beat, inverted for Ethernet standard
            m_axis_tuser  <= (pipe_last[2] && pipe_valid[2] && crc_enable) ? 
                            ~pipe_crc[2] : 32'h0;
        end
    end
end

// =====================================================
// Performance Counters
// =====================================================

reg [31:0] packet_count;
reg [31:0] byte_count;

// Accurate byte counting using popcount
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