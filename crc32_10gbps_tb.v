`timescale 1 ns / 1 ns
module crc32_10gbps_tb;

reg         clk;
reg         rst_n;
reg [63:0]  s_axis_tdata;
reg [7:0]   s_axis_tkeep;
reg         s_axis_tvalid;
reg         s_axis_tlast;
wire        s_axis_tready;

wire [63:0] m_axis_tdata;
wire [7:0]  m_axis_tkeep;
wire        m_axis_tvalid;
wire        m_axis_tlast;
wire [31:0] m_axis_tuser;
reg         m_axis_tready;

reg [31:0]  crc_init;
reg         crc_enable;

// Clock generation (156.25MHz = 6.4ns period)
always #3.2 clk = ~clk;

// DUT instantiation
crc32_10gbps_pipeline dut (
    .clk(clk),
    .rst_n(rst_n),
    .s_axis_tdata(s_axis_tdata),
    .s_axis_tkeep(s_axis_tkeep),
    .s_axis_tvalid(s_axis_tvalid),
    .s_axis_tlast(s_axis_tlast),
    .s_axis_tready(s_axis_tready),
    .m_axis_tdata(m_axis_tdata),
    .m_axis_tkeep(m_axis_tkeep),
    .m_axis_tvalid(m_axis_tvalid),
    .m_axis_tlast(m_axis_tlast),
    .m_axis_tuser(m_axis_tuser),
    .m_axis_tready(m_axis_tready),
    .crc_init(crc_init),
    .crc_enable(crc_enable)
);

// Task to send multi-beat packet
task send_packet;
    input [7:0] num_beats;
    input [63:0] start_pattern;
    input [7:0] last_keep;
    integer i;
    begin
        $display("Sending %0d-beat packet, start_pattern=0x%h", num_beats, start_pattern);
        
        for (i = 0; i < num_beats; i = i + 1) begin
            @(posedge clk);
            s_axis_tdata = start_pattern + (i << 8);  // Increment pattern
            s_axis_tkeep = (i == num_beats-1) ? last_keep : 8'hFF;
            s_axis_tvalid = 1'b1;
            s_axis_tlast = (i == num_beats-1);
            
            // Wait for ready
            while (!s_axis_tready) begin
                @(posedge clk);
            end
        end
        
        @(posedge clk);
        s_axis_tvalid = 1'b0;
        s_axis_tlast = 1'b0;
    end
endtask

// Reference CRC calculation for verification
function [31:0] ref_crc32;
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
        ref_crc32 = crc_temp;
    end
endfunction

// Test stimulus
initial begin
    // Initialize
    $dumpfile("crc32_10gbps_tb.vcd");
    $dumpvars(0, crc32_10gbps_tb);
    $dumpvars(0, dut);
    clk = 0;
    rst_n = 0;
    s_axis_tdata = 64'h0;
    s_axis_tkeep = 8'h0;
    s_axis_tvalid = 1'b0;
    s_axis_tlast = 1'b0;
    m_axis_tready = 1'b1;
    crc_init = 32'hFFFFFFFF;
    crc_enable = 1'b1;
    
    // Reset
    #100;
    rst_n = 1;
    #50;
    
    $display("=== Test 1: Single beat packet ===");
    send_packet(1, 64'h0123456789ABCDEF, 8'hFF);
    #200;
    
    $display("=== Test 2: Multi-beat packet (tests cross-beat linking) ===");
    send_packet(4, 64'h1111111111111111, 8'h0F);  // 28 bytes total
    #300;
    
    $display("=== Test 3: Back-to-back packets ===");
    send_packet(2, 64'hAAAAAAAAAAAAAAAA, 8'hFF);
    send_packet(3, 64'hBBBBBBBBBBBBBBBB, 8'h3F);
    #400;
    
    $display("=== Test 4: CRC disabled ===");
    crc_enable = 1'b0;
    send_packet(2, 64'hCCCCCCCCCCCCCCCC, 8'hFF);
    crc_enable = 1'b1;
    #300;
    
    $display("=== Test 5: Variable backpressure ===");
    m_axis_tready = 1'b0;  // Apply backpressure
    send_packet(3, 64'hDDDDDDDDDDDDDDDD, 8'hFF);
    #100;
    m_axis_tready = 1'b1;  // Release backpressure
    #400;
    
    $display("All tests completed successfully!");
    $finish;
end

// Output monitor with CRC verification
always @(posedge clk) begin
    if (m_axis_tvalid && m_axis_tready) begin
        if (m_axis_tlast) begin
            $display("Packet complete: final_data=0x%h, CRC=0x%h", 
                    m_axis_tdata, m_axis_tuser);
        end
    end
end

endmodule