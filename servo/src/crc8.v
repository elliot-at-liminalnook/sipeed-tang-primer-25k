// CRC-8, polynomial 0x07 (CRC-8/ATM), init 0x00, no reflection, no final XOR.
//
// Chosen because it is trivial on both sides: eight shift-and-XOR steps per
// byte in hardware, and about four lines of Python or C on the host. The
// servo bus uses a plain additive checksum, which catches single-byte
// corruption but happily accepts transposed bytes; the link carries position
// commands for a 30 kg.cm servo, so it gets something with better distance.
//
// This is a header, not a module: `include it and call the function.

function [7:0] crc8_byte(input [7:0] crc_in, input [7:0] data);
    integer i;
    reg [7:0] c;
    begin
        c = crc_in ^ data;
        for (i = 0; i < 8; i = i + 1)
            c = c[7] ? ((c << 1) ^ 8'h07) : (c << 1);
        crc8_byte = c;
    end
endfunction
