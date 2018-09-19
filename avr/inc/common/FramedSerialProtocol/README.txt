Framed Serial Protocol Info
-------------

Message Structure (bi-directional):
Segment				Length			Notes
------------------------------------------------
Frame start			1 byte (0x7e)
Length				1 byte			Length of packet from command to end of payload inclusive
Command				1 byte
Payload				0 - 254 bytes
Checksum			1 byte			8-bit sum of all unescaped bytes from command to end of payload inclusive subtracted from 0xFF.  I.e. if if the sum is 0x02, the checksum would be 0xFD.

Note that if any byte from Length to Checksum (inclusive) matches the Frame Start or Escape bytes, then it must be escaped.
Escaping is accomplished by writing the escape character (0x7d) followed by the byte which needs escaping XOR'd with 0x20.
