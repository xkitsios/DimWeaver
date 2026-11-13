package io.github.xkitsios.dimweaver;

/**
 * Variable-byte encoding of a non-negative long: 7 bits per byte, least significant first; the high bit marks the
 * last byte. At most 10 bytes.
 */
class VariableByteEncoder {
    /** Writes {@code number} at {@code pos} and returns the position after it. */
    public static int write(long number, byte[] buffer, int pos) {
        while ((number & ~0x7FL) != 0) {
            buffer[pos++] = (byte) (number & 0x7F);
            number >>>= 7;
        }
        buffer[pos++] = (byte) (number | 0x80);

        return pos; // new offset
    }

    /** Reads a value at {@code offsetHolder[0]} and advances it. */
    public static long read(byte[] buffer, int[] offsetHolder) {
        long result = 0;
        int shift = 0;
        byte b;

        while (true) {
            b = buffer[offsetHolder[0]++];
            result |= (long) (b & 0x7F) << shift;
            if ((b & 0x80) != 0) break;
            shift += 7;
        }

        return result;
    }
}
