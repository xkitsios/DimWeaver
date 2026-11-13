package io.github.xkitsios.dimweaver;

/** A double as 8 bytes, little-endian (the intercepts of the segments). */
class DoubleEncoder {
    /** Writes {@code number} at {@code pos} and returns the position after it. */
    public static int write(double number, byte[] buffer, int pos) {
        long longBits = Double.doubleToLongBits(number);
        for (int i = 0; i < 8; i++)
            buffer[pos++] = (byte) (longBits >>> (i * 8));

        return pos;
    }

    /** Reads a double at {@code offsetHolder[0]} and advances it. */
    public static double read(byte[] buffer, int[] offsetHolder) {
        long value = 0L;

        for (int i = 0; i < 8; i++)
            value |= ((long) (buffer[offsetHolder[0]++] & 0xFF)) << (i * 8);

        return Double.longBitsToDouble(value);
    }
}
