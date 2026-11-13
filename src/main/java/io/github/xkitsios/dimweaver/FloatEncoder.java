package io.github.xkitsios.dimweaver;

/** A float as 4 bytes, little-endian (the slopes of the groups). */
class FloatEncoder {
    /** Writes {@code number} at {@code pos} and returns the position after it. */
    public static int write(float number, byte[] buffer, int pos) {
        int intBits = Float.floatToIntBits(number);
        for (int i = 0; i < 4; i++)
            buffer[pos++] = (byte) (intBits >>> (i * 8));

        return pos;
    }

    /** Reads a float at {@code offsetHolder[0]} and advances it. */
    public static float read(byte[] buffer, int[] offsetHolder) {
        int value = 0;

        for (int i = 0; i < 4; i++)
            value |= (buffer[offsetHolder[0]++] & 0xFF) << (i * 8);

        return Float.intBitsToFloat(value);
    }
}
