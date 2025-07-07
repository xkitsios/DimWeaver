package io.github.xkitsios;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;

public class VariableByteEncoder {

    private static byte extract7bits(int i, long val) {
        return (byte) ((val >> (7 * i)) & 0x7F);
    }

    private static byte extract7bitsmaskless(int i, long val) {
        return (byte) (val >> (7 * i));
    }

    protected static void write(long number, ByteArrayOutputStream outputStream) {
        int i = 0;
        while ((number >> (7 * i)) >= 0x80) {
            outputStream.write(extract7bits(i, number));
            i++;
        }
        outputStream.write((byte) (extract7bitsmaskless(i, number) | 0x80)); // Last byte with MSB set
    }

    protected static long read(ByteArrayInputStream inputStream) {
        long number = 0;
        int shift = 0;
        int b;

        while (true) {
            b = inputStream.read();
            if (b == -1) throw new IllegalStateException("Unexpected end of input");
            number |= (long) (b & 0x7F) << shift;

            if ((b & 0x80) != 0) break;
            shift += 7;
        }

        return number;
    }
}
