package io.github.xkitsios;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;

class DoubleEncoder {
    protected static void write(double number, ByteArrayOutputStream outputStream) throws IOException {
        long longBits = Double.doubleToLongBits(number);
        LongEncoder.write(longBits, outputStream);
    }

    protected static double read(ByteArrayInputStream inputStream) throws IOException {
        long number = LongEncoder.read(inputStream);
        return Double.longBitsToDouble(number);
    }
}
