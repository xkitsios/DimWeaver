package io.github.xkitsios;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;

class LongEncoder {
    protected static void write(long number, ByteArrayOutputStream outputStream) throws IOException {
        ByteBuffer buffer = ByteBuffer.allocate(Long.BYTES);
        buffer.putLong(number);
        outputStream.write(buffer.array());
    }

    protected static long read(ByteArrayInputStream inputStream) throws IOException {
        byte[] byteArray = new byte[Long.BYTES];
        int k = inputStream.read(byteArray);
        if (k != Long.BYTES) throw new IOException();
        ByteBuffer buffer = ByteBuffer.allocate(Long.BYTES);
        buffer.put(byteArray);
        buffer.flip();

        return buffer.getLong();
    }
}
