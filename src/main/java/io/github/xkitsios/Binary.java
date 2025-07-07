package io.github.xkitsios;

import me.lemire.integercompression.*;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.*;
import java.util.stream.IntStream;

class Binary {
    private static final int MIN_TIMESTAMPS_THRESHOLD = 128;

    private static byte[] simple16Writer(List<Integer> data) {
        IntegerCODEC CODEC = new Composition(new Simple16(), new VariableByte());
        int[] intArray = data.stream().mapToInt(i -> i).toArray();
        int[] compressed = new int[intArray.length + 1];
        IntWrapper outputOffset = new IntWrapper(0);
        CODEC.compress(intArray, new IntWrapper(0), intArray.length, compressed, outputOffset);
        ByteBuffer byteBuffer = ByteBuffer.allocate(outputOffset.intValue() * Integer.BYTES);
        for (int i = 0; i < outputOffset.intValue(); i++)
            byteBuffer.putInt(compressed[i]);

        return byteBuffer.array();
    }

    private static int[] simple16Reader(ByteArrayInputStream is) {
        IntegerCODEC CODEC = new Composition(new Simple16(), new VariableByte());
        long size = VariableByteEncoder.read(is);
        byte[] byteArray = new byte[Math.toIntExact(size)];
        int k = is.read(byteArray, 0, (int) size);
        assert (k == byteArray.length);
        int[] intArray = new int[byteArray.length / Integer.BYTES];
        ByteBuffer byteBuffer = ByteBuffer.wrap(byteArray);
        for (int i = 0; i < intArray.length; i++)
            intArray[i] = byteBuffer.getInt();
        int[] data = new int[intArray[0]];
        IntWrapper recOffset = new IntWrapper(0);
        CODEC.uncompress(intArray, new IntWrapper(0), intArray.length, data, recOffset);

        return data;
    }

    private static byte[] fastPFOR128Writer(int[] intArray) {
        IntegerCODEC CODEC = new Composition(new FastPFOR128(), new VariableByte());
        int[] compressed = new int[intArray.length + 1];
        IntWrapper outputOffset = new IntWrapper(0);
        CODEC.compress(intArray, new IntWrapper(0), intArray.length, compressed, outputOffset);
        ByteBuffer byteBuffer = ByteBuffer.allocate(outputOffset.intValue() * Integer.BYTES);
        for (int i = 0; i < outputOffset.intValue(); i++)
            byteBuffer.putInt(compressed[i]);

        return byteBuffer.array();
    }

    private static int[] fastPFOR128Reader(ByteArrayInputStream is, int totalTimestamps) {
        IntegerCODEC CODEC = new Composition(new FastPFOR128(), new VariableByte());
        long size = VariableByteEncoder.read(is);
        byte[] byteArray = new byte[Math.toIntExact(size)];
        int k = is.read(byteArray, 0, (int) size);
        assert (k == byteArray.length);
        int[] intArray = new int[byteArray.length / Integer.BYTES];
        ByteBuffer byteBuffer = ByteBuffer.wrap(byteArray);
        for (int i = 0; i < intArray.length; i++)
            intArray[i] = byteBuffer.getInt();
        int[] data = new int[totalTimestamps];
        IntWrapper recOffset = new IntWrapper(0);
        CODEC.uncompress(intArray, new IntWrapper(0), intArray.length, data, recOffset);

        return data;
    }

    protected static byte[] encode(Map<Double, List<Segment>> groupedSegments, long smallestTimestamp) throws IOException {
        ByteArrayOutputStream segmentOS = new ByteArrayOutputStream();
        int dimIdx = 0;
        BitSet dim = new BitSet();
        List<Integer> counters = new ArrayList<>();
        int totalTimestamps = groupedSegments.values().stream().mapToInt(List::size).sum();
        boolean vb = totalTimestamps < MIN_TIMESTAMPS_THRESHOLD;
        int[] timestamps = new int[totalTimestamps];
        int index = 0;

        for (List<Segment> segments : groupedSegments.values()) {
            counters.add(segments.size());
            FloatEncoder.write((float) segments.getFirst().getMiddle().getA(), segmentOS);
            segments.sort(Comparator.comparingLong(Segment::getStart));
            long previousT = 0L;
            for (Segment segment : segments) {
                DoubleEncoder.write(segment.getMiddle().getB(), segmentOS);
                if (vb) VariableByteEncoder.write(segment.getStart() - previousT - smallestTimestamp, segmentOS);
                else timestamps[index++] = (int) (segment.getStart() - previousT - smallestTimestamp);
                previousT = segment.getStart() - smallestTimestamp;
                dim.set(dimIdx++, segment.getIsY());
            }
        }

        byte[] dimBytes = dim.toByteArray();
        byte[] countersBytes = simple16Writer(counters);
        ByteArrayOutputStream os = new ByteArrayOutputStream();
        VariableByteEncoder.write(smallestTimestamp, os);
        VariableByteEncoder.write(dimBytes.length, os);
        os.write(dimBytes);
        VariableByteEncoder.write(countersBytes.length, os);
        os.write(countersBytes);
        if (!vb) {
            byte[] timestampsBytes = fastPFOR128Writer(timestamps);
            VariableByteEncoder.write(timestampsBytes.length, os);
            os.write(timestampsBytes);
        }
        os.write(segmentOS.toByteArray());

        return os.toByteArray();
    }

    protected static List<Segment> decode(byte[] binary) throws IOException {
        List<Segment> segments = new ArrayList<>();
        int dimIdx = 0, countersIdx = 0, timestampsIdx = 0;

        ByteArrayInputStream is = new ByteArrayInputStream(binary);

        long smallestT = VariableByteEncoder.read(is);
        long dimLength = VariableByteEncoder.read(is);
        byte[] dimBytes = new byte[Math.toIntExact(dimLength)];
        int k = is.read(dimBytes);
        assert (k == dimBytes.length);
        BitSet dim = BitSet.valueOf(dimBytes);

        int[] counters = simple16Reader(is);
        int totalTimestamps = IntStream.of(counters).sum();
        boolean vb = totalTimestamps < MIN_TIMESTAMPS_THRESHOLD;
        int[] timestamps = null;
        if (!vb)
            timestamps = fastPFOR128Reader(is, totalTimestamps);
        while (is.available() > 0) {
            double a = FloatEncoder.read(is);
            long previousT = 0L;
            for (int i = 0; i < counters[countersIdx]; i++) {
                double b = DoubleEncoder.read(is);
                long t = vb ? VariableByteEncoder.read(is) + previousT + smallestT : timestamps[timestampsIdx++] + previousT + smallestT;
                segments.add(new Segment(t, new Line(t, a, b), dim.get(dimIdx++)));
                previousT = t - smallestT;
            }
            countersIdx++;
        }

        return segments;
    }
}
