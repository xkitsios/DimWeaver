package io.github.xkitsios.dimweaver;

import me.lemire.integercompression.*;

import java.io.IOException;
import java.util.*;

/**
 * The binary format. Layout:
 * <pre>
 *   format version                    1 byte
 *   first timestamp                   VB
 *   last timestamp − first            VB
 *   dimension of each segment         VB byte count + bitmap (0: X, 1: Y), in storage order
 *   segments per group                VB word count + VB first value + Simple16 words
 *   segment start timestamps          VB word count + FastPFOR words     (only with ≥ 128 segments)
 *   per group: slope                  float
 *     per segment: intercept          double
 *                  start timestamp    VB                                 (only with &lt; 128 segments)
 * </pre>
 * Within a group, segments are sorted by start time and each start timestamp is stored as the difference from the
 * previous one in the group (the first one from the first timestamp of the trajectory).
 */
class Binary {
    /** First byte of every binary. Bump it whenever the layout changes; decode rejects other versions. */
    static final byte FORMAT_VERSION = 1;
    private static final int MIN_TIMESTAMPS_THRESHOLD = 128; // from this many segments on, timestamps go to FastPFOR
    // Simple16, VariableByte and Composition are stateless, so one shared instance is safe.
    private static final IntegerCODEC CODEC_Simple16 = new Composition(new Simple16(), new VariableByte());

    /**
     * A new FastPFOR codec per call: it keeps internal buffers, so a shared instance is not thread-safe, and leftovers
     * from previous calls end up in the padding of its output, making the binary depend on earlier calls. Creating
     * it costs ~3% of compression time.
     */
    private static IntegerCODEC fastPFOR128() {
        return new Composition(new FastPFOR128(), new VariableByte());
    }

    /**
     * Decoding reuses one FastPFOR codec per thread: creating one costs 15-30 µs, which dominated decompression,
     * and leftovers in its buffers do not affect what it decodes (unlike encoding, see {@link #fastPFOR128}).
     */
    private static final ThreadLocal<IntegerCODEC> FASTPFOR_DECODER = ThreadLocal.withInitial(Binary::fastPFOR128);

    /** Writes the dimension bitmap (byte count, then one bit per segment) and returns the position after it. */
    private static int dimensionWriter(boolean[] dimensions, byte[] buffer, int pos) {
        int dimBytes = (int) Math.ceil((double) dimensions.length / 8);
        pos = VariableByteEncoder.write(dimBytes, buffer, pos);
        for (int i = 0; i < dimensions.length; i++) {
            if (dimensions[i]) {
                int byteIndex = i / 8;
                int bitIndex = i % 8;
                buffer[pos + byteIndex] |= (byte) (1 << bitIndex);
            }
        }
        pos += dimBytes;

        return pos;
    }

    /** Reads the dimension bitmap; the result may have up to 7 unused trailing entries. */
    private static boolean[] dimensionReader(byte[] binary, int[] pos) {
        int dimensionBytes = readSize(binary, pos, binary.length);
        boolean[] dimensions = new boolean[dimensionBytes * 8];
        for (int i = 0; i < dimensions.length; i++) {
            int byteIndex = i / 8;
            int bitIndex = i % 8;
            dimensions[i] = ((binary[pos[0] + byteIndex] >> bitIndex) & 1) == 1;
        }
        pos[0] += dimensionBytes;

        return dimensions;
    }

    /**
     * Compresses with an output array large enough for incompressible input: the codecs may emit more words than
     * they consume (FastPFOR page/block metadata, VariableByte uses up to 5 bytes per int).
     */
    private static int[] compressInts(IntegerCODEC codec, int[] input) {
        int[] compressed = new int[input.length + input.length / 16 + 1024];
        IntWrapper outputOffset = new IntWrapper(0);
        codec.compress(input, new IntWrapper(0), input.length, compressed, outputOffset);
        return Arrays.copyOf(compressed, outputOffset.intValue());
    }

    /** Writes {@code words[from..]} as big-endian 4-byte words and returns the position after them. */
    private static int writeWords(int[] words, int from, byte[] buffer, int pos) {
        for (int i = from; i < words.length; i++) {
            int val = words[i];
            buffer[pos++] = (byte) (val >>> 24);
            buffer[pos++] = (byte) (val >>> 16);
            buffer[pos++] = (byte) (val >>> 8);
            buffer[pos++] = (byte) val;
        }

        return pos;
    }

    /** Writes the Simple16-compressed group sizes: word count, first word (the number of groups), remaining words. */
    /**
     * Reads a size from the binary and checks it is at most {@code limit}, so that a corrupted binary cannot make the
     * decoder allocate a huge array.
     */
    private static int readSize(byte[] buffer, int[] pos, int limit) {
        long size = VariableByteEncoder.read(buffer, pos);
        if (size < 0 || size > limit)
            throw new IllegalArgumentException("Invalid size " + size + " (at most " + limit + ")");
        return (int) size;
    }

    /** Reads one big-endian 4-byte word at {@code pos[0]} and advances it. */
    private static int readWord(byte[] buffer, int[] pos) {
        return ((buffer[pos[0]++] & 0xFF) << 24) | ((buffer[pos[0]++] & 0xFF) << 16) | ((buffer[pos[0]++] & 0xFF) << 8) | (buffer[pos[0]++] & 0xFF);
    }

    private static int countersWriter(int[] compressedCounters, byte[] buffer, int pos) {
        pos = VariableByteEncoder.write(compressedCounters.length, buffer, pos);
        pos = VariableByteEncoder.write(compressedCounters[0], buffer, pos);

        return writeWords(compressedCounters, 1, buffer, pos);
    }

    /** Reads and decompresses the group sizes written by {@link #countersWriter}. */
    private static int[] countersReader(byte[] buffer, int[] pos) {
        int compressedInt = readSize(buffer, pos, buffer.length);
        int uncompressedInt = readSize(buffer, pos, buffer.length);
        int[] intArray = new int[uncompressedInt + 1];
        intArray[0] = uncompressedInt;
        for (int i = 1; i < compressedInt; i++)
            intArray[i] = readWord(buffer, pos);
        int[] counters = new int[intArray[0]];
        IntWrapper recOffset = new IntWrapper(0);
        CODEC_Simple16.uncompress(intArray, new IntWrapper(0), intArray.length, counters, recOffset);

        return counters;
    }

    /** Writes the FastPFOR-compressed start timestamps: word count, then the words. */
    private static int timestampsWriter(int[] compressedTimestamps, byte[] buffer, int pos) {
        pos = VariableByteEncoder.write(compressedTimestamps.length, buffer, pos);

        return writeWords(compressedTimestamps, 0, buffer, pos);
    }

    /** Reads and decompresses {@code totalTimestamps} start timestamps written by {@link #timestampsWriter}. */
    private static int[] timestampsReader(byte[] buffer, int[] pos, int totalTimestamps) {
        int compressedInt = readSize(buffer, pos, buffer.length / 4);
        int[] intArray = new int[compressedInt];
        for (int i = 0; i < compressedInt; i++)
            intArray[i] = readWord(buffer, pos);
        int[] timestamps = new int[totalTimestamps];
        IntWrapper recOffset = new IntWrapper(0);
        FASTPFOR_DECODER.get().uncompress(intArray, new IntWrapper(0), intArray.length, timestamps, recOffset);

        return timestamps;
    }

    /**
     * Encodes the grouped segments in the layout described above. Each segment is stored with the group's slope and
     * the intercept of its line with that slope ({@link Segment#lineWithSlope}). Groups are written in the order
     * KinematicGrouping returns them, so the same input always gives the same bytes.
     *
     * @throws IllegalArgumentException if a timestamp offset does not fit in an int (trajectory span too long)
     */
    public static byte[] encode(List<KinematicGrouping.Group> groups, long firstTimestamp, long lastTimestamp) {
        int totalSegments = groups.stream().mapToInt(g -> g.segments().size()).sum();

        boolean vb = totalSegments < MIN_TIMESTAMPS_THRESHOLD;
        int[] timestamps = new int[totalSegments];
        int timestampsIdx = 0;

        // Dimension metadata
        boolean[] dimensions = new boolean[totalSegments];
        int dimIdx = 0;

        //Counters metadata
        int[] counters = new int[groups.size()];
        int countersIdx = 0;

        byte[] groupsBuffer = new byte[groups.size() * 4 + totalSegments * (8 + 10)]; // float slope; double b + VB long (<= 10 bytes)
        int groupsBufferPos = 0;
        for (KinematicGrouping.Group group : groups) { // in KinematicGrouping's fixed order
            double slope = group.slope(); // exactly representable as float
            List<Segment> segments = group.segments(); // sorted by start time
            counters[countersIdx++] = segments.size();
            groupsBufferPos = FloatEncoder.write((float) slope, groupsBuffer, groupsBufferPos);
            long previousT = 0L;
            for (Segment segment : segments) {
                groupsBufferPos = DoubleEncoder.write(segment.lineWithSlope(slope).getB(), groupsBuffer, groupsBufferPos);
                if (vb)
                    groupsBufferPos = VariableByteEncoder.write(segment.getStart() - previousT - firstTimestamp, groupsBuffer, groupsBufferPos);
                else {
                    // FastPFOR stores 32-bit ints: a larger offset is rejected instead of being silently corrupted
                    long offset = segment.getStart() - previousT - firstTimestamp;
                    if (offset > Integer.MAX_VALUE)
                        throw new IllegalArgumentException("Timestamp offset " + offset + " exceeds "
                                + Integer.MAX_VALUE + " time units; the trajectory span is too long for the current format");
                    timestamps[timestampsIdx++] = (int) offset;
                }
                previousT = segment.getStart() - firstTimestamp;
                dimensions[dimIdx++] = segment.getDim();
            }
        }

        // Create binary: compress first, then size the header buffer from the actual compressed lengths
        int[] compressedCounters = compressInts(CODEC_Simple16, counters);
        int[] compressedTimestamps = vb ? new int[0] : compressInts(fastPFOR128(), timestamps);
        int dimBytes = (dimensions.length + 7) / 8;
        int maxVB = 10; // VariableByteEncoder: at most 10 bytes per long
        byte[] buffer = new byte[1 + 2 * maxVB
                + maxVB + dimBytes
                + 2 * maxVB + compressedCounters.length * 4
                + maxVB + compressedTimestamps.length * 4];
        int pos = 0;

        buffer[pos++] = FORMAT_VERSION;
        pos = VariableByteEncoder.write(firstTimestamp, buffer, pos);
        pos = VariableByteEncoder.write(lastTimestamp - firstTimestamp, buffer, pos);
        pos = dimensionWriter(dimensions, buffer, pos);
        pos = countersWriter(compressedCounters, buffer, pos);
        if (!vb)
            pos = timestampsWriter(compressedTimestamps, buffer, pos);
        byte[] output = new byte[groupsBufferPos + pos];
        System.arraycopy(buffer, 0, output, 0, pos);
        System.arraycopy(groupsBuffer, 0, output, pos, groupsBufferPos);

        return output;
    }

    /**
     * Decodes a binary into the lines of each dimension, sorted by start time.
     *
     * @throws IOException if the first byte is not {@link #FORMAT_VERSION}, or the binary is truncated or corrupted
     */
    public static DecompressedTrajectory decode(byte[] binary) throws IOException {
        if (binary.length == 0 || binary[0] != FORMAT_VERSION)
            throw new IOException("Unsupported binary format version: "
                    + (binary.length == 0 ? "empty input" : Byte.toUnsignedInt(binary[0])) + " (expected " + FORMAT_VERSION + ")");
        try {
            return decodeChecked(binary);
        } catch (RuntimeException e) { // reading past the end, impossible sizes, ...
            throw new IOException("Corrupted or truncated DimWeaver binary", e);
        }
    }

    /** Decodes a binary whose version was checked; malformed input ends in an exception, never in a wrong result. */
    private static DecompressedTrajectory decodeChecked(byte[] binary) throws IOException {
        int[] pos = {0};
        List<Line> linesX = new ArrayList<>();
        List<Line> linesY = new ArrayList<>();
        int countersIdx = 0, dimIdx = 0;

        pos[0] = 1;
        long firstTimestamp = VariableByteEncoder.read(binary, pos);
        long lastTimestamp = firstTimestamp + VariableByteEncoder.read(binary, pos);
        boolean[] dimensions = dimensionReader(binary, pos);
        int[] counters = countersReader(binary, pos);
        int totalTimestamps = 0;
        for (int counter : counters)
            totalTimestamps = Math.addExact(totalTimestamps, counter);
        if (totalTimestamps > binary.length / 8 || totalTimestamps > dimensions.length) // each segment: ≥ 8 bytes, 1 bit
            throw new IOException("Corrupted DimWeaver binary: " + totalTimestamps + " segments in " + binary.length + " bytes");
        boolean vb = totalTimestamps < MIN_TIMESTAMPS_THRESHOLD;
        int[] timestamps = null;
        if (!vb)
            timestamps = timestampsReader(binary, pos, totalTimestamps);
        int timestampsIdx = 0;

        List<int[]> subgroupsX = new ArrayList<>();
        List<int[]> subgroupsY = new ArrayList<>();

        while (pos[0] < binary.length) { // one group per iteration
            double a = FloatEncoder.read(binary, pos);
            long previousT = 0L;

            int startX = linesX.size();
            int startY = linesY.size();

            for (int i = 0; i < counters[countersIdx]; i++) {
                double b = DoubleEncoder.read(binary, pos);
                long t;
                if (vb)
                    t = VariableByteEncoder.read(binary, pos) + previousT + firstTimestamp;
                else
                    t = timestamps[timestampsIdx++] + previousT + firstTimestamp;

                if (!dimensions[dimIdx])
                    linesX.add(new Line(t, a, b));
                else
                    linesY.add(new Line(t, a, b));
                dimIdx++;

                previousT = t - firstTimestamp;
            }

            int endX = linesX.size();
            int endY = linesY.size();
            subgroupsX.add(new int[]{startX, endX});
            subgroupsY.add(new int[]{startY, endY});

            countersIdx++;
        }

        if (countersIdx != counters.length)
            throw new IOException("Truncated DimWeaver binary: " + countersIdx + " of " + counters.length + " groups");

        // Merge using k-way merge
        linesX = mergeKWay(linesX, subgroupsX);
        linesY = mergeKWay(linesY, subgroupsY);

        return new DecompressedTrajectory(linesX, linesY, firstTimestamp, lastTimestamp);
    }

    /**
     * Merges the lines of all groups, each already sorted by start time, into one list sorted by start time.
     * {@code subgroups} holds the [start, end) range of each group in {@code lines}.
     */
    private static List<Line> mergeKWay(List<Line> lines, List<int[]> subgroups) {
        List<Line> result = new ArrayList<>(lines.size());
        PriorityQueue<Entry> pq = new PriorityQueue<>(Comparator.comparingDouble(e -> e.t));

        for (int i = 0; i < subgroups.size(); i++) {
            int[] range = subgroups.get(i);
            if (range[0] < range[1]) {
                Line line = lines.get(range[0]);
                pq.add(new Entry(line.getTStart(), range[0], range[1]));
            }
        }

        while (!pq.isEmpty()) {
            Entry e = pq.poll();
            result.add(lines.get(e.idx));
            int nextIdx = e.idx + 1;
            if (nextIdx < e.end) {
                e.idx = nextIdx;
                e.t = lines.get(nextIdx).getTStart();
                pq.add(e);
            }
        }

        return result;
    }

    /** Position of one group in the k-way merge: its next line and the start time of that line. */
    private static class Entry {
        double t;
        int idx;
        int end;

        Entry(double t, int idx, int end) {
            this.t = t;
            this.idx = idx;
            this.end = end;
        }
    }
}