package io.github.xkitsios.dimweaver;

import io.github.xkitsios.dimweaver.utils.Validation;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class TestEdgeCases {
    private static final double EPSILON = 1e-4;

    private void assertRoundTrip(List<TrajectoryPoint> original) throws IOException {
        DecompressedTrajectory samples = DimWeaver.decompress(DimWeaver.compress(original, EPSILON, ErrorBound.SAMPLES));
        Validation.validateAtSamples(original, samples, EPSILON);

        DecompressedTrajectory continuous = DimWeaver.decompress(DimWeaver.compress(original, EPSILON, ErrorBound.CONTINUOUS));
        Validation.validateAtSamples(original, continuous, EPSILON);
        Validation.validateContinuous(original, continuous, EPSILON);
    }

    @Test
    public void defaultErrorBoundIsSamples() throws IOException {
        List<TrajectoryPoint> points = List.of(
                new TrajectoryPoint(0L, 0.0, 0.0), new TrajectoryPoint(10L, 1.0, 0.0),
                new TrajectoryPoint(20L, 1.0, 1.0), new TrajectoryPoint(30L, 0.0, 1.0));
        assertArrayEquals(DimWeaver.compress(points, EPSILON, ErrorBound.SAMPLES), DimWeaver.compress(points, EPSILON));
    }

    /**
     * AIS-like: sparse sampling (10 min) with sharp turns. Extending a segment's line over a 10-minute gap would
     * leave the ε-band around the interpolated trajectory; virtual points must prevent that.
     */
    @Test
    public void sparseSamplingWithTurns() throws IOException {
        List<TrajectoryPoint> points = new ArrayList<>();
        double x = 23.6, y = 37.9, heading = 0;
        for (int i = 0; i < 300; i++) {
            if (i % 7 == 0) heading += Math.PI / 2 + i % 3; // sharp turn
            x += 0.002 * Math.cos(heading);
            y += 0.002 * Math.sin(heading);
            points.add(new TrajectoryPoint(1_700_000_000L + i * 600L, x, y));
        }
        assertRoundTrip(points);
    }

    @Test
    public void singlePoint() throws IOException {
        assertRoundTrip(List.of(new TrajectoryPoint(1224730384L, 116.318417, 39.984702)));
    }

    @Test
    public void twoPoints() throws IOException {
        assertRoundTrip(List.of(
                new TrajectoryPoint(1224730384L, 116.318417, 39.984702),
                new TrajectoryPoint(1224730390L, 116.318450, 39.984683)));
    }

    /**
     * Sparse, random trajectories: almost every point starts a new segment with its own slope, and the timestamps
     * are large and incompressible. The compressed metadata then exceeds the uncompressed size, which used to
     * overflow the codec output arrays and the header buffer. The time span stays below 2^30 (int-safe).
     */
    @Test
    public void sparseIncompressibleTrajectories() throws IOException {
        for (int n : new int[]{50, 130, 200, 500, 2000}) {
            for (int seed = 0; seed < 5; seed++) {
                Random random = new Random(seed);
                long maxGap = (Integer.MAX_VALUE / 2) / n;
                List<TrajectoryPoint> points = new ArrayList<>();
                long t = 0;
                for (int i = 0; i < n; i++) {
                    t += 1 + (long) (random.nextDouble() * maxGap);
                    points.add(new TrajectoryPoint(t, random.nextDouble(), random.nextDouble()));
                }
                assertRoundTrip(points);
            }
        }
    }

    /**
     * Bug 2 (deferred): with >= 128 segments, timestamp offsets are stored as 32-bit ints. Until the format supports
     * longer spans, a span beyond Integer.MAX_VALUE time units must be rejected instead of silently corrupted.
     */
    @Test
    public void timestampSpanBeyondIntIsRejected() throws IOException {
        Random random = new Random(0);
        List<TrajectoryPoint> fits = new ArrayList<>(), tooLong = new ArrayList<>();
        for (int i = 0; i < 1000; i++) {
            double x = random.nextDouble(), y = random.nextDouble();
            fits.add(new TrajectoryPoint(i * 2_000_000L, x, y));      // span 2.0e9 < 2^31 - 1
            tooLong.add(new TrajectoryPoint(i * 3_000_000L, x, y));   // span 3.0e9 > 2^31 - 1
        }
        assertRoundTrip(fits);
        for (ErrorBound errorBound : ErrorBound.values())
            assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(tooLong, EPSILON, errorBound));
    }

    @Test
    public void formatVersionIsWrittenAndChecked() throws IOException {
        List<TrajectoryPoint> points = List.of(
                new TrajectoryPoint(0L, 0.0, 0.0), new TrajectoryPoint(10L, 1.0, 0.0), new TrajectoryPoint(20L, 1.0, 1.0));
        byte[] binary = DimWeaver.compress(points, EPSILON);
        assertEquals(Binary.FORMAT_VERSION, binary[0]);

        byte[] otherVersion = binary.clone();
        otherVersion[0] = (byte) (Binary.FORMAT_VERSION + 1);
        assertThrows(IOException.class, () -> DimWeaver.decompress(otherVersion));
        assertThrows(IOException.class, () -> DimWeaver.decompress(new byte[0]));
    }

    /** coordinatesAt gives the coordinates of evaluateAt; equal decompressions are equal; toString names the range. */
    @Test
    public void coordinatesAtEqualsAndToString() throws IOException {
        Random random = new Random(9);
        List<TrajectoryPoint> points = new ArrayList<>();
        for (int i = 0; i < 1_000; i++) points.add(new TrajectoryPoint(50L + 2L * i, random.nextDouble(), random.nextDouble()));
        long[] t = points.stream().mapToLong(TrajectoryPoint::t).toArray();
        byte[] binary = DimWeaver.compress(points, EPSILON);
        DecompressedTrajectory a = DimWeaver.decompress(binary), b = DimWeaver.decompress(binary.clone());
        double[][] coordinates = DimWeaver.coordinatesAt(a, t);
        List<TrajectoryPoint> evaluated = DimWeaver.evaluateAt(a, t);
        for (int i = 0; i < t.length; i++) {
            assertEquals(evaluated.get(i).x(), coordinates[0][i]);
            assertEquals(evaluated.get(i).y(), coordinates[1][i]);
        }
        assertEquals(a, b);
        assertEquals(a.hashCode(), b.hashCode());
        assertNotEquals(a, DimWeaver.decompress(DimWeaver.compress(points, 2 * EPSILON)));
        assertEquals("DecompressedTrajectory[firstTimestamp=50, lastTimestamp=2048, segments=" + a.segmentCount() + "]", a.toString());
    }

    /** Arrays give the same binary as a list of points; invalid arrays are rejected. */
    @Test
    public void arrays() {
        Random random = new Random(5);
        List<TrajectoryPoint> points = new ArrayList<>();
        for (int i = 0; i < 1_000; i++) points.add(new TrajectoryPoint(100L + 7L * i, random.nextDouble(), random.nextDouble()));
        long[] t = points.stream().mapToLong(TrajectoryPoint::t).toArray();
        double[] x = points.stream().mapToDouble(TrajectoryPoint::x).toArray();
        double[] y = points.stream().mapToDouble(TrajectoryPoint::y).toArray();
        assertArrayEquals(DimWeaver.compress(points, EPSILON), DimWeaver.compress(t, x, y, EPSILON));
        for (ErrorBound errorBound : ErrorBound.values())
            assertArrayEquals(DimWeaver.compress(points, EPSILON, errorBound), DimWeaver.compress(t, x, y, EPSILON, errorBound));
        assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(new long[]{1, 2}, new double[]{1}, new double[]{1, 2}, EPSILON));
        assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(null, x, y, EPSILON));
        assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(new long[0], new double[0], new double[0], EPSILON));
    }

    @Test
    public void emptyTrajectoryIsRejected() {
        assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(List.of(), EPSILON));
    }

    @Test
    public void invalidTimestampsAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(List.of(
                new TrajectoryPoint(-1L, 1.0, 1.0)), EPSILON));
        assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(List.of(
                new TrajectoryPoint(1L, 1.0, 1.0), new TrajectoryPoint(1L, 1.0, 1.0)), EPSILON));
        assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(List.of(
                new TrajectoryPoint(2L, 1.0, 1.0), new TrajectoryPoint(1L, 1.0, 1.0)), EPSILON));
    }

    @Test
    public void zeroTimestampIsAccepted() throws IOException {
        assertRoundTrip(List.of(new TrajectoryPoint(0L, 1.0, 1.0), new TrajectoryPoint(1L, 1.0, 1.0)));
    }

    @Test
    public void nonPositiveEpsilonIsRejected() {
        List<TrajectoryPoint> points = List.of(new TrajectoryPoint(1L, 1.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(points, 0));
        assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(points, -1e-4));
        assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(points, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(points, Double.POSITIVE_INFINITY));
    }

    @Test
    public void nonFiniteCoordinatesAreRejected() {
        for (double bad : new double[]{Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
            assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(List.of(
                    new TrajectoryPoint(1L, 1.0, 1.0), new TrajectoryPoint(2L, bad, 1.0)), EPSILON));
            assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(List.of(
                    new TrajectoryPoint(1L, 1.0, 1.0), new TrajectoryPoint(2L, 1.0, bad)), EPSILON));
        }
        List<TrajectoryPoint> withNull = new ArrayList<>();
        withNull.add(new TrajectoryPoint(1L, 1.0, 1.0));
        withNull.add(null);
        assertThrows(IllegalArgumentException.class, () -> DimWeaver.compress(withNull, EPSILON));
    }

    /** Every truncation of a binary is rejected with an IOException, never decoded into a wrong trajectory. */
    @Test
    public void truncatedBinaryIsRejected() {
        for (int n : new int[]{50, 500}) { // inline timestamps (< 128 segments) and FastPFOR timestamps
            Random random = new Random(n);
            List<TrajectoryPoint> points = new ArrayList<>();
            for (int i = 0; i < n; i++) points.add(new TrajectoryPoint(i * 10L, random.nextDouble(), random.nextDouble()));
            byte[] binary = DimWeaver.compress(points, EPSILON);
            for (int length = 0; length < binary.length; length++) {
                byte[] truncated = java.util.Arrays.copyOf(binary, length);
                assertThrows(IOException.class, () -> DimWeaver.decompress(truncated), "length " + length);
            }
        }
    }

    @Test
    public void positionAtMatchesEvaluateAt() throws IOException {
        Random random = new Random(7);
        List<TrajectoryPoint> points = new ArrayList<>();
        for (int i = 0; i < 300; i++) points.add(new TrajectoryPoint(1_000L + i * 7L, random.nextDouble(), random.nextDouble()));
        DecompressedTrajectory decompressed = DimWeaver.decompress(DimWeaver.compress(points, EPSILON));
        long[] times = new long[3_000];
        for (int i = 0; i < times.length; i++) times[i] = 900L + i; // also before the first and after the last point
        List<TrajectoryPoint> expected = DimWeaver.evaluateAt(decompressed, times);
        for (int i = 0; i < times.length; i++)
            assertEquals(expected.get(i), DimWeaver.positionAt(decompressed, times[i]));
    }
}
