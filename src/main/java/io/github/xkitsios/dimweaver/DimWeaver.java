package io.github.xkitsios.dimweaver;

import java.io.IOException;
import java.util.*;

/**
 * DimWeaver: error-bounded lossy compression of 2D trajectories.
 *
 * <pre>{@code
 * byte[] binary = DimWeaver.compress(points, epsilon);                    // ε at the original samples
 * byte[] strict = DimWeaver.compress(points, epsilon, ErrorBound.CONTINUOUS); // ε at every time
 * DecompressedTrajectory d = DimWeaver.decompress(binary);
 * List<TrajectoryPoint> atSamples = DimWeaver.evaluateAt(d, timestamps);
 * TrajectoryPoint at = DimWeaver.positionAt(d, t);
 * List<TrajectoryPoint> vertices = DimWeaver.reconstructTrajectory(d);
 * }</pre>
 *
 * The error is the Euclidean distance in the units of x and y (for example meters, or degrees for lon/lat).
 * Timestamps are integers in any unit (seconds, milliseconds, ...), non-negative and strictly increasing.
 * All methods are thread-safe.
 */
public final class DimWeaver {
    private DimWeaver() {
    }

    /**
     * Decay factor of the exponential average of segment sizes that chooses the primary dimension. Fixed: the
     * compression ratio is almost insensitive to it.
     */
    static final double C = 0.85;

    /**
     * Compresses with the error bound guaranteed at the original samples ({@link ErrorBound#SAMPLES}).
     *
     * @param points  the trajectory: at least one point, timestamps non-negative and strictly increasing
     * @param epsilon the maximum error (&gt; 0), in the units of x and y
     * @return the compressed binary
     * @throws IllegalArgumentException if the points (null, NaN or infinite coordinates, timestamps) or epsilon are
     *                                  invalid, or the trajectory spans more than
     *                                  2<sup>31</sup> - 1 time units
     */
    public static byte[] compress(List<TrajectoryPoint> points, double epsilon) {
        return compress(points, epsilon, ErrorBound.SAMPLES);
    }

    /**
     * Compresses with the error bound guaranteed where {@code errorBound} says.
     *
     * @param points     the trajectory: at least one point, timestamps non-negative and strictly increasing
     * @param epsilon    the maximum error (&gt; 0), in the units of x and y
     * @param errorBound {@link ErrorBound#SAMPLES} or {@link ErrorBound#CONTINUOUS}
     * @return the compressed binary
     * @throws IllegalArgumentException if the arguments are invalid, or the trajectory spans more than
     *                                  2<sup>31</sup> - 1 time units
     */
    public static byte[] compress(List<TrajectoryPoint> points, double epsilon, ErrorBound errorBound) {
        return Binary.encode(KinematicGrouping.group(dimWeaver(points, epsilon, errorBound)), points.get(0).t(), points.get(points.size() - 1).t());
    }

    /**
     * Compresses a trajectory given as arrays, with the error bound guaranteed at the original samples.
     *
     * @param t       timestamps: at least one, non-negative and strictly increasing
     * @param x       first coordinate, one per timestamp
     * @param y       second coordinate, one per timestamp
     * @param epsilon the maximum error (&gt; 0), in the units of x and y
     * @return the compressed binary
     * @throws IllegalArgumentException as {@link #compress(List, double)}, or if the arrays have different lengths
     */
    public static byte[] compress(long[] t, double[] x, double[] y, double epsilon) {
        return compress(t, x, y, epsilon, ErrorBound.SAMPLES);
    }

    /**
     * Compresses a trajectory given as arrays, with the error bound guaranteed where {@code errorBound} says.
     *
     * @param t the time
     * @param x first coordinate, one per timestamp
     * @param y second coordinate, one per timestamp
     * @param epsilon the maximum error (&gt; 0)
     * @param errorBound {@link ErrorBound#SAMPLES} or {@link ErrorBound#CONTINUOUS}
     * @return the binary
     * @throws IllegalArgumentException as {@link #compress(long[], double[], double[], double)}
     */
    public static byte[] compress(long[] t, double[] x, double[] y, double epsilon, ErrorBound errorBound) {
        if (t == null || x == null || y == null)
            throw new IllegalArgumentException("Arrays must not be null");
        if (t.length != x.length || t.length != y.length)
            throw new IllegalArgumentException("t, x and y must have the same length, got " + t.length + ", " + x.length + " and " + y.length);
        List<TrajectoryPoint> points = new ArrayList<>(t.length);
        for (int i = 0; i < t.length; i++) points.add(new TrajectoryPoint(t[i], x[i], y[i]));
        return compress(points, epsilon, errorBound);
    }

    /**
     * Decompresses a binary produced by {@code compress}.
     *
     * @param binary a binary produced by {@code compress}
     * @return the decompressed trajectory
     * @throws IOException if the binary is not a DimWeaver binary of this format version
     */
    public static DecompressedTrajectory decompress(byte[] binary) throws IOException {
        return Binary.decode(binary);
    }

    /**
     * The segments of both dimensions, before grouping and encoding: checks the input and feeds the points, in
     * order, to the {@link Weaver}.
     */
    static List<Segment> dimWeaver(List<TrajectoryPoint> points, double epsilon, ErrorBound errorBound) {
        if (points == null || points.isEmpty())
            throw new IllegalArgumentException("Trajectory must contain at least one point");
        if (!(epsilon > 0) || epsilon == Double.POSITIVE_INFINITY)
            throw new IllegalArgumentException("Epsilon must be finite and > 0, got " + epsilon);
        if (errorBound == null)
            throw new IllegalArgumentException("Error bound must not be null");
        Weaver weaver = new Weaver(epsilon, C, errorBound == ErrorBound.CONTINUOUS);
        long previousT = -1; // -1 so that the same check also rejects negative timestamps
        for (TrajectoryPoint point : points) {
            if (point == null)
                throw new IllegalArgumentException("Points must not be null");
            if (point.t() <= previousT)
                throw new IllegalArgumentException(point.t() < 0
                        ? "Timestamps must be non-negative, got " + point.t()
                        : "Timestamps must be strictly increasing, got " + point.t() + " after " + previousT);
            if (!Double.isFinite(point.x()) || !Double.isFinite(point.y()))
                throw new IllegalArgumentException("Coordinates must be finite, got " + point);
            previousT = point.t();
            weaver.feed(point);
        }

        return weaver.finish();
    }

    /**
     * Weaves the two dimensions. One dimension is primary and is compressed with tolerance ε/√2; when its segment
     * ends, the points it covered are given to the other (secondary) dimension with the residual tolerance
     * √(ε² − err²), err being the primary's error at that point, so that the L2 error stays within ε.
     * <p>
     * Gaps ({@link ErrorBound#CONTINUOUS} only): decompression uses each segment until the next one starts, also
     * between samples. Hence a segment must stay
     * within tolerance of the linear interpolation between samples up to the time right before the next segment starts. When a
     * segment cannot absorb the next sample p, it is extended with a virtual point on that interpolation at the latest time t*
     * it still fits ({@link DisjointPLA#latestFittingTime}), and the next segment starts with a virtual point at t* + 1.
     * Virtual points are processed like real ones in both dimensions, so the error bound holds at every time with
     * respect to the linearly interpolated trajectory, and the binary needs no extra information.
     */
    private static final class Weaver {
        private final double epsilon;
        private final double primaryTolerance;
        private final double c;
        private final boolean continuous; // ErrorBound.CONTINUOUS: use virtual points at segment boundaries
        private final DisjointPLA plaX = new DisjointPLA(false), plaY = new DisjointPLA(true);
        private final List<TrajectoryPoint> buffer = new ArrayList<>(); // points of the open primary segment, not yet given to the secondary
        private double exponentialDecayX = 0, exponentialDecayY = 0; // average segment size (points) per dimension
        private boolean residualY = false; // true: residual@Y (X primary), false: residual@X (Y primary)
        private TrajectoryPoint lastPoint; // last point added to the primary
        private TrajectoryPoint lastSecondaryPoint; // last point added to the secondary
        private double lastSecondaryTolerance; // and its tolerance

        private Weaver(double epsilon, double c, boolean continuous) {
            this.epsilon = epsilon;
            this.primaryTolerance = epsilon / Math.sqrt(2);
            this.c = c;
            this.continuous = continuous;
        }

        /** The dimension currently compressed with ε/√2. */
        private DisjointPLA primary() {
            return residualY ? plaX : plaY;
        }

        /** The dimension currently compressed with the residual tolerance. */
        private DisjointPLA secondary() {
            return residualY ? plaY : plaX;
        }

        /** The point's coordinate in the primary dimension. */
        private TimeSeriesPoint inPrimary(TrajectoryPoint point) {
            return new TimeSeriesPoint(point.t(), residualY ? point.x() : point.y());
        }

        /** The point's coordinate in the secondary dimension. */
        private TimeSeriesPoint inSecondary(TrajectoryPoint point) {
            return new TimeSeriesPoint(point.t(), residualY ? point.y() : point.x());
        }

        /**
         * Updates the exponential average of the segment size of a dimension (x: true for X) with a segment that
         * just closed. The dimension with the larger average, i.e. the one that compresses better, becomes primary.
         */
        private void updateDecay(boolean x, int segmentSize) {
            if (x) exponentialDecayX = exponentialDecayX * (1 - c) + segmentSize * c;
            else exponentialDecayY = exponentialDecayY * (1 - c) + segmentSize * c;
        }

        /** Tolerance left for the secondary dimension at this point, given the primary segment that covers it. */
        private double residual(Segment primarySegment, TrajectoryPoint point) {
            return calculateResidual(primarySegment, inPrimary(point), epsilon);
        }

        /**
         * Adds a point to the primary dimension. If it does not fit, the primary segment is closed (with
         * CONTINUOUS, after extending it with virtual points), its points are given to the secondary dimension, and
         * the point is fed again, possibly to the other dimension if the primary changed.
         */
        private void feed(TrajectoryPoint point) {
            if (primary().add(inPrimary(point), primaryTolerance)) {
                buffer.add(point);
                lastPoint = point;
                return;
            }
            if (!continuous) {
                closePrimary();
                feed(point);
                return;
            }
            // The primary segment ends between lastPoint and point: extend it along the interpolation as far as it fits
            TrajectoryPoint end = lastPoint;
            long t = primary().latestFittingTime(inPrimary(point), primaryTolerance, primaryTolerance);
            if (t > end.t()) {
                TrajectoryPoint virtual = interpolate(end, point, t);
                if (primary().add(inPrimary(virtual), primaryTolerance)) {
                    buffer.add(virtual);
                    end = virtual;
                }
            }
            closePrimary();
            lastPoint = end;
            // The next primary segment starts right after the end of this one
            if (point.t() - end.t() > 1) {
                feed(interpolate(end, point, end.t() + 1));
            }
            feed(point);
        }

        /**
         * Closes the primary segment, gives its buffered points to the secondary dimension with their residual
         * tolerances, and chooses the primary dimension for the next segment.
         */
        private void closePrimary() {
            updateDecay(residualY, primary().closeSegment());
            if (!buffer.isEmpty()) {
                Segment primarySegment = primary().lastSegment();
                for (TrajectoryPoint point : buffer)
                    feedSecondary(point, primarySegment);
                buffer.clear();
            }
            residualY = exponentialDecayX > exponentialDecayY;
        }

        /**
         * Adds a point to the secondary dimension with the tolerance left by {@code primarySegment}. If it does not
         * fit, the secondary segment is closed (with CONTINUOUS, after extending it with virtual points) and a new
         * one starts with the point.
         */
        private void feedSecondary(TrajectoryPoint point, Segment primarySegment) {
            DisjointPLA secondary = secondary();
            double tolerance = residual(primarySegment, point);
            if (!secondary.add(inSecondary(point), tolerance)) {
                TrajectoryPoint end = lastSecondaryPoint;
                if (continuous) {
                    // Same as in the primary. The residual tolerance is concave in t between two samples, so its
                    // linear interpolation (used by latestFittingTime) is a safe lower bound.
                    long t = secondary.latestFittingTime(inSecondary(point), lastSecondaryTolerance, tolerance);
                    if (t > end.t()) {
                        TrajectoryPoint virtual = interpolate(end, point, t);
                        if (secondary.add(inSecondary(virtual), residual(primarySegment, virtual))) {
                            end = virtual;
                        }
                    }
                }
                updateDecay(!residualY, secondary.closeSegment());
                if (continuous && point.t() - end.t() > 1) {
                    TrajectoryPoint virtual = interpolate(end, point, end.t() + 1);
                    secondary.add(inSecondary(virtual), residual(primarySegment, virtual)); // first point: always fits
                }
                secondary.add(inSecondary(point), tolerance); // first or second point: always fits
            }
            lastSecondaryPoint = point;
            lastSecondaryTolerance = tolerance;
        }

        /** Closes the open segments of both dimensions and returns all segments (X first, then Y). */
        private List<Segment> finish() {
            primary().closeSegment();
            Segment primarySegment = primary().lastSegment();
            for (TrajectoryPoint point : buffer)
                feedSecondary(point, primarySegment);
            secondary().closeSegment();

            plaX.getSegments().addAll(plaY.getSegments());
            return plaX.getSegments();
        }
    }

    /** The point at time t on the line from a to b (linear interpolation). */
    private static TrajectoryPoint interpolate(TrajectoryPoint a, TrajectoryPoint b, long t) {
        double ratio = (double) (t - a.t()) / (b.t() - a.t());
        return new TrajectoryPoint(t, a.x() + (b.x() - a.x()) * ratio, a.y() + (b.y() - a.y()) * ratio);
    }

    /**
     * √(ε² − err²), err being the largest error of the segment's extreme lines at the point. Never below ε/√2: the
     * primary's error is at most ε/√2, so this only guards against rounding.
     */
    private static double calculateResidual(Segment segment, TimeSeriesPoint point, double epsilon) {
        double maxError = Math.max(Math.abs(segment.getUpper().valueAt(point.getT()) - point.getV()), Math.abs(segment.getLower().valueAt(point.getT()) - point.getV()));

        return Math.max(Math.sqrt(epsilon * epsilon - maxError * maxError), epsilon / Math.sqrt(2));
    }

    /**
     * Evaluates the decompressed trajectory at the given timestamps (ascending): each dimension uses the last segment
     * that starts at or before t. Needs only timestamps, never the original coordinates. Timestamps outside
     * [firstTimestamp, lastTimestamp] are extrapolated from the first or last segment, without guarantee.
     *
     * @param decompressed the decompressed trajectory
     * @param timestamps timestamps in ascending order
     * @return one point per timestamp
     * @throws IllegalArgumentException if the timestamps are not ascending
     */
    public static List<TrajectoryPoint> evaluateAt(DecompressedTrajectory decompressed, long[] timestamps) {
        List<Line> linesX = decompressed.linesX(), linesY = decompressed.linesY();
        List<TrajectoryPoint> result = new ArrayList<>(timestamps.length);
        int xIdx = 0, yIdx = 0;
        long previousT = Long.MIN_VALUE;
        for (long t : timestamps) {
            if (t < previousT)
                throw new IllegalArgumentException("Timestamps must be ascending, got " + t + " after " + previousT);
            previousT = t;
            while (xIdx + 1 < linesX.size() && linesX.get(xIdx + 1).getTStart() <= t) xIdx++;
            while (yIdx + 1 < linesY.size() && linesY.get(yIdx + 1).getTStart() <= t) yIdx++;
            result.add(new TrajectoryPoint(t, linesX.get(xIdx).valueAt(t), linesY.get(yIdx).valueAt(t)));
        }

        return result;
    }

    /**
     * Same as {@link #evaluateAt}, but only the coordinates, as arrays: {@code coordinates[0][i]} and
     * {@code coordinates[1][i]} are x and y at {@code timestamps[i]}.
     *
     * @param decompressed the decompressed trajectory
     * @param timestamps timestamps in ascending order
     * @return {x, y}, one coordinate per timestamp each
     * @throws IllegalArgumentException if the timestamps are not ascending
     */
    public static double[][] coordinatesAt(DecompressedTrajectory decompressed, long[] timestamps) {
        List<Line> linesX = decompressed.linesX(), linesY = decompressed.linesY();
        double[] x = new double[timestamps.length], y = new double[timestamps.length];
        int xIdx = 0, yIdx = 0;
        for (int i = 0; i < timestamps.length; i++) {
            long t = timestamps[i];
            if (i > 0 && t < timestamps[i - 1])
                throw new IllegalArgumentException("Timestamps must be ascending, got " + t + " after " + timestamps[i - 1]);
            while (xIdx + 1 < linesX.size() && linesX.get(xIdx + 1).getTStart() <= t) xIdx++;
            while (yIdx + 1 < linesY.size() && linesY.get(yIdx + 1).getTStart() <= t) yIdx++;
            x[i] = linesX.get(xIdx).valueAt(t);
            y[i] = linesY.get(yIdx).valueAt(t);
        }

        return new double[][]{x, y};
    }

    /**
     * Position of the decompressed trajectory at time t: each dimension uses the last segment that starts at or
     * before t (binary search). For many ascending timestamps, {@link #evaluateAt} is faster. Outside
     * [firstTimestamp, lastTimestamp] the first or last segment is extrapolated, without guarantee.
     *
     * @param decompressed the decompressed trajectory
     * @param t the time
     * @return the position at time t
     */
    public static TrajectoryPoint positionAt(DecompressedTrajectory decompressed, long t) {
        return new TrajectoryPoint(t, lineAt(decompressed.linesX(), t).valueAt(t), lineAt(decompressed.linesY(), t).valueAt(t));
    }

    /** The last line that starts at or before t, or the first line if none does. */
    private static Line lineAt(List<Line> lines, long t) {
        int lo = 0, hi = lines.size() - 1;
        while (lo < hi) {
            int mid = (lo + hi + 1) >>> 1;
            if (lines.get(mid).getTStart() <= t) lo = mid;
            else hi = mid - 1;
        }
        return lines.get(lo);
    }

    /**
     * Reconstructs the trajectory without any external information, as the vertices of a piecewise-linear trajectory.
     * Each segment s_i of a dimension is used up to s_{i+1} - 1: timestamps are integers, so no time lies strictly
     * between them. Vertices are placed at every s_i and s_{i+1} - 1 of both dimensions and at the first and last
     * timestamp. With {@link ErrorBound#CONTINUOUS}, linear interpolation between consecutive vertices stays within ε
     * of the linearly interpolated original trajectory at every time. With {@link ErrorBound#SAMPLES}, ε is guaranteed
     * only at the original timestamps; between them, a segment may be extended over a sampling gap.
     *
     * @param decompressed the decompressed trajectory
     * @return the vertices, in time order
     */
    public static List<TrajectoryPoint> reconstructTrajectory(DecompressedTrajectory decompressed) {
        long first = decompressed.firstTimestamp(), last = decompressed.lastTimestamp();
        long[] times = new long[2 * (decompressed.linesX().size() + decompressed.linesY().size()) + 2];
        int n = 0;
        times[n++] = first;
        times[n++] = last;
        for (List<Line> lines : List.of(decompressed.linesX(), decompressed.linesY())) {
            for (Line line : lines) {
                long start = (long) line.getTStart();
                times[n++] = start;
                if (start > first) times[n++] = start - 1;
            }
        }
        Arrays.sort(times, 0, n);
        int unique = 0;
        for (int i = 0; i < n; i++)
            if (unique == 0 || times[i] != times[unique - 1]) times[unique++] = times[i];

        return evaluateAt(decompressed, Arrays.copyOf(times, unique));
    }
}
