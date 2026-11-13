package io.github.xkitsios.dimweaver.utils;

import io.github.xkitsios.dimweaver.DecompressedTrajectory;
import io.github.xkitsios.dimweaver.DimWeaver;
import io.github.xkitsios.dimweaver.TrajectoryPoint;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class Validation {
    // The bound holds exactly in real arithmetic; allow only floating-point rounding (~1 ulp of the coordinates).
    public static final double FP_TOLERANCE = 1e-9;

    /**
     * Checks the error at the original samples, evaluating the decompressed trajectory at their timestamps only.
     */
    public static void validateAtSamples(List<TrajectoryPoint> original, DecompressedTrajectory decompressed, double epsilon) {
        assertEquals(original.get(0).t(), decompressed.firstTimestamp());
        assertEquals(original.get(original.size() - 1).t(), decompressed.lastTimestamp());
        long[] timestamps = original.stream().mapToLong(TrajectoryPoint::t).toArray();
        List<TrajectoryPoint> evaluated = DimWeaver.evaluateAt(decompressed, timestamps);
        assertEquals(original.size(), evaluated.size());
        for (int i = 0; i < original.size(); i++) {
            assertEquals(original.get(i).t(), evaluated.get(i).t());
            double error = Math.hypot(evaluated.get(i).x() - original.get(i).x(), evaluated.get(i).y() - original.get(i).y());
            assertTrue(error <= epsilon * (1 + FP_TOLERANCE), "sample " + i + ": error " + error + " > epsilon " + epsilon);
        }
    }

    /**
     * Checks the error at every time: the timestamp-free reconstruction and the original, both linearly interpolated.
     * The error is linear between consecutive vertices of either polyline and its norm convex, so checking all
     * vertices of both covers every time in between.
     */
    public static void validateContinuous(List<TrajectoryPoint> original, DecompressedTrajectory decompressed, double epsilon) {
        List<TrajectoryPoint> reconstructed = DimWeaver.reconstructTrajectory(decompressed);
        assertEquals(original.get(0).t(), reconstructed.get(0).t());
        assertEquals(original.get(original.size() - 1).t(), reconstructed.get(reconstructed.size() - 1).t());
        for (int i = 1; i < reconstructed.size(); i++)
            assertTrue(reconstructed.get(i).t() > reconstructed.get(i - 1).t(), "reconstructed timestamps must be strictly increasing");

        long[] times = new long[original.size() + reconstructed.size()];
        int n = 0;
        for (TrajectoryPoint p : original) times[n++] = p.t();
        for (TrajectoryPoint p : reconstructed) times[n++] = p.t();
        Arrays.sort(times);

        int o = 0, r = 0;
        for (long t : times) {
            while (o + 1 < original.size() && original.get(o + 1).t() <= t) o++;
            while (r + 1 < reconstructed.size() && reconstructed.get(r + 1).t() <= t) r++;
            double[] a = interpolate(original, o, t), b = interpolate(reconstructed, r, t);
            double error = Math.hypot(a[0] - b[0], a[1] - b[1]);
            assertTrue(error <= epsilon * (1 + FP_TOLERANCE), "t=" + t + ": error " + error + " > epsilon " + epsilon);
        }
    }

    private static double[] interpolate(List<TrajectoryPoint> polyline, int i, long t) {
        TrajectoryPoint a = polyline.get(i);
        if (a.t() == t || i + 1 == polyline.size()) return new double[]{a.x(), a.y()};
        TrajectoryPoint b = polyline.get(i + 1);
        double ratio = (double) (t - a.t()) / (b.t() - a.t());
        return new double[]{a.x() + (b.x() - a.x()) * ratio, a.y() + (b.y() - a.y()) * ratio};
    }
}
