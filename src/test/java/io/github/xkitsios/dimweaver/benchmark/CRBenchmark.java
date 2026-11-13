package io.github.xkitsios.dimweaver.benchmark;

import io.github.xkitsios.dimweaver.DecompressedTrajectory;
import io.github.xkitsios.dimweaver.DimWeaver;
import io.github.xkitsios.dimweaver.ErrorBound;
import io.github.xkitsios.dimweaver.TrajectoryPoint;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.stream.Stream;

/**
 * Compression ratio of DimWeaver with {@link ErrorBound#SAMPLES} and {@link ErrorBound#CONTINUOUS} on one dataset.
 * <p>
 * Usage: {@code CRBenchmark <dataset dir> <dataset name> <output csv>}. Reads every *.csv (t,x,y with a header) under
 * the directory, recursively. Writes the CSV "epsilon,CR samples,CR continuous".
 * The error bound is checked for every trajectory (at the samples, and continuously for CONTINUOUS); violations are
 * counted and reported instead of aborting. Runs on one thread; Binary is thread-safe, so datasets can run in parallel.
 */
public class CRBenchmark {
    private static final double FP_TOLERANCE = 1e-9;

    static double[] epsilons(String dataset) {
        double base = switch (dataset) {
            case "atc" -> 0.01;
            case "trajair" -> 10.0;
            case "robots" -> 0.1;
            default -> 0.0001;
        };
        return new double[]{base, 2 * base, 3 * base, 4 * base, 5 * base};
    }

    public static void main(String[] args) throws IOException {
        Locale.setDefault(Locale.US);
        Path dir = Path.of(args[0]);
        String dataset = args[1];
        Path out = Path.of(args[2]);
        double[] epsilons = epsilons(dataset);

        List<Path> files;
        try (Stream<Path> walk = Files.walk(dir)) {
            files = walk.filter(p -> p.toString().endsWith(".csv") && Files.isRegularFile(p)).sorted().toList();
        }

        long originalSize = 0, points = 0, trajectories = 0, rejected = 0, unreadable = 0;
        long[] samplesSize = new long[epsilons.length], continuousSize = new long[epsilons.length];
        long[] samplesViolations = new long[epsilons.length], continuousViolations = new long[epsilons.length];
        double[] continuousMaxRatio = new double[epsilons.length];
        String firstRejection = null;
        long start = System.nanoTime();

        for (Path file : files) {
            List<TrajectoryPoint> trajectory;
            try {
                trajectory = read(file);
            } catch (IOException | RuntimeException e) {
                unreadable++;
                continue;
            }
            if (trajectory.isEmpty()) continue;
            try {
                long[] sizes = new long[2 * epsilons.length];
                long[] sv = new long[epsilons.length], cv = new long[epsilons.length];
                double[] cMax = new double[epsilons.length];
                long[] timestamps = trajectory.stream().mapToLong(TrajectoryPoint::t).toArray();
                for (int i = 0; i < epsilons.length; i++) {
                    double epsilon = epsilons[i];
                    byte[] binary = DimWeaver.compress(trajectory, epsilon, ErrorBound.SAMPLES);
                    sizes[2 * i] = binary.length;
                    sv[i] = samplesViolations(trajectory, DimWeaver.evaluateAt(DimWeaver.decompress(binary), timestamps), epsilon);

                    binary = DimWeaver.compress(trajectory, epsilon, ErrorBound.CONTINUOUS);
                    sizes[2 * i + 1] = binary.length;
                    DecompressedTrajectory decompressed = DimWeaver.decompress(binary);
                    sv[i] += samplesViolations(trajectory, DimWeaver.evaluateAt(decompressed, timestamps), epsilon);
                    cMax[i] = continuousMaxError(trajectory, DimWeaver.reconstructTrajectory(decompressed)) / epsilon;
                    cv[i] = cMax[i] > 1 + FP_TOLERANCE ? 1 : 0;
                }
                // Count the trajectory only once all epsilons succeeded
                for (int i = 0; i < epsilons.length; i++) {
                    samplesSize[i] += sizes[2 * i];
                    continuousSize[i] += sizes[2 * i + 1];
                    samplesViolations[i] += sv[i];
                    continuousViolations[i] += cv[i];
                    continuousMaxRatio[i] = Math.max(continuousMaxRatio[i], cMax[i]);
                }
                originalSize += (long) trajectory.size() * (8 + 8 + 8);
                points += trajectory.size();
                trajectories++;
            } catch (IllegalArgumentException e) {
                if (firstRejection == null) firstRejection = file.getFileName() + ": " + e.getMessage();
                rejected++;
            }
        }

        Files.createDirectories(out.toAbsolutePath().getParent());
        try (PrintWriter writer = new PrintWriter(Files.newBufferedWriter(out))) {
            for (int i = 0; i < epsilons.length; i++)
                writer.printf("%s,%.2f,%.2f%n", format(epsilons[i]), (double) originalSize / samplesSize[i], (double) originalSize / continuousSize[i]);
        }

        System.out.printf("%s: %d trajectories, %d points, %.1fs; rejected %d (IllegalArgumentException), unreadable %d%n",
                dataset, trajectories, points, (System.nanoTime() - start) / 1e9, rejected, unreadable);
        if (firstRejection != null) System.out.println("  first rejection: " + firstRejection);
        System.out.println("  epsilon, CR samples, CR continuous, sample violations, continuous violations (trajectories), max continuous error/eps");
        for (int i = 0; i < epsilons.length; i++)
            System.out.printf("  %s, %.2f, %.2f, %d, %d, %.9f%n", format(epsilons[i]), (double) originalSize / samplesSize[i],
                    (double) originalSize / continuousSize[i], samplesViolations[i], continuousViolations[i], continuousMaxRatio[i]);
    }

    private static String format(double epsilon) {
        return String.format("%.6f", epsilon).replaceAll("0+$", "").replaceAll("\\.$", "");
    }

    private static List<TrajectoryPoint> read(Path file) throws IOException {
        List<TrajectoryPoint> points = new ArrayList<>();
        try (BufferedReader reader = Files.newBufferedReader(file)) {
            String line = reader.readLine(); // header
            while ((line = reader.readLine()) != null) {
                if (line.isBlank()) continue;
                String[] parts = line.split(",");
                points.add(new TrajectoryPoint(Long.parseLong(parts[0].trim()), Double.parseDouble(parts[1]), Double.parseDouble(parts[2])));
            }
        }
        return points;
    }

    private static long samplesViolations(List<TrajectoryPoint> original, List<TrajectoryPoint> evaluated, double epsilon) {
        long violations = 0;
        for (int i = 0; i < original.size(); i++)
            if (Math.hypot(evaluated.get(i).x() - original.get(i).x(), evaluated.get(i).y() - original.get(i).y()) > epsilon * (1 + FP_TOLERANCE))
                violations++;
        return violations;
    }

    /**
     * Max distance between the two linearly interpolated polylines, checked at the vertices of both.
     */
    private static double continuousMaxError(List<TrajectoryPoint> original, List<TrajectoryPoint> reconstructed) {
        double max = 0;
        int o = 0, r = 0;
        while (o < original.size() || r < reconstructed.size()) {
            long t;
            if (r >= reconstructed.size() || (o < original.size() && original.get(o).t() <= reconstructed.get(r).t()))
                t = original.get(o++).t();
            else
                t = reconstructed.get(r++).t();
            double[] a = at(original, t), b = at(reconstructed, t);
            max = Math.max(max, Math.hypot(a[0] - b[0], a[1] - b[1]));
        }
        return max;
    }

    private static double[] at(List<TrajectoryPoint> polyline, long t) {
        int lo = 0, hi = polyline.size() - 1;
        while (lo < hi) { // last vertex with time <= t
            int mid = (lo + hi + 1) >>> 1;
            if (polyline.get(mid).t() <= t) lo = mid;
            else hi = mid - 1;
        }
        TrajectoryPoint a = polyline.get(lo);
        if (a.t() == t || lo + 1 == polyline.size()) return new double[]{a.x(), a.y()};
        TrajectoryPoint b = polyline.get(lo + 1);
        double ratio = (double) (t - a.t()) / (b.t() - a.t());
        return new double[]{a.x() + (b.x() - a.x()) * ratio, a.y() + (b.y() - a.y()) * ratio};
    }
}
