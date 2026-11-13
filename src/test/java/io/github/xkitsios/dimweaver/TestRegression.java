package io.github.xkitsios.dimweaver;

import io.github.xkitsios.dimweaver.utils.Reader;
import io.github.xkitsios.dimweaver.utils.Validation;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.TestFactory;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Real trajectories that once broke the error bound, in src/test/resources/regression/<dataset>/. Each one is
 * compressed with both error bounds for the dataset's five epsilons and checked at the samples and, for CONTINUOUS,
 * at every time.
 * <p>
 * Current files: trajectories whose slope interval collapsed below float resolution in CONTINUOUS mode (virtual points
 * on the band boundary), which exceeded ε by up to 1.000015ε before DisjointPLA rejected unrepresentable slopes.
 */
public class TestRegression {
    private static final String FOLDER = "src/test/resources/regression";

    private static double epsilonBase(String dataset) {
        return switch (dataset) {
            case "atc" -> 0.01;
            case "trajair" -> 10.0;
            case "robots" -> 0.1;
            default -> 0.0001;
        };
    }

    @TestFactory
    public List<DynamicTest> regressionTrajectories() {
        List<DynamicTest> tests = new ArrayList<>();
        File[] datasets = Objects.requireNonNull(new File(FOLDER).listFiles(File::isDirectory));
        for (File dataset : datasets) {
            for (String file : Reader.list(dataset.getPath())) {
                String name = dataset.getName() + "/" + new File(file).getName();
                tests.add(DynamicTest.dynamicTest(name, () -> {
                    List<TrajectoryPoint> trajectory = Reader.parse(file);
                    for (int k = 1; k <= 5; k++) {
                        double epsilon = k * epsilonBase(dataset.getName());
                        Validation.validateAtSamples(trajectory,
                                DimWeaver.decompress(DimWeaver.compress(trajectory, epsilon, ErrorBound.SAMPLES)), epsilon);
                        DecompressedTrajectory continuous = DimWeaver.decompress(DimWeaver.compress(trajectory, epsilon, ErrorBound.CONTINUOUS));
                        Validation.validateAtSamples(trajectory, continuous, epsilon);
                        Validation.validateContinuous(trajectory, continuous, epsilon);
                    }
                }));
            }
        }
        return tests;
    }
}
