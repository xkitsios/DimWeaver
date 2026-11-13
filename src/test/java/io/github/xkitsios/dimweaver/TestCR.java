package io.github.xkitsios.dimweaver;

import io.github.xkitsios.dimweaver.utils.Reader;
import io.github.xkitsios.dimweaver.utils.Validation;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;


public class TestCR {
    String datasetFolder = "src/test/resources/";

    private long calcBinarySize(List<TrajectoryPoint> trajectory) {
        return (long) trajectory.size() * (8 + 8 + 8);
    }

    private void run(List<String> trajectoryFiles, List<Double> epsilons) throws IOException {
        long[] samplesSize = new long[epsilons.size()];
        long[] continuousSize = new long[epsilons.size()];
        long trajectoriesSize = 0;

        for (String trajectoryFile : trajectoryFiles) {
            List<TrajectoryPoint> trajectory = Reader.parse(trajectoryFile);
            trajectoriesSize += calcBinarySize(trajectory);
            for (int i = 0; i < epsilons.size(); i++) {
                double epsilon = epsilons.get(i);

                byte[] binary = DimWeaver.compress(trajectory, epsilon, ErrorBound.SAMPLES);
                samplesSize[i] += binary.length;
                Validation.validateAtSamples(trajectory, DimWeaver.decompress(binary), epsilon);

                binary = DimWeaver.compress(trajectory, epsilon, ErrorBound.CONTINUOUS);
                continuousSize[i] += binary.length;
                DecompressedTrajectory decompressed = DimWeaver.decompress(binary);
                Validation.validateAtSamples(trajectory, decompressed, epsilon);
                Validation.validateContinuous(trajectory, decompressed, epsilon);
            }
        }

        System.out.println("epsilon,CR samples,CR continuous");
        for (int i = 0; i < epsilons.size(); i++)
            System.out.printf("%.4f,%.2f,%.2f%n", epsilons.get(i), (double) trajectoriesSize / samplesSize[i], (double) trajectoriesSize / continuousSize[i]);
    }

    @Test
    public void testGeoLife() throws IOException {
        Locale.setDefault(Locale.US);
        String dataset = "geolife";
        List<String> trajectoriesFiles = Reader.list(datasetFolder + dataset);
        ArrayList<Double> epsilons = new ArrayList<>();
        for (double i = 0.0001; i <= 0.0005; i += 0.0001)
            epsilons.add(i);
        run(trajectoriesFiles, epsilons);
    }
}
