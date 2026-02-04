package io.github.xkitsios;

import io.github.xkitsios.utils.Reader;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;


public class TestCR {
    String datasetFolder = "src/test/resources/";

    private void validate(List<TrajectoryPoint> original, List<TrajectoryPoint> simplified, double epsilon) {
        assertEquals(simplified.size(), original.size());
        for (int i = 0; i < simplified.size(); i++) {
            float error = (float) Math.sqrt(Math.pow(simplified.get(i).x() - original.get(i).x(), 2) + Math.pow(simplified.get(i).y() - original.get(i).y(), 2));
            assertEquals(simplified.get(i).t(), original.get(i).t());
            assertTrue(error <= epsilon * 1.1);
        }
    }

    private long calcBinarySize(List<TrajectoryPoint> trajectory) {
        return (long) trajectory.size() * (8 + 8 + 8);
    }

    private void run(List<String> trajectoryFiles, List<Double> epsilons, double c) throws IOException {
        long[] dimWeaverSize = new long[epsilons.size()];
        long trajectoriesSize = 0;

        for (String trajectoryFile : trajectoryFiles) {
            List<TrajectoryPoint> trajectory = Reader.parse(trajectoryFile);
            trajectoriesSize += calcBinarySize(trajectory);
            for (int i = 0; i < epsilons.size(); i++) {
                double epsilon = epsilons.get(i);

                byte[] binary = DimWeaver.compress(trajectory, epsilon, c);
                dimWeaverSize[i] += binary.length;
                List<List<Line>> lines = DimWeaver.decompress(binary);
                List<TrajectoryPoint> simplified = DimWeaver.reconstructTrajectory(lines, trajectory);
                validate(trajectory, simplified, epsilon);
            }
        }

        System.out.println("epsilon,CR");
        for (int i = 0; i < epsilons.size(); i++)
            System.out.println(String.format("%.4f,", epsilons.get(i)) + String.format("%.2f", (double) trajectoriesSize / dimWeaverSize[i]));
    }

    @Test
    public void testGeoLife() throws IOException {
        Locale.setDefault(Locale.US);
        String dataset = "geolife";
        List<String> trajectoriesFiles = Reader.list(datasetFolder + dataset);
        ArrayList<Double> epsilons = new ArrayList<>();
        for (double i = 0.0001; i <= 0.0005; i += 0.0001)
            epsilons.add(i);
        run(trajectoriesFiles, epsilons, 0.85);
    }
}
