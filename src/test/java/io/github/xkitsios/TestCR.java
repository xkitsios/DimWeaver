package io.github.xkitsios;

import io.github.xkitsios.utils.Reader;
import io.github.xkitsios.utils.Trajectory;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.junit.jupiter.api.Assertions.assertTrue;


public class TestCR {
    String datasetFolder = "src/test/resources/";

    private void validate(List<TrajectoryPoint> init, List<TrajectoryPoint> simplified, double epsilon) {
        int simplifiedIdx = -2;
        double error, simplifiedX, simplifiedY, slopeX = 0, slopeY = 0;
        for (TrajectoryPoint initPoint : init) {
            if (initPoint.getT() == simplified.getLast().getT()) break;
            if (simplified.get(simplifiedIdx + 2).getT() == initPoint.getT()) {
                simplifiedIdx += 2;
                simplifiedX = simplified.get(simplifiedIdx).getX();
                simplifiedY = simplified.get(simplifiedIdx).getY();
                if (initPoint.getT() != simplified.getLast().getT()) {
                    slopeX = (simplified.get(simplifiedIdx + 1).getX() - simplified.get(simplifiedIdx).getX()) / (simplified.get(simplifiedIdx + 1).getT() - simplified.get(simplifiedIdx).getT());
                    slopeY = (simplified.get(simplifiedIdx + 1).getY() - simplified.get(simplifiedIdx).getY()) / (simplified.get(simplifiedIdx + 1).getT() - simplified.get(simplifiedIdx).getT());
                }
            } else {
                simplifiedX = slopeX * (initPoint.getT() - simplified.get(simplifiedIdx).getT()) + simplified.get(simplifiedIdx).getX();
                simplifiedY = slopeY * (initPoint.getT() - simplified.get(simplifiedIdx).getT()) + simplified.get(simplifiedIdx).getY();
            }
            error = Math.sqrt(Math.pow(simplifiedX - initPoint.getX(), 2) + Math.pow(simplifiedY - initPoint.getY(), 2));
            assertTrue(error <= epsilon * 1.1);
        }
    }

    private void run(List<String> trajectoryFiles, List<Double> epsilons, double c) throws IOException {
        long[] dimWeaverSize = new long[epsilons.size()];
        long trajectoriesSize = 0;

        for (String trajectoryFile : trajectoryFiles) {
            Trajectory trajectory = Reader.parse(trajectoryFile);
            trajectoriesSize += trajectory.binarySize();
            for (int i = 0; i < epsilons.size(); i++) {
                double epsilon = epsilons.get(i);

                byte[] binary = DimWeaver.compress(trajectory.trajectoryPoints, epsilon, c);
                dimWeaverSize[i] += binary.length;
                List<TrajectoryPoint> simplified = DimWeaver.decompress(binary);
                validate(trajectory.trajectoryPoints, simplified, epsilon);
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
