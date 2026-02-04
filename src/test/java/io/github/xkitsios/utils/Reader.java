package io.github.xkitsios.utils;

import io.github.xkitsios.TrajectoryPoint;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Stream;

public class Reader {
    public static List<String> list(String dir) {
        return Stream.of(Objects.requireNonNull(new File(dir).listFiles())).filter(file -> !file.isDirectory()).filter(file -> file.getName().endsWith(".csv")).map(File::getAbsolutePath).toList();
    }

    public static List<TrajectoryPoint> parse(String filename) {
        ArrayList<TrajectoryPoint> trajectoryPoints = new ArrayList<>();
        try {
            BufferedReader reader = new BufferedReader(new FileReader(filename));
            reader.readLine();
            String line;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                long t = Long.parseLong(parts[0]);
                double x = Double.parseDouble(parts[1]);
                double y = Double.parseDouble(parts[2]);
                trajectoryPoints.add(new TrajectoryPoint(t, x, y));
            }
            reader.close();
        } catch (Exception e) {
            System.err.println("Error reading " + filename);
        }

        return trajectoryPoints;
    }
}
