package io.github.xkitsios.utils;

import io.github.xkitsios.TrajectoryPoint;

import java.util.List;

public class Trajectory {
    public List<TrajectoryPoint> trajectoryPoints;

    public Trajectory(List<TrajectoryPoint> trajectoryPoints) {
        this.trajectoryPoints = trajectoryPoints;
    }

    public int binarySize() {
        return trajectoryPoints.size() * (8 + 8 + 8);
    }
}
