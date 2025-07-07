package io.github.xkitsios;

public class TrajectoryPoint {
    private final long t;
    private final double x;
    private final double y;

    public TrajectoryPoint(long t, double x, double y) {
        this.t = t;
        this.x = x;
        this.y = y;
    }

    public long getT() {
        return t;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
