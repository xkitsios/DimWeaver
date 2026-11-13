package io.github.xkitsios.dimweaver;

/** A point (t, v) of one dimension of a trajectory. */
class TimeSeriesPoint {
    private final long t;
    private final double v;

    protected TimeSeriesPoint(long t, double v) {
        this.t = t;
        this.v = v;
    }

    protected long getT() {
        return t;
    }

    protected double getV() {
        return v;
    }
}
