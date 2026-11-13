package io.github.xkitsios.dimweaver;

/**
 * A line v(t) = a·(t − tStart) + b of one dimension, anchored at tStart: b is its value at tStart. Anchoring keeps
 * the arithmetic away from large absolute timestamps.
 */
class Line {
    private final double tStart;
    private final double a;
    private final double b;

    /** The line through (t1, v1) and (t2, v2), anchored at t1. */
    public Line(double t1, double v1, double t2, double v2) {
        this.tStart = t1;
        this.a = (v2 - v1) / (t2 - t1);
        this.b = v1;
    }

    /** The line with slope a and value b at tStart. */
    public Line(double tStart, double a, double b) {
        this.tStart = tStart;
        this.a = a;
        this.b = b;
    }

    /** Value of the line at time t. */
    public double valueAt(double t) {
        return this.a * (t - this.tStart) + this.b;
    }

    /** Anchor time; in a decompressed trajectory, the time the segment starts. */
    public double getTStart() {
        return tStart;
    }

    /** Slope (velocity, in coordinate units per time unit). */
    public double getA() {
        return a;
    }

    /** Value at the anchor time (intercept). */
    public double getB() {
        return b;
    }
}
