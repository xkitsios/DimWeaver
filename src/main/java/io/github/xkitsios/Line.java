package io.github.xkitsios;

class Line {
    private final double tStart;
    private final double a;
    private final double b;

    protected Line(double t1, double v1, double t2, double v2) {
        this.tStart = t1;
        this.a = (v2 - v1) / (t2 - t1);
        this.b = v1;
    }

    protected Line(double tStart, double a, double b) {
        this.tStart = tStart;
        this.a = a;
        this.b = b;
    }

    protected static Line getMiddleLine(double tStart, Line upper, Line lower) {
        if (upper.getA() - lower.getA() == 0) return new Line(tStart, 0, (lower.getB() + upper.getB()) / 2);
        double intersectionT = (upper.getA() * upper.getTStart() - upper.getB() - lower.getA() * lower.getTStart() + lower.getB()) / (upper.getA() - lower.getA());
        double intersectionV = lower.getA() * (intersectionT - lower.getTStart()) + lower.getB();
        double aMiddle = (lower.getA() + upper.getA()) / 2;
        return new Line(tStart, aMiddle, intersectionV - aMiddle * (intersectionT - tStart));
    }

    protected double valueAt(long t) {
        return this.a * (t - this.tStart) + this.b;
    }

    protected double getTStart() {
        return tStart;
    }

    protected double getA() {
        return a;
    }

    protected double getB() {
        return b;
    }
}
