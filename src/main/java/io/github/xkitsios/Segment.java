package io.github.xkitsios;

class Segment {
    private final long start;
    private final boolean isY;
    private Line upper;
    private Line middle;
    private Line lower;

    protected Segment(long start, Line middle, boolean isY) {
        this.start = start;
        this.isY = isY;
        this.upper = null;
        this.middle = middle;
        this.lower = null;
    }

    protected Segment(long start, Line upper, Line lower, boolean isY) {
        this.start = start;
        this.upper = upper;
        this.middle = null;
        this.lower = lower;
        this.isY = isY;
    }

    protected long getStart() {
        return start;
    }

    protected Line getUpper() {
        return upper;
    }

    protected Line getMiddle() {
        if (middle == null) middle = Line.getMiddleLine(start, upper, lower);
        return middle;
    }

    protected Line getLower() {
        return lower;
    }

    protected boolean getIsY() {
        return isY;
    }

    protected double getSmallestSlope() {
        return Math.min(lower.getA(), upper.getA());
    }

    protected double getBiggestSlope() {
        return Math.max(lower.getA(), upper.getA());
    }

    protected void updateSlopes(double smallest, double biggest) {
        double aLow, aUp;
        if (lower.getA() < upper.getA()) {
            aLow = smallest;
            aUp = biggest;
        } else if (lower.getA() > upper.getA()) {
            aLow = biggest;
            aUp = smallest;
        } else {
            return;
        }
        double intersectionT = (upper.getA() * upper.getTStart() - upper.getB() - lower.getA() * lower.getTStart() + lower.getB()) / (upper.getA() - lower.getA());
        double intersectionV = lower.getA() * (intersectionT - lower.getTStart()) + lower.getB();
        lower = new Line(lower.getTStart(), aLow, intersectionV - aLow * (intersectionT - lower.getTStart()));
        upper = new Line(upper.getTStart(), aUp, intersectionV - aUp * (intersectionT - upper.getTStart()));
        middle = null;
    }
}

