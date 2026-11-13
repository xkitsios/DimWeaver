package io.github.xkitsios.dimweaver;

/**
 * A segment of one dimension, as produced by {@link DisjointPLA}: it starts at {@code start} and holds until the next
 * segment of the same dimension starts. {@code upper} and {@code lower} are the two extreme lines that are within the
 * tolerance of all the segment's points; any line through their intersection with a slope between theirs is too, so
 * the segment can be stored with any such slope (see {@link KinematicGrouping}).
 */
class Segment {
    private final long start;
    private final boolean dim; // false: X, true: Y
    private final Line upper;
    private final Line lower;

    /** Both lines are re-anchored at {@code start}. {@code dim}: false for X, true for Y. */
    public Segment(long start, Line upper, Line lower, boolean dim) {
        this.start = start;
        if (upper.getTStart() != start) this.upper = new Line(start, upper.getA(), upper.valueAt(start));
        else this.upper = upper;
        if (lower.getTStart() != start) this.lower = new Line(start, lower.getA(), lower.valueAt(start));
        else this.lower = lower;
        this.dim = dim;
    }

    /** Start time of the segment. */
    public long getStart() {
        return start;
    }

    public Line getUpper() {
        return upper;
    }

    public Line getLower() {
        return lower;
    }

    /** The dimension: false for X, true for Y. */
    public boolean getDim() {
        return dim;
    }

    /** Lower end of the interval of admissible slopes. */
    public double getSmallestSlope() {
        return Math.min(lower.getA(), upper.getA());
    }

    /** Upper end of the interval of admissible slopes. */
    public double getBiggestSlope() {
        return Math.max(lower.getA(), upper.getA());
    }

    /**
     * Returns the line with slope {@code a} that passes through the intersection of upper and lower.
     * For any {@code a} between the two slopes this line is feasible (convexity of the feasible region).
     * The intersection is computed relative to {@code start} (both lines are anchored there): working with
     * absolute timestamps would cancel large {@code a * t} products and shift the line outside the bound.
     */
    public Line lineWithSlope(double a) {
        if (upper.getA() == lower.getA()) return new Line(start, a, (upper.getB() + lower.getB()) / 2);
        double dt = (lower.getB() - upper.getB()) / (upper.getA() - lower.getA());
        return new Line(start, a, lower.getB() + (lower.getA() - a) * dt);
    }
}

