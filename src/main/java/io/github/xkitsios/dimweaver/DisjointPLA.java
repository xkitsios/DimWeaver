package io.github.xkitsios.dimweaver;

import java.util.ArrayList;
import java.util.List;

/**
 * Disjoint piecewise-linear approximation of one dimension: points are added in time order to the open segment for
 * as long as some line is within the tolerance of all of them; then the segment is closed and the next one starts
 * at the next point (segments do not share end points). Each point may have its own tolerance: the secondary
 * dimension gets what the primary did not use. The open segment is kept as its two extreme lines, {@code upper} and
 * {@code lower}, updated in time proportional to the hull size.
 */
class DisjointPLA {
    /**
     * Fraction of the tolerance kept as slack when a segment is extended with a virtual point (see latestFittingTime).
     */
    static final double VIRTUAL_POINT_MARGIN = 0.25;

    private final boolean y; // the dimension: false for X, true for Y
    private final ConvexHull hull;
    private final List<Segment> segments; // closed segments, in time order
    private final List<TimeSeriesPoint> segmentPoints; // points of the open segment
    private Line lower; // extreme lines of the open segment (null with fewer than 2 points)
    private Line upper;

    /** An approximation of dimension X ({@code y} false) or Y ({@code y} true). */
    protected DisjointPLA(boolean y) {
        this.y = y;
        this.hull = new ConvexHull();
        this.segments = new ArrayList<>();
        this.segmentPoints = new ArrayList<>();
    }

    /**
     * Adds the point to the open segment if a line within the tolerance of all its points, with a slope representable
     * as a float, still exists. Otherwise the open segment is left unchanged and false is returned; the caller decides
     * how to close it. The first two points are always accepted (with 2 points the slope interval has width
     * 2(ε₀ + ε₁)/Δt, which contains floats unless the displacement is ~10⁷ times ε).
     */
    protected boolean add(TimeSeriesPoint newPoint, double epsilon) {
        segmentPoints.add(newPoint);
        if (segmentPoints.size() == 1) {
            hull.addToHull(newPoint, epsilon);
        } else if (segmentPoints.size() == 2) {
            lower = new Line(hull.get(0, true).getT(), hull.get(0, true).getV(), segmentPoints.get(1).getT(), segmentPoints.get(1).getV() - epsilon);
            upper = new Line(hull.get(0, false).getT(), hull.get(0, false).getV(), segmentPoints.get(1).getT(), segmentPoints.get(1).getV() + epsilon);
            hull.addToHull(newPoint, epsilon);
        } else {
            if (newPoint.getV() - epsilon > upper.valueAt(newPoint.getT()) || newPoint.getV() + epsilon < lower.valueAt(newPoint.getT())) {
                segmentPoints.remove(segmentPoints.size() - 1);
                return false;
            }
            // Candidate lines from the hull *before* adding the point, so that the point can still be rejected without
            // undoing hull changes. Same result as after adding it: the tangent point from (t, v ∓ ε) has every
            // previous point on one side, and so does the new hull point (t, v ± ε), hence adding it cannot prune it.
            Line newLower = lower, newUpper = upper;
            if (lower.valueAt(newPoint.getT()) + epsilon < newPoint.getV()) {
                for (int i = 0; i < hull.size(true); i++) {
                    Line tempLine = new Line(hull.get(i, true).getT(), hull.get(i, true).getV(), newPoint.getT(), newPoint.getV() - epsilon);
                    if (tempLine.getA() > newLower.getA()) newLower = tempLine;
                }
            }
            if (upper.valueAt(newPoint.getT()) - epsilon > newPoint.getV()) {
                for (int i = 0; i < hull.size(false); i++) {
                    Line tempLine = new Line(hull.get(i, false).getT(), hull.get(i, false).getV(), newPoint.getT(), newPoint.getV() + epsilon);
                    if (tempLine.getA() < newUpper.getA()) newUpper = tempLine;
                }
            }
            // The slope is stored as a float: the segment must keep at least one float slope within its interval
            if (!FloatSlope.containsFloat(Math.min(newLower.getA(), newUpper.getA()), Math.max(newLower.getA(), newUpper.getA()))) {
                segmentPoints.remove(segmentPoints.size() - 1);
                return false;
            }
            hull.addToHull(newPoint, epsilon);
            lower = newLower;
            upper = newUpper;
        }

        return true;
    }

    /**
     * Latest integer time t in [q.t, p.t - 1], where q is the last point of the open segment, at which the point on
     * the line from q to p (linear interpolation) still fits the open segment. The tolerance varies linearly from {@code toleranceQ} at q to
     * {@code toleranceP} at p. With C(t) that line and τ(t) the tolerance, the point fits iff
     * <pre>
     *   f(t) = upper(t) - C(t) + τ(t) >= 0   and   g(t) = C(t) + τ(t) - lower(t) >= 0
     * </pre>
     * Both are linear in t, so each is evaluated at q and p and, if negative at p, its root is found by linear
     * interpolation: t = q + (p - q) · f(q) / (f(q) - f(p)). The answer is the floor of the earliest root, in O(1).
     * Returns q.t if no later time fits. Only meaningful right after {@link #add} rejected p.
     * <p>
     * The roots are computed with the tolerance reduced by {@link #VIRTUAL_POINT_MARGIN}, so that the virtual point
     * keeps a slack of 25% of its tolerance from the band boundary (it is then added with the full tolerance). A point
     * exactly on the boundary pins the segment's slope interval to almost a single slope, which hurts kinematic
     * grouping. The error bound is unaffected: any time in [q, t*] is a valid end.
     */
    protected long latestFittingTime(TimeSeriesPoint p, double toleranceQ, double toleranceP) {
        TimeSeriesPoint q = segmentPoints.get(segmentPoints.size() - 1);
        long dt = p.getT() - q.getT();
        if (dt <= 1) return q.getT();
        if (upper == null) return p.getT() - 1; // a single point: any interpolated point fits
        toleranceQ *= 1 - VIRTUAL_POINT_MARGIN;
        toleranceP *= 1 - VIRTUAL_POINT_MARGIN;

        double fq = upper.valueAt(q.getT()) - q.getV() + toleranceQ;
        double gq = q.getV() + toleranceQ - lower.valueAt(q.getT());
        if (fq < 0 || gq < 0) return q.getT();
        double fp = upper.valueAt(p.getT()) - p.getV() + toleranceP;
        double gp = p.getV() + toleranceP - lower.valueAt(p.getT());

        double offset = dt - 1; // relative to q.t, to keep the arithmetic away from large absolute timestamps
        if (fp < 0) offset = Math.min(offset, dt * fq / (fq - fp));
        if (gp < 0) offset = Math.min(offset, dt * gq / (gq - gp));

        return q.getT() + (long) Math.floor(offset);
    }

    /**
     * Closes the open segment, if any, and returns its number of points.
     */
    protected int closeSegment() {
        int size = segmentPoints.size();
        if (size == 1)
            segments.add(new Segment(segmentPoints.get(0).getT(), new Line(segmentPoints.get(0).getT(), 0, hull.get(0, true).getV()), new Line(segmentPoints.get(0).getT(), 0, hull.get(0, false).getV()), y));
        else if (size > 1) segments.add(new Segment(segmentPoints.get(0).getT(), upper, lower, y));
        segmentPoints.clear();
        lower = null;
        upper = null;
        hull.clearHull();

        return size;
    }

    /** The most recently closed segment. */
    protected Segment lastSegment() {
        return segments.get(segments.size() - 1);
    }

    /** All closed segments, in time order (the live list). */
    protected List<Segment> getSegments() {
        return segments;
    }
}
