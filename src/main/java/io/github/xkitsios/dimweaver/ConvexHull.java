package io.github.xkitsios.dimweaver;

import java.util.ArrayList;
import java.util.List;

/**
 * The convex hulls of the points of the open segment shifted up by their tolerance (upper hull) and down by it
 * (lower hull), kept incrementally as points arrive in time order. The extreme lines of {@link DisjointPLA} pass
 * through their vertices, so only these vertices need to be checked when a point is added.
 */
class ConvexHull {
    private final List<TimeSeriesPoint> upperHull;
    private final List<TimeSeriesPoint> lowerHull;

    protected ConvexHull() {
        upperHull = new ArrayList<>();
        lowerHull = new ArrayList<>();
    }

    /** Adds (t, v + ε) to the upper hull and (t, v − ε) to the lower hull. */
    protected void addToHull(TimeSeriesPoint p, double epsilon) {
        upperHull.add(new TimeSeriesPoint(p.getT(), p.getV() + epsilon));
        lowerHull.add(new TimeSeriesPoint(p.getT(), p.getV() - epsilon));
        updateHull(upperHull, true);
        updateHull(lowerHull, false);
    }

    /** Cross product of (a − o) and (b − o): positive for a left (counter-clockwise) turn o → a → b. */
    private double crossProduct(TimeSeriesPoint o, TimeSeriesPoint a, TimeSeriesPoint b) {
        return (a.getT() - o.getT()) * (b.getV() - o.getV()) - (a.getV() - o.getV()) * (b.getT() - o.getT());
    }

    /** Removes the middle vertices that are no longer on the hull after the last point was appended. */
    private void updateHull(List<TimeSeriesPoint> hull, boolean isUpper) {
        while (hull.size() > 2) {
            int size = hull.size();
            TimeSeriesPoint a = hull.get(size - 3);
            TimeSeriesPoint b = hull.get(size - 2);
            TimeSeriesPoint c = hull.get(size - 1);

            // Collinear middle points are removed too (<= / >=): they never define the upper or lower line, and keeping
            // them made the hull grow to every point of a stationary object (quadratic time).
            if ((isUpper && crossProduct(a, b, c) <= 0) || (!isUpper && crossProduct(a, b, c) >= 0)) {
                hull.remove(size - 2);
            } else {
                break;
            }
        }
    }

    /** Number of vertices of the upper or lower hull. */
    protected int size(boolean upper) {
        if (upper) return upperHull.size();
        else return lowerHull.size();
    }

    /** Vertex {@code index} of the upper or lower hull, in time order. */
    protected TimeSeriesPoint get(int index, boolean upper) {
        if (upper) return upperHull.get(index);
        else return lowerHull.get(index);
    }

    /** Empties both hulls, for the next segment. */
    protected void clearHull() {
        upperHull.clear();
        lowerHull.clear();
    }
}
