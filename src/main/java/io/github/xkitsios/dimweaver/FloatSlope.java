package io.github.xkitsios.dimweaver;

/**
 * Slopes are stored as floats. A segment's (or group's) line is within the error bound only if its slope lies in the
 * segment's slope interval, so an interval is usable only if it contains a float. DisjointPLA and KinematicGrouping
 * share these checks so that they never disagree.
 */
final class FloatSlope {
    private FloatSlope() {
    }

    /** Whether [lower, upper] contains a value exactly representable as a float. */
    static boolean containsFloat(double lower, double upper) {
        float f = (float) lower;
        if (f < lower) f = Math.nextUp(f);
        return f <= upper;
    }

    /**
     * The float closest to the angle bisector of [lower, upper] that lies inside the interval. Callers guarantee the
     * interval contains a float ({@link #containsFloat}); otherwise the nearest float is returned.
     */
    static double floatWithin(double lower, double upper) {
        float f = (float) Math.tan((Math.atan(lower) + Math.atan(upper)) / 2);
        if (f >= lower && f <= upper) return f;
        // Rounded out of the interval: the float inside it that is closest to that side
        if (f < lower) {
            float first = (float) lower;
            return first < lower ? Math.nextUp(first) : first;
        }
        float last = (float) upper;
        return last > upper ? Math.nextDown(last) : last;
    }
}
