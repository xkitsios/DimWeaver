package io.github.xkitsios.dimweaver;

import java.util.*;

/**
 * Kinematic grouping: segments that can share a slope, i.e. move with the same velocity, are stored once as a group
 * with a single slope. Segments of both dimensions are grouped together.
 */
class KinematicGrouping {
    /** A group: its slope (exactly representable as a float) and its segments, sorted by start time. */
    record Group(double slope, List<Segment> segments) {
    }

    /**
     * Groups segments whose slope intervals share a common slope. The map key is the group's slope; it is always
     * exactly representable as a float (the binary stores it as one), so encoder and decoder use the same value.
     * <p>
     * Segments are sorted by their smallest slope and added greedily to the current group while the intersection of
     * the slope intervals still contains a float. Stationary segments (slope interval exactly [0, 0]) all go to the
     * group of slope 0. Sorts {@code segments} in place.
     * <p>
     * The groups are returned in a fixed order, by the start time of their first segment and then by slope, so the
     * binary never depends on hash order.
     */
    public static List<Group> group(List<Segment> segments) {
        Map<Double, List<Segment>> groupedSegments = new HashMap<>();
        double aLowerGroup = -Double.MAX_VALUE;
        double aUpperGroup = Double.MAX_VALUE;
        ArrayList<Segment> group = new ArrayList<>();

        segments.sort(Comparator.comparingDouble(Segment::getSmallestSlope));

        for (Segment plaSegment : segments) {
            if (plaSegment.getSmallestSlope() == 0 && plaSegment.getBiggestSlope() == 0) {
                List<Segment> lastPoint = groupedSegments.getOrDefault(0.0, new ArrayList<>());
                lastPoint.add(plaSegment);
                groupedSegments.put(0.0, lastPoint);
                continue;
            }
            double newLower = Math.max(aLowerGroup, plaSegment.getSmallestSlope());
            double newUpper = Math.min(aUpperGroup, plaSegment.getBiggestSlope());
            if (newLower <= newUpper && FloatSlope.containsFloat(newLower, newUpper)) {
                aLowerGroup = newLower;
                aUpperGroup = newUpper;
                group.add(plaSegment);
            } else {
                closeGroup(groupedSegments, group, aLowerGroup, aUpperGroup);
                group = new ArrayList<>();
                group.add(plaSegment);
                aLowerGroup = plaSegment.getSmallestSlope();
                aUpperGroup = plaSegment.getBiggestSlope();
            }
        }
        if (!group.isEmpty())
            closeGroup(groupedSegments, group, aLowerGroup, aUpperGroup);

        List<Group> groups = new ArrayList<>(groupedSegments.size());
        for (Map.Entry<Double, List<Segment>> entry : groupedSegments.entrySet()) {
            entry.getValue().sort(Comparator.comparingLong(Segment::getStart));
            groups.add(new Group(entry.getKey(), entry.getValue()));
        }
        groups.sort(Comparator.comparingLong((Group g) -> g.segments().get(0).getStart()).thenComparingDouble(Group::slope));

        return groups;
    }

    /** Stores the group under a float slope within [aLower, aUpper], merging it with a group of the same slope. */
    private static void closeGroup(Map<Double, List<Segment>> groupedSegments, List<Segment> group, double aLower, double aUpper) {
        double slope = FloatSlope.floatWithin(aLower, aUpper);
        List<Segment> existing = groupedSegments.get(slope);
        if (existing != null) group.addAll(existing);
        groupedSegments.put(slope, group);
    }
}
