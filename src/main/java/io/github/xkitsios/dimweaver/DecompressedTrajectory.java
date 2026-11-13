package io.github.xkitsios.dimweaver;

import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * A decompressed trajectory: one piecewise-linear function of time per dimension. Evaluate it with
 * {@link DimWeaver#evaluateAt} or turn it into points with {@link DimWeaver#reconstructTrajectory}.
 */
public final class DecompressedTrajectory {
    private final List<Line> linesX;
    private final List<Line> linesY;
    private final long firstTimestamp;
    private final long lastTimestamp;

    /** Lines of each dimension sorted by start time; a line holds until the next one starts. */
    DecompressedTrajectory(List<Line> linesX, List<Line> linesY, long firstTimestamp, long lastTimestamp) {
        this.linesX = Collections.unmodifiableList(linesX);
        this.linesY = Collections.unmodifiableList(linesY);
        this.firstTimestamp = firstTimestamp;
        this.lastTimestamp = lastTimestamp;
    }

    List<Line> linesX() {
        return linesX;
    }

    List<Line> linesY() {
        return linesY;
    }

    /**
     * Timestamp of the first point of the original trajectory.
     *
     * @return the first timestamp
     */
    public long firstTimestamp() {
        return firstTimestamp;
    }

    /**
     * Timestamp of the last point of the original trajectory.
     *
     * @return the last timestamp
     */
    public long lastTimestamp() {
        return lastTimestamp;
    }

    /**
     * Number of stored line segments, over both dimensions.
     *
     * @return the number of segments
     */
    public int segmentCount() {
        return linesX.size() + linesY.size();
    }

    /** Equal if both have the same time range and the same lines (start, slope and starting value) per dimension. */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof DecompressedTrajectory other)) return false;
        return firstTimestamp == other.firstTimestamp && lastTimestamp == other.lastTimestamp
                && sameLines(linesX, other.linesX) && sameLines(linesY, other.linesY);
    }

    private static boolean sameLines(List<Line> a, List<Line> b) {
        if (a.size() != b.size()) return false;
        for (int i = 0; i < a.size(); i++) {
            Line s = a.get(i), t = b.get(i);
            if (Double.compare(s.getTStart(), t.getTStart()) != 0 || Double.compare(s.getA(), t.getA()) != 0
                    || Double.compare(s.getB(), t.getB()) != 0) return false;
        }
        return true;
    }

    @Override
    public int hashCode() {
        int hash = Objects.hash(firstTimestamp, lastTimestamp);
        for (List<Line> lines : List.of(linesX, linesY))
            for (Line line : lines)
                hash = 31 * hash + Objects.hash(line.getTStart(), line.getA(), line.getB());
        return hash;
    }

    @Override
    public String toString() {
        return "DecompressedTrajectory[firstTimestamp=" + firstTimestamp + ", lastTimestamp=" + lastTimestamp
                + ", segments=" + segmentCount() + "]";
    }
}
