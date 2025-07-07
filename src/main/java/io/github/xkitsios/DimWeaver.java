package io.github.xkitsios;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

public class DimWeaver {
    public static byte[] compress(List<TrajectoryPoint> points, double epsilon, double c) throws IOException {
        return Binary.encode(dimWeaver(points, epsilon, c), points.getFirst().getT());
    }

    public static List<TrajectoryPoint> decompress(byte[] binary) throws IOException {
        return reconstruct(Binary.decode(binary));
    }

    private static Map<Double, List<Segment>> dimWeaver(List<TrajectoryPoint> points, double epsilon, double c) {
        DisjointPLA disjointPLAX = new DisjointPLA(false), disjointPLAY = new DisjointPLA(true);
        List<TimeSeriesPoint> bufferX = new ArrayList<>();
        List<TimeSeriesPoint> bufferY = new ArrayList<>();
        double exponentialDecayX = 0, exponentialDecayY = 0;
        boolean residualY = false; // true: residual@Y, false: residual@X
        boolean added;
        for (TrajectoryPoint point : points) {
            TimeSeriesPoint pointX = new TimeSeriesPoint(point.getT(), point.getX()), pointY = new TimeSeriesPoint(point.getT(), point.getY());
            bufferX.add(pointX);
            bufferY.add(pointY);
            added = (residualY) ? disjointPLAX.add(pointX, epsilon / Math.sqrt(2)) : disjointPLAY.add(pointY, epsilon / Math.sqrt(2));
            if (!added) {
                if (residualY) exponentialDecayX = exponentialDecayX * (1 - c) + disjointPLAX.segmentSize() * c;
                else exponentialDecayY = exponentialDecayY * (1 - c) + disjointPLAY.segmentSize() * c;
                Segment lastSegment = (residualY) ? disjointPLAX.getSegments().getLast() : disjointPLAY.getSegments().getLast();
                List<TimeSeriesPoint> bufferUncompressed = (residualY) ? bufferY : bufferX;
                List<TimeSeriesPoint> bufferCompressed = (residualY) ? bufferX : bufferY;
                for (int i = 0; i < bufferUncompressed.size() - 1; i++) {
                    added = (residualY) ? disjointPLAY.add(bufferUncompressed.get(i), calculateResidual(lastSegment, bufferCompressed.get(i), epsilon)) : disjointPLAX.add(bufferUncompressed.get(i), calculateResidual(lastSegment, bufferCompressed.get(i), epsilon));
                    if (!added) {
                        if (residualY) exponentialDecayY = exponentialDecayY * (1 - c) + disjointPLAY.segmentSize() * c;
                        else exponentialDecayX = exponentialDecayX * (1 - c) + disjointPLAX.segmentSize() * c;

                        if (residualY)
                            disjointPLAY.add(bufferUncompressed.get(i), calculateResidual(lastSegment, bufferCompressed.get(i), epsilon));
                        else
                            disjointPLAX.add(bufferUncompressed.get(i), calculateResidual(lastSegment, bufferCompressed.get(i), epsilon));
                    }
                }
                residualY = exponentialDecayX * c > exponentialDecayY * c;
                added = (residualY) ? disjointPLAX.add(pointX, epsilon / Math.sqrt(2)) : disjointPLAY.add(pointY, epsilon / Math.sqrt(2));
                if (!added) {
                    if (residualY) exponentialDecayX = exponentialDecayX * (1 - c) + disjointPLAX.segmentSize() * c;
                    else exponentialDecayY = exponentialDecayY * (1 - c) + disjointPLAY.segmentSize() * c;
                    residualY = exponentialDecayX * c > exponentialDecayY * c;
                    if (residualY) disjointPLAX.add(pointX, epsilon / Math.sqrt(2));
                    else disjointPLAY.add(pointY, epsilon / Math.sqrt(2));
                }

                bufferX.clear();
                bufferX.add(pointX);
                bufferY.clear();
                bufferY.add(pointY);
            }
        }

        if (residualY) {
            disjointPLAX.closeSegment();
            Segment lastSegment = disjointPLAX.getSegments().getLast();
            for (int i = 0; i < bufferY.size(); i++)
                if (!disjointPLAY.add(bufferY.get(i), calculateResidual(lastSegment, bufferX.get(i), epsilon)))
                    disjointPLAY.add(bufferY.get(i), calculateResidual(lastSegment, bufferX.get(i), epsilon));
            disjointPLAY.closeSegment();
        } else {
            disjointPLAY.closeSegment();
            Segment lastSegment = disjointPLAY.getSegments().getLast();
            for (int i = 0; i < bufferX.size(); i++)
                if (!disjointPLAX.add(bufferX.get(i), calculateResidual(lastSegment, bufferY.get(i), epsilon)))
                    disjointPLAX.add(bufferX.get(i), calculateResidual(lastSegment, bufferY.get(i), epsilon));
            disjointPLAX.closeSegment();
        }

        disjointPLAX.getSegments().addAll(disjointPLAY.getSegments());

        return KinematicGrouping.group(disjointPLAX.getSegments());
    }

    private static double calculateResidual(Segment segment, TimeSeriesPoint point, double epsilon) {
        double maxError = Math.max(Math.abs(segment.getUpper().valueAt(point.getT()) - point.getV()), Math.abs(segment.getLower().valueAt(point.getT()) - point.getV()));

        return Math.max(Math.sqrt(epsilon * epsilon - maxError * maxError), epsilon / Math.sqrt(2));
    }

    private static List<TrajectoryPoint> reconstruct(List<Segment> segments) {
        List<TrajectoryPoint> points = new ArrayList<>();

        segments.sort(Comparator.comparingLong(Segment::getStart));

        Segment segmentX = null, segmentY = null;
        for (int i = 0; i < segments.size(); i++) {
            if (segments.get(i).getIsY()) segmentY = segments.get(i);
            else segmentX = segments.get(i);
            if (segmentX == null || segmentY == null || (i + 1 != segments.size() && segments.get(i).getStart() == segments.get(i + 1).getStart()))
                continue;
            points.add(new TrajectoryPoint(segments.get(i).getStart(), segmentX.getMiddle().valueAt(segments.get(i).getStart()), segmentY.getMiddle().valueAt(segments.get(i).getStart())));
            if (i + 1 != segments.size())
                points.add(new TrajectoryPoint(segments.get(i + 1).getStart() - 1, segmentX.getMiddle().valueAt(segments.get(i + 1).getStart() - 1), segmentY.getMiddle().valueAt(segments.get(i + 1).getStart() - 1)));
        }

        return points;
    }
}