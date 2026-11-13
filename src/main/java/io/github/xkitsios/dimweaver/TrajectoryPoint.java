package io.github.xkitsios.dimweaver;

/**
 * A point of a trajectory.
 *
 * @param t timestamp: an integer in any unit (seconds, milliseconds, ...). Decompression computes with doubles, which
 *          are exact up to 2<sup>53</sup> (about 9·10<sup>15</sup>): enough for epoch milliseconds, not for epoch
 *          nanoseconds
 *
 * @param x first coordinate (for example longitude, or meters east)
 * @param y second coordinate (for example latitude, or meters north)
 */
public record TrajectoryPoint(long t, double x, double y) {
}
