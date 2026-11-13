package io.github.xkitsios.dimweaver;

/**
 * Where the error bound ε is guaranteed. The binary format and decompression are the same for both.
 */
public enum ErrorBound {
    /**
     * At the original samples: evaluating the decompressed trajectory at any original timestamp is within ε.
     */
    SAMPLES,
    /**
     * At every time: the decompressed trajectory is within ε of the linearly interpolated original trajectory,
     * also between samples. Costs compression ratio: segments are extended or started with virtual points on the
     * linear interpolation between samples, so that no segment is extrapolated over a gap.
     */
    CONTINUOUS
}
