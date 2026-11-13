package io.github.xkitsios.dimweaver;

import io.github.xkitsios.dimweaver.utils.Reader;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class TestConcurrency {
    private static final double EPSILON = 1e-4;
    private static final int THREADS = 8;
    private static final int ROUNDS = 3;

    /**
     * Compressing from many threads at once, each in a different order, must give exactly the sequential bytes. This
     * covers thread safety and determinism: FastPFOR keeps internal buffers, which used to break shared use across
     * threads and also leaked leftovers of earlier calls into the padding of the output.
     */
    @Test
    public void concurrentCompressionMatchesSequential() throws Exception {
        List<List<TrajectoryPoint>> trajectories = new ArrayList<>();
        for (String file : Reader.list("src/test/resources/geolife"))
            trajectories.add(Reader.parse(file));
        List<byte[]> expected = new ArrayList<>();
        for (List<TrajectoryPoint> trajectory : trajectories)
            expected.add(DimWeaver.compress(trajectory, EPSILON));

        ExecutorService executor = Executors.newFixedThreadPool(THREADS);
        try {
            List<Future<?>> futures = new ArrayList<>();
            for (int thread = 0; thread < THREADS; thread++) {
                int offset = thread; // each thread walks the trajectories in a different order
                futures.add(executor.submit(() -> {
                    for (int round = 0; round < ROUNDS; round++) {
                        for (int k = 0; k < trajectories.size(); k++) {
                            int i = (k + offset * 17) % trajectories.size();
                            byte[] binary = DimWeaver.compress(trajectories.get(i), EPSILON);
                            assertArrayEquals(expected.get(i), binary, "trajectory " + i);
                            DecompressedTrajectory decompressed = DimWeaver.decompress(binary);
                            assertEquals(trajectories.get(i).get(0).t(), decompressed.firstTimestamp());
                        }
                    }
                    return null;
                }));
            }
            for (Future<?> future : futures)
                future.get(); // rethrows assertion failures from the threads
        } finally {
            executor.shutdownNow();
        }
    }
}
