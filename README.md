# DimWeaver

DimWeaver is a lossy compression algorithm for trajectories with a guaranteed maximum error ε.

It approximates each dimension of a trajectory separately with disjoint piecewise-linear segments and weaves the dimensions together, so that the error over all dimensions stays within ε. Segments whose slopes can share a single value are grouped and stored once.

## Usage

Requires Java 17+. With Maven:

```xml
<dependency>
    <groupId>io.github.xkitsios</groupId>
    <artifactId>dimweaver</artifactId>
    <version>1.0.0</version>
</dependency>
```

### With points

```java
import io.github.xkitsios.dimweaver.*;
import java.nio.file.*;

// A trajectory from a CSV file with lines "t,x,y" (timestamps strictly increasing)
List<TrajectoryPoint> points = Files.readAllLines(Path.of("trajectory.csv")).stream()
        .map(line -> line.split(","))
        .map(f -> new TrajectoryPoint(Long.parseLong(f[0]), Double.parseDouble(f[1]), Double.parseDouble(f[2])))
        .toList();

double epsilon = 1e-4;  // maximum error, in the units of the coordinates

// Compress and save
Files.write(Path.of("trajectory.dimweaver"), DimWeaver.compress(points, epsilon));

// Load and decompress
DecompressedTrajectory decompressed = DimWeaver.decompress(Files.readAllBytes(Path.of("trajectory.dimweaver")));

// Positions at given timestamps (for example the original ones)
long[] timestamps = points.stream().mapToLong(TrajectoryPoint::t).toArray();
List<TrajectoryPoint> atSamples = DimWeaver.evaluateAt(decompressed, timestamps);

// Position at a single time
TrajectoryPoint at = DimWeaver.positionAt(decompressed, timestamps[0]);

// The trajectory as a polyline, without needing the original timestamps
List<TrajectoryPoint> vertices = DimWeaver.reconstructTrajectory(decompressed);
```

### With arrays

The same methods, here with the trajectory as arrays:

```java
import io.github.xkitsios.dimweaver.*;
import java.nio.file.*;

// A trajectory from a CSV file with lines "t,x,y" (timestamps strictly increasing)
List<String> lines = Files.readAllLines(Path.of("trajectory.csv"));
long[] t = lines.stream().mapToLong(line -> Long.parseLong(line.split(",")[0])).toArray();
double[] x = lines.stream().mapToDouble(line -> Double.parseDouble(line.split(",")[1])).toArray();
double[] y = lines.stream().mapToDouble(line -> Double.parseDouble(line.split(",")[2])).toArray();

double epsilon = 1e-4;  // maximum error, in the units of the coordinates

// Compress and save
Files.write(Path.of("trajectory.dimweaver"), DimWeaver.compress(t, x, y, epsilon));

// Load, decompress, and get the coordinates at the original timestamps as arrays: {x[], y[]}
DecompressedTrajectory decompressed = DimWeaver.decompress(Files.readAllBytes(Path.of("trajectory.dimweaver")));
double[][] coordinates = DimWeaver.coordinatesAt(decompressed, t);
```

Timestamps are non-negative whole numbers (Java `long`) in any unit, e.g. seconds or milliseconds. A trajectory may span up to 2³¹ − 1 time units (about 24.8 days in milliseconds). Every decompressed position at an original timestamp is within ε
of the original one. Compression and decompression are thread-safe.

> For better quality, with a small cost in space, `DimWeaver.compress(points, epsilon, ErrorBound.CONTINUOUS)` keeps
> the error within ε at every time, also between samples.

## Building

Requires Java 17+ and Maven.

```sh
mvn test      # runs the tests
mvn install   # installs the jar in the local Maven repository
```

## Citation

DimWeaver is described in:

- **[DimWeaver: Dimensional Weaved Trajectory Compression](https://xkitsios.github.io/assets/pdf/dimweaver-sigmod26.pdf)** (2026)  
  X. Kitsios, P. Liakos, K. Papakonstantinopoulou, N. Giatrakos, Y. Kotidis  
  _Proceedings of the ACM on Management of Data (SIGMOD)_, Vol. 4, No. 3, Article 163  
  DOI: [10.1145/3802040](https://doi.org/10.1145/3802040)

If you use DimWeaver in your work, please cite it:

```bibtex
@article{dimweaver,
  author = {Kitsios, Xenophon and Liakos, Panagiotis and Papakonstantinopoulou, Katia and Giatrakos, Nikos and Kotidis, Yannis},
  title = {DimWeaver: Dimensional Weaved Trajectory Compression},
  journal = {Proc. ACM Manag. Data (SIGMOD)},
  year = {2026},
  month = may,
  volume = {4},
  number = {3},
  numpages = {26},
  articleno = {163},
  issue_date = {June 2026},
  publisher = {Association for Computing Machinery},
  address = {New York, NY, USA},
  doi = {10.1145/3802040},
  url = {https://doi.org/10.1145/3802040},
  keywords = {trajectory simplification},
}
```

## See also

- [Mix-Piece / Sim-Piece](https://github.com/xkitsios/Mix-Piece_Sim-Piece): lossy compression of time series with a
  maximum absolute error.
- [CHIMP](https://github.com/panagiotisl/chimp): lossless compression of floating-point time series.

## Contact

- [Xenophon Kitsios](https://xkitsios.github.io/)
- [Panagiotis Liakos](https://cgi.di.uoa.gr/~p.liakos/)
- [Katia Papakonstantinopoulou](https://www2.aueb.gr/users/katia/)
- [Nikos Giatrakos](http://users.softnet.tuc.gr/~ngiatrakos/)
- [Yannis Kotidis](https://pages.aueb.gr/users/kotidis/)

## License

Apache License 2.0, see [LICENSE](LICENSE).
