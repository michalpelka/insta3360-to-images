# insta360-to-images

A dependency-free C++17 rewrite of
[InstaToBag](https://github.com/michalpelka/InstaToBag) that converts an Insta360
`.insv` capture into **per-camera folders of timestamped JPEG frames** instead of a
ROS 2 MCAP bag: both fisheye tracks, each frame named by its nanosecond timestamp on
the camera's own clock, plus an optional 1 kHz IMU CSV and per-lens intrinsics
sidecar.

Developed and verified against an **Insta360 X5** (firmware `v1.10.11_build1`, 5.7K
dual-fisheye, 2880x2880 per lens at 24 fps). The trailer format is shared across the
X-series, so ONE X2/X3/X4 files should work; the parsers degrade to "field not
present" rather than to wrong values when a firmware revision differs.

## Build

Requires a C++17 compiler, CMake >= 3.16, and `ffmpeg`/`ffprobe` on `PATH` at
*runtime* (not needed to build):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This produces `build/insta360-to-images` and, unless you pass
`-DINSTA360_BUILD_TESTS=OFF`, a unit test binary run via `ctest`:

```bash
ctest --test-dir build --output-on-failure
```

## Use

```bash
# See what's in a file without writing anything
insta360-to-images VID_20260909_132827_00_005.insv --inspect

# Convert (writes VID_20260909_132827_00_005/ beside the input)
insta360-to-images VID_20260909_132827_00_005.insv

# A quick look at a long clip, at a quarter resolution
insta360-to-images capture.insv -o small/ --max-frames 100 --scale 0.25
```

Run `insta360-to-images --help` for the full set. The useful ones:

| Flag | Effect |
| --- | --- |
| `--jpeg-quality N` | ffmpeg MJPEG quality, 2 (best) to 31. Default 3. |
| `--scale SPEC` | `0.5` or `1440x1440`. Intrinsics are rescaled to match. |
| `--max-frames N` | Stop after N frames. |
| `--swap-lenses` | Map the second video track to `cam_front`. |
| `--no-imu` / `--no-camera-info` | Leave out `imu.csv` / the intrinsics sidecar. |
| `--relative-time` | Start timestamps at zero instead of the capture wall clock. |
| `-f`, `--force` | Write into a non-empty output directory. |

## Output layout

```
<output>/
  cam_front/
    camera_info.json        # once, if lens calibration was present
    1736432112123456789.jpg # one per frame, named by its epoch nanosecond timestamp
    1736432112165123456.jpg
    ...
  cam_back/
    camera_info.json
    <timestamp_ns>.jpg
    ...
  imu.csv                   # timestamp_ns,accel_x_mps2,...,gyro_z_radps at ~1 kHz
```

With `--relative-time`, filenames and `imu.csv` timestamps count nanoseconds from the
start of the clip instead of epoch nanoseconds.

### `camera_info.json`

One per camera, written once, holding the same intrinsics the original tool put on
`sensor_msgs/CameraInfo`: `fx`/`fy`/`cx`/`cy`, the 3x3 `k`, 3x3 `r` (identity; no
stereo rectification is defined for a back-to-back fisheye pair), the 3x4 `p`, the
raw `distortion` coefficients under `distortion_model: "insta360_fisheye_v2"`, and the
per-lens `rotation_deg`/`translation` the camera reports (not a calibrated
IMU-to-camera extrinsic).

## Caveats — read before you use the data

Carried over unchanged from the original tool; nothing about the trailer format or
its ambiguities changed in this rewrite.

- **IMU axes are the sensor's own, not REP 103.** Accelerometer and gyroscope are
  converted to m/s² and rad/s, but they are *not* rotated into ROS's x-forward /
  y-left / z-up convention, because the mapping from the X-series sensor frame to the
  camera body frame is undocumented. Determine it for your rig before fusing.
- **No IMU-to-camera extrinsics.** The metadata carries per-lens rotations, but not a
  calibrated IMU-to-camera transform. The per-lens rotation and translation values are
  passed through verbatim in `camera_info.json`.
- **The distortion model is Insta360's, not OpenCV's.** `distortion_model` is
  `insta360_fisheye_v2` and `distortion` holds the camera's own five coefficients.
  They are *not* `plumb_bob` or `equidistant` and must not be fed to
  `cv::undistort`/`cv::fisheye` as if they were. `k` and `p` are ordinary pinhole
  intrinsics and are safe to use.
- **`cam_front` / `cam_back` is naming, not a determination.** They follow the order
  of the video tracks in the file, which is stable but has not been confirmed against
  which lens physically faces the screen. Use `--swap-lenses` if you need them the
  other way round.
- **JPEG re-encode is lossy.** Frames are decoded from HEVC and re-encoded as JPEG.
  Lower `--jpeg-quality` for more fidelity, or keep the original file for archival.
- **IMU covariances are unknown** and are simply not written; treat `imu.csv` as raw
  sensor readings, not a calibrated `sensor_msgs/Imu`-equivalent.

## The `.insv` format

Unchanged from the original project; see its
[README](https://github.com/michalpelka/InstaToBag#the-insv-format) for the full
trailer layout, timestamp derivation, and how the IMU axis/scale assignment was
physically validated. This rewrite reads the identical byte layout.

## Layout

| File | Role |
| --- | --- |
| `include/insta360/trailer.hpp`, `src/trailer.cpp` | Trailer tail, record directory, random access to records |
| `include/insta360/protobuf.hpp`, `src/protobuf.cpp` | Dependency-free protobuf wire decoder |
| `include/insta360/metadata.hpp`, `src/metadata.cpp` | Typed view over record `0x0101`, including the wall-clock mapping |
| `include/insta360/calibration.hpp`, `src/calibration.cpp` | Lens calibration strings -> rescaled intrinsics |
| `include/insta360/sensors.hpp`, `src/sensors.cpp` | IMU, exposure and frame-timestamp decoding |
| `include/insta360/subprocess.hpp`, `src/subprocess.cpp` | `posix_spawn`-based child process wrapper (no shell) |
| `include/insta360/media.hpp`, `src/media.cpp` | ffprobe/ffmpeg piping, JPEG frame splitting |
| `include/insta360/convert.hpp`, `src/convert.cpp` | Merges every stream into timestamped files on disk |
| `src/cli.cpp` | Argument parsing, `--inspect`, `main` |
| `tests/` | Unit tests against byte-exact synthetic trailers and protobuf messages |

## License

MIT

## Trademarks

This is an independent project, not affiliated with, endorsed by, or sponsored by
Arashi Vision Inc. "Insta360" and the product names above are trademarks of their
respective owners, used here only to identify the file format and hardware this tool
interoperates with. No Insta360 source code, SDK or firmware is included: the trailer
format was derived (in the original Python project this is ported from) by observing
captures from a camera its author owns.
