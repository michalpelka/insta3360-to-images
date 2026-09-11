// Parsing of the Insta360 lens calibration strings found in the metadata record.
//
// The camera stores calibration as underscore-joined decimal text. The variant that
// matters here is "offset_v2" (metadata field 54/56), one group of 19 values per lens,
// which carries separate fx/fy plus a distortion polynomial:
//
//   <n>_ [ <type> <fx> <fy> <cx> <cy> <rx> <ry> <rz> <tx> <ty> <tz>
//          <k1> <k2> <k3> <k4> <k5> <canvas_w> <canvas_h> <crop> ] * n _<trailer>
//
// All pixel coordinates are expressed on a *stitched dual-fisheye canvas* of
// canvas_w x canvas_h in which each lens occupies a canvas_w/2 wide square, so lens i
// has its principal point offset by i * canvas_w/2. Frames stored in the file are
// usually smaller than that canvas, so the intrinsics are rescaled to the actual frame
// size on the way out.
//
// The distortion coefficients are Insta360's own polynomial, *not* OpenCV's
// plumb_bob or equidistant. They are exposed verbatim under a distortion model name
// of "insta360_fisheye_v2" so that no consumer mistakes them for a standard model.
#pragma once

#include <array>
#include <string>
#include <vector>

namespace insta360 {

constexpr const char* DISTORTION_MODEL = "insta360_fisheye_v2";

// Intrinsics for one fisheye lens, rescaled to the stored frame size.
struct LensCalibration {
    int index = 0;
    int width = 0;
    int height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    std::vector<double> distortion;          // 5 coefficients, Insta360's own model
    std::array<double, 3> rotation_deg{};    // lens yaw/pitch/roll, as reported
    std::array<double, 3> translation{};     // lens translation, in canvas pixels
    std::array<int, 2> canvas{};             // canvas the raw values were expressed on
    std::array<double, 2> scale{};           // rescaling applied to reach width x height

    std::array<double, 9> k() const { return {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0}; }
    // No stereo rectification is defined for a back-to-back fisheye pair.
    std::array<double, 9> r() const { return {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}; }
    std::array<double, 12> p() const {
        return {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    }
};

// Parses a field-54/56 calibration string, rescaled to width x height. Returns an
// empty vector if the string does not have the expected shape, so that a firmware
// revision with a different layout degrades to "no calibration" rather than to
// silently wrong intrinsics.
std::vector<LensCalibration> parse_offset_v2(const std::string& text, int width, int height);

std::string summarise(const LensCalibration& lens);

}  // namespace insta360
