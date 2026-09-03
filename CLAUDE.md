# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A fork of [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) v1.0 maintained by Lanka Robotics. Upstream is a visual / visual-inertial / multi-map SLAM library (monocular, stereo, RGB-D; pinhole and Kannala-Brandt fisheye). It builds as a shared library `lib/libORB_SLAM3.so` plus a set of dataset/camera example executables.

The fork's own addition on top of upstream is **feature-detector masking** (see "Fork-specific changes" below). Keep changes backward compatible with upstream config files.

## Build

```bash
./build.sh        # builds Thirdparty/{DBoW2,g2o,Sophus}, untars the vocabulary, then the library + all Examples/*
./build_ros.sh    # optional: builds Examples/ROS/ORB_SLAM3 nodes (needs ROS_PACKAGE_PATH set, ROS Melodic-era)
```

`build.sh` is not incremental-aware — it always `mkdir build` then `cmake .. -DCMAKE_BUILD_TYPE=Release && make`. For iterative work on the main library, after the first full build just run `make -jN` inside `build/`. Re-run the full `build.sh` only when a Thirdparty lib or the vocabulary changed.

Requirements: C++11, OpenCV **>= 4.4** (CMake hard-fails otherwise), Eigen3 >= 3.1, Pangolin, Boost.serialization, OpenSSL (`-lcrypto`, used for atlas checksums). `realsense2` is optional — if not found, the RealSense examples are silently skipped. `CMAKE_CXX_FLAGS_RELEASE` adds `-march=native`, so binaries are not portable across machines.

Upstream targets Ubuntu 16.04/18.04. This checkout is on macOS; expect to patch include paths / linker flags for local builds.

## Tests

There is no unit test suite. Correctness is validated by running dataset sequences and comparing trajectories to ground truth:

- `evaluation/evaluate_ate_scale.py` — RMS ATE with Sim(3) alignment (the metric upstream reports).
- `evaluation/associate.py` — timestamp association for TUM-format files.
- `evaluation/Ground_truth/` — EuRoC ground truth transformed to the left-camera frame (for pure-visual runs).

Example runners referenced in `README.md` (`euroc_examples.sh`, `tum_vi_examples.sh`, and their `*_eval_*` variants) are not checked in; construct the invocation manually from `CMakeLists.txt` target names.

Typical manual run (executable, vocabulary, settings YAML, then dataset args):

```bash
./Examples/Stereo-Inertial/stereo_inertial_euroc Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml <seq_dir> <timestamps_file>
```

`Vocabulary/ORBvoc.txt` must exist (untarred by `build.sh` from `ORBvoc.txt.tar.gz`). Trajectories are written by `System::SaveTrajectory*` / `SaveKeyFrameTrajectory*`, which must be called after `System::Shutdown()`.

## Architecture

### Threading model
`System` (`src/System.cc`) is the entry point. Its constructor loads the vocabulary + settings and spawns three threads:
- **LocalMapping** (`src/LocalMapping.cc`) — local bundle adjustment, keyframe culling, MapPoint creation, IMU initialization.
- **LoopClosing** (`src/LoopClosing.cc`) — loop & map-merge detection via DBoW2, Sim(3)/SE(3) pose-graph optimization, and full BA in a nested thread.
- **Viewer** (`src/Viewer.cc`) — Pangolin GUI (disable with `bUseViewer=false`).

**Tracking** (`src/Tracking.cc`) runs on the *caller's* thread — it executes inside `TrackMonocular` / `TrackStereo` / `TrackRGBD`. Cross-thread hand-off is via queues guarded by mutexes on each component; expect to reason about lock order when touching `KeyFrame` / `MapPoint` / `Map`.

### Data flow per image
`System::Track*` → build a `Frame` (`src/Frame.cc`), which runs `ORBextractor` (`src/ORBextractor.cc`, a modified OpenCV `orb.cpp` with an image pyramid + octree feature-count balancing) → `Tracking` estimates a pose (motion model / reference keyframe / relocalization), decides on keyframe insertion → new `KeyFrame` goes to `LocalMapping` → then `LoopClosing`.

### Map storage
`Atlas` (`src/Atlas.cc`) owns multiple `Map`s (the "multi-map" in ORB-SLAM3). One map is active; others are inactive until a merge. `Map` holds `KeyFrame*` and `MapPoint*`. `KeyFrameDatabase` (DBoW2 inverted index) serves relocalization and loop candidates. Atlas save/load is binary (Boost.serialization) with an OpenSSL checksum; configured via `System.LoadAtlasFromFile` / `System.SaveAtlasToFile` YAML keys.

### Optimization
All g2o glue lives in `src/Optimizer.cc` with custom vertex/edge types in `src/G2oTypes.cc` (inertial) and `src/OptimizableTypes.cpp` (reprojection). `src/MLPnPsolver.cpp` and `src/Sim3Solver.cc` are the RANSAC pose/similarity solvers. Poses use Sophus `SE3f` / `Sim3f` (`Thirdparty/Sophus`), not raw cv::Mat, since v1.0.

### Camera models
`include/CameraModels/GeometricCamera.h` is the interface; `Pinhole` and `KannalaBrandt8` (fisheye) are the implementations. Stereo fisheye is handled as two `KannalaBrandt8` cameras with a relative pose, not by rectification.

### Configuration
Two parser generations coexist:
- **`src/Settings.cc`** — the v1.0 YAML format (documented in `Calibration_Tutorial.pdf`); used by everything under `Examples/`. Handles undistort/rectify/resize map precomputation.
- **`src/Config.cc` + legacy inline parsing** — older path used by `Examples_old/`. `CMakeLists.txt` builds both trees (`*_old` target suffix). New work goes through `Settings`.

`include/Config.h`: uncomment `#define REGISTER_TIMES` to emit per-stage timing (also gates `#ifdef REGISTER_TIMES` code paths across the library). Verbosity is the `Verbose` class in `include/System.h`.

### Thirdparty (vendored, built separately by `build.sh`)
`DBoW2` (place recognition), `g2o` (optimization — a pinned modified copy, do not `apt install` over it), `Sophus` (Lie groups, header-mostly). Changes here need the Thirdparty lib rebuilt before the main library picks them up.

## Fork-specific changes

**Feature-detector masking** (`feature/orb-feature-mask`, commits `8c1f660`, `005353f`): exclude static image regions (e.g. the robot's own body in frame) from ORB extraction.

- New optional YAML keys `Camera.mask` and `Camera2.mask` — paths to 8-bit single-channel images drawn against a **raw, unrectified** frame. Non-zero = eligible for features, zero = excluded.
- `Settings::loadMasks` (`src/Settings.cc`) loads them **last** in the ctor (after rectification maps / `newImSize_` are final) and warps each mask into the exact pixel space the extractor sees — `cv::remap` with the rectification maps if rectifying, else `cv::resize`, both `INTER_NEAREST`. It **throws** on an unreadable mask path (was `exit(-1)` before `005353f`).
- `Settings::mask1()` / `mask2()` expose them; `Tracking` holds them and threads them through every `Frame` construction site into `Frame::ExtractORB` → `ORBextractor::operator()`.
- `ORBextractor` builds a per-pyramid-level mask and drops FAST corners on masked pixels *before* octree budget distribution (so budget isn't spent on corners that would be discarded). Pixel values are never modified, so no edge artifacts along the mask boundary.
- Fully backward compatible: no `Camera.mask` key ⇒ empty `cv::Mat` ⇒ upstream behavior.

Example YAMLs documenting the keys: `Examples/{Monocular/EuRoC,Stereo/EuRoC,RGB-D/RealSense_D435i}.yaml`.
