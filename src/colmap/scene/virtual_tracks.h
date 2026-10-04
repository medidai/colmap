// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include "colmap/scene/reconstruction.h"
#include "colmap/util/eigen_alignment.h"
#include "colmap/util/types.h"

#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace colmap {

// Virtual tracks are 2D correspondences produced by an external geometry model
// (e.g. feed-forward depth + poses reprojected on a pixel grid). They are
// consumed only as additional, robustly weighted reprojection residuals in
// bundle adjustment: each track owns a free 3D point that is (re)triangulated
// from the current camera poses before every adjustment. Virtual tracks never
// take part in image registration, triangulation of the model, point filtering,
// or the written reconstruction.

struct VirtualObservation {
  image_t image_id = kInvalidImageId;
  // Image coordinates in COLMAP's pixel-centre convention (same as Point2D).
  Eigen::Vector2d xy = Eigen::Vector2d::Zero();
  // Scales the robust loss of this residual (typically model confidence).
  double weight = 1.0;
  // The generating model placed the point behind this camera and `xy` is the
  // antipodal projection (the pixel of the point reflected through the camera
  // centre). The residual then projects the negated point, as in GLUEMAP's
  // negative-depth cost: the observation still constrains the pose through the
  // reversed ray, even though nothing is visible at `xy`.
  bool negative = false;
  // Set before every bundle adjustment: whether the observation passed the
  // triangulation / angular error filter and is used as a residual.
  bool active = false;
};

struct VirtualTrack {
  // Free parameter block of the bundle adjustment; valid only when
  // `is_triangulated` is set.
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  bool is_triangulated = false;
  std::vector<VirtualObservation> observations;

  size_t NumActiveObservations() const;
};

// Text file contract (whitespace separated, `#` starts a comment line):
//
//   track_id  x  y  weight  image_name
//
// One observation per line; observations of one track share `track_id`.
// `image_name` is the image name as stored in the database (may contain
// path separators, must not contain whitespace). `x`, `y` follow the same
// pixel-centre convention as the keypoints in the database.
struct VirtualTrackFileObservation {
  int64_t track_id = -1;
  std::string image_name;
  Eigen::Vector2d xy = Eigen::Vector2d::Zero();
  double weight = 1.0;
  bool negative = false;
};

// File format: one observation per line,
//   track_id x y weight image_name [negative]
// with `negative` an optional 0/1 (default 0); lines starting with '#' are
// comments.
std::vector<VirtualTrackFileObservation> ReadVirtualTrackObservations(
    const std::string& path);

// Groups file observations by track id and resolves image names to image ids
// of the given reconstruction. Observations of unknown images and tracks with
// fewer than two remaining observations are dropped; the number of dropped
// observations is returned through `num_unresolved`, their image names
// through `unresolved_image_names`.
std::vector<VirtualTrack> ResolveVirtualTracks(
    const std::vector<VirtualTrackFileObservation>& observations,
    const Reconstruction& reconstruction,
    size_t* num_unresolved = nullptr,
    std::set<std::string>* unresolved_image_names = nullptr);

}  // namespace colmap
