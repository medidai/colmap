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

#include "colmap/feature/types.h"

#include "colmap/util/logging.h"

namespace colmap {

FeatureKeypoint::FeatureKeypoint() : FeatureKeypoint(0, 0, 1, -1) {}

FeatureKeypoint::FeatureKeypoint(const float x,
                                 const float y,
                                 const float weight_)
    : FeatureKeypoint(x, y, weight_, -1) {}

FeatureKeypoint::FeatureKeypoint(const float x,
                                 const float y,
                                 const float weight_,
                                 const int constraint_point_id_)
    : FeatureKeypoint(x, y, weight_, constraint_point_id_, 1, 0, 0, 1) {}

FeatureKeypoint::FeatureKeypoint(const float x_,
                                 const float y_,
                                 const float weight_,
                                 const int constraint_point_id_,
                                 const float scale,
                                 const float orientation)
    : x(x_), y(y_) {
  THROW_CHECK_GE(scale, 0.0);
  const float scale_cos_orientation = scale * std::cos(orientation);
  const float scale_sin_orientation = scale * std::sin(orientation);
  a11 = scale_cos_orientation;
  a12 = -scale_sin_orientation;
  a21 = scale_sin_orientation;
  a22 = scale_cos_orientation;
  weight = weight_;
  constraint_point_id = constraint_point_id_;
}

FeatureKeypoint::FeatureKeypoint(const float x_,
                                 const float y_,
                                 const float weight_,
                                 const int constraint_point_id_,
                                 const float a11_,
                                 const float a12_,
                                 const float a21_,
                                 const float a22_)
    : x(x_),
      y(y_),
      weight(weight_),
      constraint_point_id(constraint_point_id_),
      a11(a11_),
      a12(a12_),
      a21(a21_),
      a22(a22_) {}

FeatureKeypoint FeatureKeypoint::FromShapeParameters(
    const float x,
    const float y,
    const float weight,
    const int constraint_point_id_,
    const float scale_x,
    const float scale_y,
    const float orientation,
    const float shear) {
  THROW_CHECK_GE(scale_x, 0.0);
  THROW_CHECK_GE(scale_y, 0.0);
  return FeatureKeypoint(x,
                         y,
                         weight,
                         constraint_point_id_,
                         scale_x * std::cos(orientation),
                         -scale_y * std::sin(orientation + shear),
                         scale_x * std::sin(orientation),
                         scale_y * std::cos(orientation + shear));
}

void FeatureKeypoint::Rescale(const float scale) { Rescale(scale, scale); }

void FeatureKeypoint::Rescale(const float scale_x, const float scale_y) {
  THROW_CHECK_GT(scale_x, 0);
  THROW_CHECK_GT(scale_y, 0);
  x *= scale_x;
  y *= scale_y;
  a11 *= scale_x;
  a12 *= scale_y;
  a21 *= scale_x;
  a22 *= scale_y;
}

float FeatureKeypoint::ComputeScale() const {
  return (ComputeScaleX() + ComputeScaleY()) / 2.0f;
}

float FeatureKeypoint::ComputeScaleX() const {
  return std::sqrt(a11 * a11 + a21 * a21);
}

float FeatureKeypoint::ComputeScaleY() const {
  return std::sqrt(a12 * a12 + a22 * a22);
}

float FeatureKeypoint::ComputeOrientation() const {
  return std::atan2(a21, a11);
}

float FeatureKeypoint::ComputeShear() const {
  return std::atan2(-a12, a22) - ComputeOrientation();
}

}  // namespace colmap
