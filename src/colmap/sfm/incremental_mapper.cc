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

#include "colmap/sfm/incremental_mapper.h"

#include "colmap/estimators/pose.h"
#include "colmap/estimators/triangulation.h"
#include "colmap/estimators/two_view_geometry.h"
#include "colmap/geometry/triangulation.h"
#include "colmap/math/math.h"
#include "colmap/scene/projection.h"
#include "colmap/sensor/bitmap.h"
#include "colmap/util/misc.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <tuple>
#include <unordered_map>

namespace colmap {
namespace {

void SortAndAppendNextImages(std::vector<std::pair<image_t, float>> image_ranks,
                             std::vector<image_t>* sorted_images_ids) {
  std::sort(image_ranks.begin(),
            image_ranks.end(),
            [](const std::pair<image_t, float>& image1,
               const std::pair<image_t, float>& image2) {
              return image1.second > image2.second;
            });

  sorted_images_ids->reserve(sorted_images_ids->size() + image_ranks.size());
  for (const auto& image : image_ranks) {
    sorted_images_ids->push_back(image.first);
  }

  image_ranks.clear();
}

float RankNextImageMaxVisiblePointsNum(
    const image_t image_id, const class ObservationManager& obs_manager) {
  return static_cast<float>(obs_manager.NumVisiblePoints3D(image_id));
}

float RankNextImageMaxVisiblePointsRatio(
    const image_t image_id, const class ObservationManager& obs_manager) {
  return static_cast<float>(obs_manager.NumVisiblePoints3D(image_id)) /
         static_cast<float>(obs_manager.NumObservations(image_id));
}

float RankNextImageMinUncertainty(const image_t image_id,
                                  const class ObservationManager& obs_manager) {
  return static_cast<float>(obs_manager.Point3DVisibilityScore(image_id));
}

}  // namespace

bool IncrementalMapper::Options::Check() const {
  CHECK_OPTION_GT(init_min_num_inliers, 0);
  CHECK_OPTION_GT(init_max_error, 0.0);
  CHECK_OPTION_GE(init_max_forward_motion, 0.0);
  CHECK_OPTION_LE(init_max_forward_motion, 1.0);
  CHECK_OPTION_GE(init_min_tri_angle, 0.0);
  CHECK_OPTION_GE(init_max_reg_trials, 1);
  CHECK_OPTION_GT(abs_pose_max_error, 0.0);
  CHECK_OPTION_GT(abs_pose_min_num_inliers, 0);
  CHECK_OPTION_GE(abs_pose_min_inlier_ratio, 0.0);
  CHECK_OPTION_LE(abs_pose_min_inlier_ratio, 1.0);
  CHECK_OPTION_GE(local_ba_num_images, 2);
  CHECK_OPTION_GE(local_ba_min_tri_angle, 0.0);
  CHECK_OPTION_GE(min_focal_length_ratio, 0.0);
  CHECK_OPTION_GE(max_focal_length_ratio, min_focal_length_ratio);
  CHECK_OPTION_GE(max_extra_param, 0.0);
  CHECK_OPTION_GE(filter_max_reproj_error, 0.0);
  CHECK_OPTION_GE(filter_min_tri_angle, 0.0);
  CHECK_OPTION_GE(max_reg_trials, 1);
  CHECK_OPTION_GT(virtual_max_angular_error_deg, 0.0);
  CHECK_OPTION_GE(virtual_min_tri_angle_deg, 0.0);
  CHECK_OPTION_GE(virtual_max_num_per_image, 0);
  return true;
}

IncrementalMapper::IncrementalMapper(
    std::shared_ptr<const DatabaseCache> database_cache)
    : database_cache_(std::move(database_cache)),
      reconstruction_(nullptr),
      obs_manager_(nullptr),
      triangulator_(nullptr),
      num_total_reg_images_(0),
      num_shared_reg_images_(0) {}

void IncrementalMapper::BeginReconstruction(
    const std::shared_ptr<class Reconstruction>& reconstruction) {
  THROW_CHECK(reconstruction_ == nullptr);
  reconstruction_ = reconstruction;
  reconstruction_->Load(*database_cache_);
  obs_manager_ = std::make_shared<class ObservationManager>(
      *reconstruction_, database_cache_->CorrespondenceGraph());
  triangulator_ = std::make_shared<IncrementalTriangulator>(
      database_cache_->CorrespondenceGraph(), *reconstruction_, obs_manager_);

  num_shared_reg_images_ = 0;
  num_reg_images_per_camera_.clear();
  for (const image_t image_id : reconstruction_->RegImageIds()) {
    RegisterImageEvent(image_id);
  }

  existing_image_ids_ =
      std::unordered_set<image_t>(reconstruction->RegImageIds().begin(),
                                  reconstruction->RegImageIds().end());

  filtered_images_.clear();
  num_reg_trials_.clear();
}

void IncrementalMapper::EndReconstruction(const bool discard) {
  THROW_CHECK_NOTNULL(reconstruction_);

  if (!discard && !virtual_tracks_.empty()) {
    LogVirtualTrackSummary(
        StringPrintf("Virtual tracks (final, %d registered images)",
                     static_cast<int>(reconstruction_->NumRegImages())),
        virtual_loss_scale_px_);
  }

  if (discard) {
    for (const image_t image_id : reconstruction_->RegImageIds()) {
      DeRegisterImageEvent(image_id);
    }
  }

  reconstruction_->TearDown();
  reconstruction_ = nullptr;
  obs_manager_.reset();
  triangulator_.reset();
  virtual_tracks_.clear();
}

size_t IncrementalMapper::LoadVirtualTracks(const std::string& path) {
  THROW_CHECK_NOTNULL(reconstruction_);
  const std::vector<VirtualTrackFileObservation> observations =
      ReadVirtualTrackObservations(path);
  size_t num_unresolved = 0;
  std::set<std::string> unresolved_names;
  virtual_tracks_ = ResolveVirtualTracks(
      observations, *reconstruction_, &num_unresolved, &unresolved_names);

  size_t num_observations = 0;
  size_t num_registered_observations = 0;
  std::vector<double> weights;
  std::unordered_map<image_t, size_t> obs_per_image;
  std::vector<size_t> track_lengths;
  for (const VirtualTrack& track : virtual_tracks_) {
    num_observations += track.observations.size();
    track_lengths.push_back(track.observations.size());
    for (const VirtualObservation& obs : track.observations) {
      weights.push_back(obs.weight);
      ++obs_per_image[obs.image_id];
      if (reconstruction_->IsImageRegistered(obs.image_id)) {
        ++num_registered_observations;
      }
    }
  }
  LOG(INFO) << StringPrintf(
      "Loaded %d virtual tracks with %d observations from %s "
      "(%d observations of unknown images dropped; %d lines in file)",
      static_cast<int>(virtual_tracks_.size()),
      static_cast<int>(num_observations),
      path.c_str(),
      static_cast<int>(num_unresolved),
      static_cast<int>(observations.size()));
  if (!unresolved_names.empty()) {
    std::string names;
    size_t shown = 0;
    for (const std::string& name : unresolved_names) {
      if (shown++ == 5) {
        names += ", ...";
        break;
      }
      names += (shown == 1 ? "" : ", ") + name;
    }
    LOG(WARNING) << "Virtual track observations reference "
                 << unresolved_names.size()
                 << " image name(s) not in the database: " << names;
  }
  if (!virtual_tracks_.empty()) {
    std::vector<size_t> counts;
    counts.reserve(obs_per_image.size());
    for (const auto& [image_id, count] : obs_per_image) {
      counts.push_back(count);
    }
    LOG(INFO) << StringPrintf(
        "Virtual tracks (loaded): %d distinct images (%d observations on "
        "already registered images), observations per image p50 %d / max %d, "
        "track length p50 %d / max %d, weight p50 %.2f / min %.2f",
        static_cast<int>(obs_per_image.size()),
        static_cast<int>(num_registered_observations),
        static_cast<int>(Percentile(counts, 50)),
        static_cast<int>(*std::max_element(counts.begin(), counts.end())),
        static_cast<int>(Percentile(track_lengths, 50)),
        static_cast<int>(
            *std::max_element(track_lengths.begin(), track_lengths.end())),
        Percentile(weights, 50),
        *std::min_element(weights.begin(), weights.end()));
  }
  return virtual_tracks_.size();
}

const std::vector<VirtualTrack>& IncrementalMapper::VirtualTracks() const {
  return virtual_tracks_;
}

IncrementalMapper::VirtualTrackResidualStats
IncrementalMapper::ComputeVirtualTrackResiduals(
    const double loss_scale_px) const {
  VirtualTrackResidualStats stats;
  std::vector<double> errors;
  for (const VirtualTrack& track : virtual_tracks_) {
    if (!track.is_triangulated) {
      continue;
    }
    for (const VirtualObservation& obs : track.observations) {
      if (!obs.active) {
        continue;
      }
      const Image& image = reconstruction_->Image(obs.image_id);
      const double squared_error = CalculateSquaredReprojectionError(
          obs.xy, track.xyz, image.CamFromWorld(), *image.CameraPtr());
      ++stats.num_residuals;
      if (squared_error == std::numeric_limits<double>::max()) {
        ++stats.num_behind_camera;
        continue;
      }
      const double error = std::sqrt(squared_error);
      errors.push_back(error);
      if (error > loss_scale_px) {
        ++stats.num_beyond_loss_scale;
      }
      if (error > 10.0) {
        ++stats.num_beyond_10px;
      }
    }
  }
  if (!errors.empty()) {
    stats.p50_px = Percentile(errors, 50);
    stats.p90_px = Percentile(errors, 90);
    stats.max_px = *std::max_element(errors.begin(), errors.end());
  }
  return stats;
}

std::string IncrementalMapper::FormatVirtualTrackResiduals(
    const VirtualTrackResidualStats& stats, const double loss_scale_px) const {
  if (stats.num_residuals == 0) {
    return "no active virtual residuals";
  }
  return StringPrintf(
      "%d virtual residuals: reprojection error p50 %.2f px, p90 %.2f px, "
      "max %.1f px; %d beyond the loss scale (%.1f px), %d beyond 10 px, "
      "%d behind camera",
      static_cast<int>(stats.num_residuals),
      stats.p50_px,
      stats.p90_px,
      stats.max_px,
      static_cast<int>(stats.num_beyond_loss_scale),
      loss_scale_px,
      static_cast<int>(stats.num_beyond_10px),
      static_cast<int>(stats.num_behind_camera));
}

void IncrementalMapper::LogVirtualTrackSummary(
    const std::string& prefix, const double loss_scale_px) const {
  if (virtual_tracks_.empty() || reconstruction_ == nullptr) {
    return;
  }
  size_t num_triangulated = 0;
  size_t num_tracks_fully_active = 0;
  std::unordered_map<image_t, std::pair<size_t, size_t>> per_image;  // active, total
  for (const VirtualTrack& track : virtual_tracks_) {
    size_t num_active = 0;
    for (const VirtualObservation& obs : track.observations) {
      auto& entry = per_image[obs.image_id];
      ++entry.second;
      if (track.is_triangulated && obs.active) {
        ++entry.first;
        ++num_active;
      }
    }
    if (track.is_triangulated) {
      ++num_triangulated;
      if (num_active == track.observations.size()) {
        ++num_tracks_fully_active;
      }
    }
  }

  std::vector<std::pair<image_t, std::pair<size_t, size_t>>> ranked(
      per_image.begin(), per_image.end());
  std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
    if (a.second.first != b.second.first) {
      return a.second.first > b.second.first;
    }
    return a.first < b.first;
  });
  size_t num_images_with_residuals = 0;
  size_t num_images_unregistered = 0;
  std::string top;
  for (size_t i = 0; i < ranked.size(); ++i) {
    const auto& [image_id, counts] = ranked[i];
    if (counts.first > 0) {
      ++num_images_with_residuals;
    }
    if (!reconstruction_->IsImageRegistered(image_id)) {
      ++num_images_unregistered;
    }
    if (i < 12) {
      top += StringPrintf("%s%s:%d/%d",
                          i == 0 ? "" : " ",
                          reconstruction_->Image(image_id).Name().c_str(),
                          static_cast<int>(counts.first),
                          static_cast<int>(counts.second));
    }
  }
  const VirtualTrackResidualStats stats =
      ComputeVirtualTrackResiduals(loss_scale_px);
  LOG(INFO) << StringPrintf(
      "%s: %d tracks loaded, %d triangulated (%d with every observation "
      "active), %d images carry virtual observations, %d of them have active "
      "residuals, %d are unregistered; %s",
      prefix.c_str(),
      static_cast<int>(virtual_tracks_.size()),
      static_cast<int>(num_triangulated),
      static_cast<int>(num_tracks_fully_active),
      static_cast<int>(per_image.size()),
      static_cast<int>(num_images_with_residuals),
      static_cast<int>(num_images_unregistered),
      FormatVirtualTrackResiduals(stats, loss_scale_px).c_str());
  LOG(INFO) << prefix
            << ": active/total virtual observations per image (top): " << top;
}

IncrementalMapper::VirtualTrackReport IncrementalMapper::PrepareVirtualTracks(
    const Options& options) {
  THROW_CHECK_NOTNULL(reconstruction_);

  VirtualTrackReport report;
  report.num_tracks = virtual_tracks_.size();

  EstimateTriangulationOptions tri_options;
  tri_options.min_tri_angle = DegToRad(options.virtual_min_tri_angle_deg);
  tri_options.residual_type =
      TriangulationEstimator::ResidualType::ANGULAR_ERROR;
  tri_options.ransac_options.max_error =
      DegToRad(options.virtual_max_angular_error_deg);

  std::vector<Eigen::Vector2d> points;
  std::vector<Rigid3d const*> cams_from_world;
  std::vector<Camera const*> cameras;
  std::vector<size_t> obs_idxs;
  std::vector<char> inlier_mask;

  for (VirtualTrack& track : virtual_tracks_) {
    track.is_triangulated = false;
    points.clear();
    cams_from_world.clear();
    cameras.clear();
    obs_idxs.clear();
    for (size_t i = 0; i < track.observations.size(); ++i) {
      VirtualObservation& obs = track.observations[i];
      obs.active = false;
      if (!reconstruction_->IsImageRegistered(obs.image_id)) {
        continue;
      }
      const Image& image = reconstruction_->Image(obs.image_id);
      points.push_back(obs.xy);
      cams_from_world.push_back(&image.CamFromWorld());
      cameras.push_back(image.CameraPtr());
      obs_idxs.push_back(i);
    }
    report.num_observations_registered += points.size();
    if (points.size() < 2) {
      continue;
    }
    ++report.num_tracks_with_reg_observations;

    Eigen::Vector3d xyz;
    if (!EstimateTriangulation(tri_options,
                               points,
                               cams_from_world,
                               cameras,
                               &inlier_mask,
                               &xyz)) {
      report.num_observations_dropped_angular += points.size();
      continue;
    }

    size_t num_inliers = 0;
    for (size_t i = 0; i < obs_idxs.size(); ++i) {
      if (inlier_mask[i]) {
        track.observations[obs_idxs[i]].active = true;
        ++num_inliers;
      }
    }
    report.num_observations_dropped_angular += points.size() - num_inliers;
    if (num_inliers < 2) {
      for (const size_t obs_idx : obs_idxs) {
        track.observations[obs_idx].active = false;
      }
      continue;
    }
    track.xyz = xyz;
    track.is_triangulated = true;
    ++report.num_tracks_triangulated;
  }

  if (options.virtual_max_num_per_image > 0) {
    const size_t cap = static_cast<size_t>(options.virtual_max_num_per_image);
    // (weight, track index, observation index) per image.
    std::unordered_map<image_t, std::vector<std::tuple<double, size_t, size_t>>>
        obs_per_image;
    for (size_t t = 0; t < virtual_tracks_.size(); ++t) {
      const VirtualTrack& track = virtual_tracks_[t];
      if (!track.is_triangulated) {
        continue;
      }
      for (size_t o = 0; o < track.observations.size(); ++o) {
        const VirtualObservation& obs = track.observations[o];
        if (obs.active) {
          obs_per_image[obs.image_id].emplace_back(obs.weight, t, o);
        }
      }
    }
    for (auto& [image_id, obs_list] : obs_per_image) {
      if (obs_list.size() <= cap) {
        continue;
      }
      std::sort(
          obs_list.begin(), obs_list.end(), [](const auto& a, const auto& b) {
            if (std::get<0>(a) != std::get<0>(b)) {
              return std::get<0>(a) > std::get<0>(b);
            }
            return std::make_pair(std::get<1>(a), std::get<2>(a)) <
                   std::make_pair(std::get<1>(b), std::get<2>(b));
          });
      for (size_t i = cap; i < obs_list.size(); ++i) {
        virtual_tracks_[std::get<1>(obs_list[i])]
            .observations[std::get<2>(obs_list[i])]
            .active = false;
        ++report.num_observations_dropped_cap;
      }
    }
    for (VirtualTrack& track : virtual_tracks_) {
      if (track.is_triangulated && track.NumActiveObservations() < 2) {
        track.is_triangulated = false;
        --report.num_tracks_triangulated;
      }
    }
  }

  for (const VirtualTrack& track : virtual_tracks_) {
    if (track.is_triangulated) {
      report.num_residuals += track.NumActiveObservations();
    }
  }
  return report;
}

bool IncrementalMapper::FindInitialImagePair(const Options& options,
                                             TwoViewGeometry& two_view_geometry,
                                             image_t& image_id1,
                                             image_t& image_id2) {
  THROW_CHECK(options.Check());

  std::vector<image_t> image_ids1;
  if (image_id1 != kInvalidImageId && image_id2 == kInvalidImageId) {
    // Only image_id1 provided.
    if (!database_cache_->ExistsImage(image_id1)) {
      return false;
    }
    image_ids1.push_back(image_id1);
  } else if (image_id1 == kInvalidImageId && image_id2 != kInvalidImageId) {
    // Only image_id2 provided.
    if (!database_cache_->ExistsImage(image_id2)) {
      return false;
    }
    image_ids1.push_back(image_id2);
  } else {
    // No initial seed image provided.
    image_ids1 = FindFirstInitialImage(options);
  }

  // Try to find good initial pair.
  for (size_t i1 = 0; i1 < image_ids1.size(); ++i1) {
    image_id1 = image_ids1[i1];

    const std::vector<image_t> image_ids2 =
        FindSecondInitialImage(options, image_id1);

    for (size_t i2 = 0; i2 < image_ids2.size(); ++i2) {
      image_id2 = image_ids2[i2];

      const image_pair_t pair_id =
          Database::ImagePairToPairId(image_id1, image_id2);

      // Try every pair only once.
      if (init_image_pairs_.count(pair_id) > 0) {
        continue;
      }

      init_image_pairs_.insert(pair_id);

      if (EstimateInitialTwoViewGeometry(
              options, two_view_geometry, image_id1, image_id2)) {
        return true;
      }
    }
  }

  // No suitable pair found in entire dataset.
  image_id1 = kInvalidImageId;
  image_id2 = kInvalidImageId;

  return false;
}

std::vector<image_t> IncrementalMapper::FindNextImages(const Options& options) {
  THROW_CHECK_NOTNULL(reconstruction_);
  THROW_CHECK(options.Check());

  std::function<float(image_t, const class ObservationManager&)>
      rank_image_func;
  switch (options.image_selection_method) {
    case Options::ImageSelectionMethod::MAX_VISIBLE_POINTS_NUM:
      rank_image_func = RankNextImageMaxVisiblePointsNum;
      break;
    case Options::ImageSelectionMethod::MAX_VISIBLE_POINTS_RATIO:
      rank_image_func = RankNextImageMaxVisiblePointsRatio;
      break;
    case Options::ImageSelectionMethod::MIN_UNCERTAINTY:
      rank_image_func = RankNextImageMinUncertainty;
      break;
  }

  std::vector<std::pair<image_t, float>> image_ranks;
  std::vector<std::pair<image_t, float>> other_image_ranks;

  // Append images that have not failed to register before.
  for (const auto& image : reconstruction_->Images()) {
    // Skip images that are already registered.
    if (image.second.HasPose()) {
      continue;
    }

    // Only consider images with a sufficient number of visible points.
    if (obs_manager_->NumVisiblePoints3D(image.first) <
        static_cast<size_t>(options.abs_pose_min_num_inliers)) {
      continue;
    }

    // Only try registration for a certain maximum number of times.
    const size_t num_reg_trials = num_reg_trials_[image.first];
    if (num_reg_trials >= static_cast<size_t>(options.max_reg_trials)) {
      continue;
    }

    // If image has been filtered or failed to register, place it in the
    // second bucket and prefer images that have not been tried before.
    const float rank = rank_image_func(image.first, *obs_manager_);
    if (filtered_images_.count(image.first) == 0 && num_reg_trials == 0) {
      image_ranks.emplace_back(image.first, rank);
    } else {
      other_image_ranks.emplace_back(image.first, rank);
    }
  }

  std::vector<image_t> ranked_images_ids;
  SortAndAppendNextImages(image_ranks, &ranked_images_ids);
  SortAndAppendNextImages(other_image_ranks, &ranked_images_ids);

  return ranked_images_ids;
}

void IncrementalMapper::RegisterInitialImagePair(
    const Options& options,
    const TwoViewGeometry& two_view_geometry,
    const image_t image_id1,
    const image_t image_id2) {
  THROW_CHECK_NOTNULL(reconstruction_);
  THROW_CHECK_NOTNULL(obs_manager_);
  THROW_CHECK_EQ(reconstruction_->NumRegImages(), 0);

  THROW_CHECK(options.Check());

  init_num_reg_trials_[image_id1] += 1;
  init_num_reg_trials_[image_id2] += 1;
  num_reg_trials_[image_id1] += 1;
  num_reg_trials_[image_id2] += 1;

  const image_pair_t pair_id =
      Database::ImagePairToPairId(image_id1, image_id2);
  init_image_pairs_.insert(pair_id);

  Image& image1 = reconstruction_->Image(image_id1);
  const Camera& camera1 = *image1.CameraPtr();

  Image& image2 = reconstruction_->Image(image_id2);
  const Camera& camera2 = *image2.CameraPtr();

  //////////////////////////////////////////////////////////////////////////////
  // Estimate two-view geometry
  //////////////////////////////////////////////////////////////////////////////

  image1.SetCamFromWorld(Rigid3d());
  image2.SetCamFromWorld(two_view_geometry.cam2_from_cam1);

  const Eigen::Matrix3x4d cam_from_world1 = image1.CamFromWorld().ToMatrix();
  const Eigen::Matrix3x4d cam_from_world2 = image2.CamFromWorld().ToMatrix();
  const Eigen::Vector3d proj_center1 = image1.ProjectionCenter();
  const Eigen::Vector3d proj_center2 = image2.ProjectionCenter();

  //////////////////////////////////////////////////////////////////////////////
  // Update Reconstruction
  //////////////////////////////////////////////////////////////////////////////

  reconstruction_->RegisterImage(image_id1);
  reconstruction_->RegisterImage(image_id2);
  RegisterImageEvent(image_id1);
  RegisterImageEvent(image_id2);

  const FeatureMatches& corrs =
      database_cache_->CorrespondenceGraph()->FindCorrespondencesBetweenImages(
          image_id1, image_id2);

  const double min_tri_angle_rad = DegToRad(options.init_min_tri_angle);

  // Add 3D point tracks.
  Track track;
  track.Reserve(2);
  track.AddElement(TrackElement());
  track.AddElement(TrackElement());
  track.Element(0).image_id = image_id1;
  track.Element(1).image_id = image_id2;
  for (const auto& corr : corrs) {
    const Eigen::Vector2d point2D1 =
        camera1.CamFromImg(image1.Point2D(corr.point2D_idx1).xy);
    const Eigen::Vector2d point2D2 =
        camera2.CamFromImg(image2.Point2D(corr.point2D_idx2).xy);
    Eigen::Vector3d xyz;
    if (TriangulatePoint(
            cam_from_world1, cam_from_world2, point2D1, point2D2, &xyz) &&
        CalculateTriangulationAngle(proj_center1, proj_center2, xyz) >=
            min_tri_angle_rad &&
        HasPointPositiveDepth(cam_from_world1, xyz) &&
        HasPointPositiveDepth(cam_from_world2, xyz)) {
      track.Element(0).point2D_idx = corr.point2D_idx1;
      track.Element(1).point2D_idx = corr.point2D_idx2;
      obs_manager_->AddPoint3D(xyz, track);
    }
  }
}

bool IncrementalMapper::RegisterNextImage(const Options& options,
                                          const image_t image_id) {
  THROW_CHECK_NOTNULL(reconstruction_);
  THROW_CHECK_NOTNULL(obs_manager_);
  THROW_CHECK_GE(reconstruction_->NumRegImages(), 2);

  THROW_CHECK(options.Check());

  Image& image = reconstruction_->Image(image_id);
  Camera& camera = *image.CameraPtr();

  THROW_CHECK(!image.HasPose()) << "Image cannot be registered multiple times";

  num_reg_trials_[image_id] += 1;

  // Check if enough 2D-3D correspondences.
  if (obs_manager_->NumVisiblePoints3D(image_id) <
      static_cast<size_t>(options.abs_pose_min_num_inliers)) {
    return false;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Search for 2D-3D correspondences
  //////////////////////////////////////////////////////////////////////////////

  std::vector<std::pair<point2D_t, point3D_t>> tri_corrs;
  std::vector<Eigen::Vector2d> tri_points2D;
  std::vector<Eigen::Vector3d> tri_points3D;

  const std::shared_ptr<const CorrespondenceGraph> correspondence_graph =
      database_cache_->CorrespondenceGraph();

  std::unordered_set<point3D_t> corr_point3D_ids;
  for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D();
       ++point2D_idx) {
    const Point2D& point2D = image.Point2D(point2D_idx);

    corr_point3D_ids.clear();
    const auto corr_range =
        correspondence_graph->FindCorrespondences(image_id, point2D_idx);
    for (const auto* corr = corr_range.beg; corr < corr_range.end; ++corr) {
      const Image& corr_image = reconstruction_->Image(corr->image_id);
      if (!corr_image.HasPose()) {
        continue;
      }

      const Point2D& corr_point2D = corr_image.Point2D(corr->point2D_idx);
      if (!corr_point2D.HasPoint3D()) {
        continue;
      }

      // Avoid duplicate correspondences.
      if (corr_point3D_ids.count(corr_point2D.point3D_id) > 0) {
        continue;
      }

      const Camera& corr_camera = *corr_image.CameraPtr();

      // Avoid correspondences to images with bogus camera parameters.
      if (corr_camera.HasBogusParams(options.min_focal_length_ratio,
                                     options.max_focal_length_ratio,
                                     options.max_extra_param)) {
        continue;
      }

      const Point3D& point3D =
          reconstruction_->Point3D(corr_point2D.point3D_id);

      tri_corrs.emplace_back(point2D_idx, corr_point2D.point3D_id);
      corr_point3D_ids.insert(corr_point2D.point3D_id);
      tri_points2D.push_back(point2D.xy);
      tri_points3D.push_back(point3D.xyz);
    }
  }

  // The size of `next_image.num_tri_obs` and `tri_corrs_point2D_idxs.size()`
  // can only differ, when there are images with bogus camera parameters, and
  // hence we skip some of the 2D-3D correspondences.
  if (tri_points2D.size() <
      static_cast<size_t>(options.abs_pose_min_num_inliers)) {
    return false;
  }

  //////////////////////////////////////////////////////////////////////////////
  // 2D-3D estimation
  //////////////////////////////////////////////////////////////////////////////

  // Only refine / estimate focal length, if no focal length was specified
  // (manually or through EXIF) and if it was not already estimated previously
  // from another image (when multiple images share the same camera
  // parameters)

  AbsolutePoseEstimationOptions abs_pose_options;
  abs_pose_options.ransac_options.max_error = options.abs_pose_max_error;
  abs_pose_options.ransac_options.min_inlier_ratio =
      options.abs_pose_min_inlier_ratio;

  AbsolutePoseRefinementOptions abs_pose_refinement_options;
  if (num_reg_images_per_camera_[image.CameraId()] > 0) {
    // Camera already refined from another image with the same camera.
    if (camera.HasBogusParams(options.min_focal_length_ratio,
                              options.max_focal_length_ratio,
                              options.max_extra_param)) {
      // Previously refined camera has bogus parameters,
      // so reset parameters and try to re-estimage.
      camera.params = database_cache_->Camera(image.CameraId()).params;
      abs_pose_options.estimate_focal_length = !camera.has_prior_focal_length;
      abs_pose_refinement_options.refine_focal_length = true;
      abs_pose_refinement_options.refine_extra_params = true;
    } else {
      abs_pose_options.estimate_focal_length = false;
      abs_pose_refinement_options.refine_focal_length = false;
      abs_pose_refinement_options.refine_extra_params = false;
    }
  } else {
    // Camera not refined before. Note that the camera parameters might have
    // been changed before but the image was filtered, so we explicitly reset
    // the camera parameters and try to re-estimate them.
    camera.params = database_cache_->Camera(image.CameraId()).params;
    abs_pose_options.estimate_focal_length = !camera.has_prior_focal_length;
    abs_pose_refinement_options.refine_focal_length = true;
    abs_pose_refinement_options.refine_extra_params = true;
  }

  if (!options.abs_pose_refine_focal_length) {
    abs_pose_options.estimate_focal_length = false;
    abs_pose_refinement_options.refine_focal_length = false;
  }

  if (!options.abs_pose_refine_extra_params) {
    abs_pose_refinement_options.refine_extra_params = false;
  }

  size_t num_inliers;
  std::vector<char> inlier_mask;

  Rigid3d cam_from_world;
  if (!EstimateAbsolutePose(abs_pose_options,
                            tri_points2D,
                            tri_points3D,
                            &cam_from_world,
                            &camera,
                            &num_inliers,
                            &inlier_mask)) {
    return false;
  }

  if (num_inliers < static_cast<size_t>(options.abs_pose_min_num_inliers)) {
    return false;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Pose refinement
  //////////////////////////////////////////////////////////////////////////////

  if (!RefineAbsolutePose(abs_pose_refinement_options,
                          inlier_mask,
                          tri_points2D,
                          tri_points3D,
                          &cam_from_world,
                          &camera)) {
    return false;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Continue tracks
  //////////////////////////////////////////////////////////////////////////////

  image.SetCamFromWorld(cam_from_world);
  reconstruction_->RegisterImage(image_id);
  RegisterImageEvent(image_id);

  for (size_t i = 0; i < inlier_mask.size(); ++i) {
    if (inlier_mask[i]) {
      const point2D_t point2D_idx = tri_corrs[i].first;
      const Point2D& point2D = image.Point2D(point2D_idx);
      if (!point2D.HasPoint3D()) {
        const point3D_t point3D_id = tri_corrs[i].second;
        const TrackElement track_el(image_id, point2D_idx);
        obs_manager_->AddObservation(point3D_id, track_el);
        triangulator_->AddModifiedPoint3D(point3D_id);
      }
    }
  }

  return true;
}

size_t IncrementalMapper::TriangulateImage(
    const IncrementalTriangulator::Options& tri_options,
    const image_t image_id) {
  THROW_CHECK_NOTNULL(reconstruction_);
  VLOG(1) << "=> Continued observations: "
          << reconstruction_->Image(image_id).NumPoints3D();
  const size_t num_tris =
      triangulator_->TriangulateImage(tri_options, image_id);
  VLOG(1) << "=> Added observations: " << num_tris;
  return num_tris;
}

size_t IncrementalMapper::Retriangulate(
    const IncrementalTriangulator::Options& tri_options) {
  THROW_CHECK_NOTNULL(reconstruction_);
  return triangulator_->Retriangulate(tri_options);
}

size_t IncrementalMapper::CompleteTracks(
    const IncrementalTriangulator::Options& tri_options) {
  THROW_CHECK_NOTNULL(reconstruction_);
  return triangulator_->CompleteAllTracks(tri_options);
}

size_t IncrementalMapper::MergeTracks(
    const IncrementalTriangulator::Options& tri_options) {
  THROW_CHECK_NOTNULL(reconstruction_);
  return triangulator_->MergeAllTracks(tri_options);
}

size_t IncrementalMapper::CompleteAndMergeTracks(
    const IncrementalTriangulator::Options& tri_options) {
  const size_t num_completed_observations = CompleteTracks(tri_options);
  VLOG(1) << "=> Completed observations: " << num_completed_observations;
  const size_t num_merged_observations = MergeTracks(tri_options);
  VLOG(1) << "=> Merged observations: " << num_merged_observations;
  return num_completed_observations + num_merged_observations;
}

IncrementalMapper::LocalBundleAdjustmentReport
IncrementalMapper::AdjustLocalBundle(
    const Options& options,
    const BundleAdjustmentOptions& ba_options,
    const IncrementalTriangulator::Options& tri_options,
    const image_t image_id,
    const std::unordered_set<point3D_t>& point3D_ids) {
  THROW_CHECK_NOTNULL(reconstruction_);
  THROW_CHECK_NOTNULL(obs_manager_);
  THROW_CHECK(options.Check());

  LocalBundleAdjustmentReport report;

  // Find images that have most 3D points with given image in common.
  const std::vector<image_t> local_bundle = FindLocalBundle(options, image_id);

  // Do the bundle adjustment only if there is any connected images.
  if (local_bundle.size() > 0) {
    BundleAdjustmentConfig ba_config;
    ba_config.AddImage(image_id);
    for (const image_t local_image_id : local_bundle) {
      ba_config.AddImage(local_image_id);
    }

    // Fix the existing images, if option specified.
    if (options.fix_existing_images) {
      for (const image_t local_image_id : local_bundle) {
        if (existing_image_ids_.count(local_image_id)) {
          ba_config.SetConstantCamPose(local_image_id);
        }
      }
    }

    // Determine which cameras to fix, when not all the registered images
    // are within the current local bundle.
    std::unordered_map<camera_t, size_t> num_images_per_camera;
    for (const image_t image_id : ba_config.Images()) {
      const Image& image = reconstruction_->Image(image_id);
      num_images_per_camera[image.CameraId()] += 1;
    }

    for (const auto& [camera_id, num_images] : num_images_per_camera) {
      const size_t num_reg_images_for_camera =
          num_reg_images_per_camera_.at(camera_id);
      if (num_images < num_reg_images_for_camera) {
        ba_config.SetConstantCamIntrinsics(camera_id);
      }
    }

    // Fix 7 DOF to avoid scale/rotation/translation drift in bundle adjustment.
    if (local_bundle.size() == 1) {
      ba_config.SetConstantCamPose(local_bundle[0]);
      ba_config.SetConstantCamPositions(image_id, {0});
    } else if (local_bundle.size() > 1) {
      const image_t image_id1 = local_bundle[local_bundle.size() - 1];
      const image_t image_id2 = local_bundle[local_bundle.size() - 2];
      ba_config.SetConstantCamPose(image_id1);
      if (!options.fix_existing_images ||
          !existing_image_ids_.count(image_id2)) {
        ba_config.SetConstantCamPositions(image_id2, {0});
      }
    }

    // Make sure, we refine all new and short-track 3D points, no matter if
    // they are fully contained in the local image set or not. Do not include
    // long track 3D points as they are usually already very stable and adding
    // to them to bundle adjustment and track merging/completion would slow
    // down the local bundle adjustment significantly.
    std::unordered_set<point3D_t> variable_point3D_ids;
    for (const point3D_t point3D_id : point3D_ids) {
      const Point3D& point3D = reconstruction_->Point3D(point3D_id);
      const size_t kMaxTrackLength = 15;
      if (!point3D.HasError() || point3D.track.Length() <= kMaxTrackLength) {
        ba_config.AddVariablePoint(point3D_id);
        variable_point3D_ids.insert(point3D_id);
      }
    }

    // Adjust the local bundle.
    std::vector<VirtualTrack>* virtual_tracks = nullptr;
    if (options.apply_virtual_tracks && ba_options.apply_virtual_tracks &&
        !virtual_tracks_.empty()) {
      const VirtualTrackReport vt_report = PrepareVirtualTracks(options);
      virtual_loss_scale_px_ = ba_options.virtual_loss_scale;
      VLOG(1) << StringPrintf(
          "=> Virtual tracks (local BA): %d/%d triangulated, %d residuals, "
          "%d observations dropped by angular filter, %d by per-image cap",
          static_cast<int>(vt_report.num_tracks_triangulated),
          static_cast<int>(vt_report.num_tracks),
          static_cast<int>(vt_report.num_residuals),
          static_cast<int>(vt_report.num_observations_dropped_angular),
          static_cast<int>(vt_report.num_observations_dropped_cap));
      virtual_tracks = &virtual_tracks_;
    }
    std::unique_ptr<BundleAdjuster> bundle_adjuster =
        CreateDefaultBundleAdjuster(
            ba_options, std::move(ba_config), *reconstruction_, virtual_tracks);
    const ceres::Solver::Summary summary = bundle_adjuster->Solve();

    report.num_adjusted_observations = summary.num_residuals / 2;

    // Merge refined tracks with other existing points.
    report.num_merged_observations =
        triangulator_->MergeTracks(tri_options, variable_point3D_ids);
    // Complete tracks that may have failed to triangulate before refinement
    // of camera pose and calibration in bundle-adjustment. This may avoid
    // that some points are filtered and it helps for subsequent image
    // registrations.
    report.num_completed_observations =
        triangulator_->CompleteTracks(tri_options, variable_point3D_ids);
    report.num_completed_observations +=
        triangulator_->CompleteImage(tri_options, image_id);
  }

  // Filter both the modified images and all changed 3D points to make sure
  // there are no outlier points in the model. This results in duplicate work as
  // many of the provided 3D points may also be contained in the adjusted
  // images, but the filtering is not a bottleneck at this point.
  std::unordered_set<image_t> filter_image_ids;
  filter_image_ids.insert(image_id);
  filter_image_ids.insert(local_bundle.begin(), local_bundle.end());
  report.num_filtered_observations =
      obs_manager_->FilterPoints3DInImages(options.filter_max_reproj_error,
                                           options.filter_min_tri_angle,
                                           filter_image_ids);
  report.num_filtered_observations +=
      obs_manager_->FilterPoints3D(options.filter_max_reproj_error,
                                   options.filter_min_tri_angle,
                                   point3D_ids);

  return report;
}

bool IncrementalMapper::AdjustGlobalBundle(
    const Options& options, const BundleAdjustmentOptions& ba_options) {
  THROW_CHECK_NOTNULL(reconstruction_);
  THROW_CHECK_NOTNULL(obs_manager_);

  const std::set<image_t>& reg_image_ids = reconstruction_->RegImageIds();

  THROW_CHECK_GE(reg_image_ids.size(), 2) << "At least two images must be "
                                             "registered for global "
                                             "bundle-adjustment";

  BundleAdjustmentOptions custom_ba_options = ba_options;
  // Use stricter convergence criteria for first registered images.
  const size_t kMinNumRegImagesForFastBA = 10;
  if (reg_image_ids.size() < kMinNumRegImagesForFastBA) {
    custom_ba_options.solver_options.function_tolerance /= 10;
    custom_ba_options.solver_options.gradient_tolerance /= 10;
    custom_ba_options.solver_options.parameter_tolerance /= 10;
    custom_ba_options.solver_options.max_num_iterations *= 2;
    custom_ba_options.solver_options.max_linear_solver_iterations = 200;
  }

  // Avoid degeneracies in bundle adjustment.
  obs_manager_->FilterObservationsWithNegativeDepth();

  // Configure bundle adjustment.
  BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reg_image_ids) {
    ba_config.AddImage(image_id);
  }

  // Fix the existing images, if option specified.
  if (options.fix_existing_images) {
    for (const image_t image_id : reg_image_ids) {
      if (existing_image_ids_.count(image_id)) {
        ba_config.SetConstantCamPose(image_id);
      }
    }
  }

  // Only use prior pose if at least 3 images have been registered.
  const bool use_prior_position =
      options.use_prior_position && reg_image_ids.size() > 2;

  std::unique_ptr<BundleAdjuster> bundle_adjuster;
  if (!use_prior_position) {
    // Fix 7-DOFs of the bundle adjustment problem.
    auto reg_image_ids_it = reg_image_ids.begin();
    ba_config.SetConstantCamPose(*(reg_image_ids_it++));  // 1st image
    if (!options.fix_existing_images ||
        !existing_image_ids_.count(*reg_image_ids_it)) {
      ba_config.SetConstantCamPositions(*reg_image_ids_it, {0});  // 2nd image
    }

    std::vector<VirtualTrack>* virtual_tracks = nullptr;
    if (options.apply_virtual_tracks &&
        custom_ba_options.apply_virtual_tracks && !virtual_tracks_.empty()) {
      const VirtualTrackReport vt_report = PrepareVirtualTracks(options);
      virtual_loss_scale_px_ = custom_ba_options.virtual_loss_scale;
      LOG(INFO) << StringPrintf(
          "Virtual tracks (global BA, %d registered images): %d loaded, %d "
          "with >= 2 registered observations (%d registered observations in "
          "total), %d triangulated, %d residuals added, %d observations "
          "dropped by angular filter (> %.2f deg or < %.2f deg parallax), %d "
          "by per-image cap (%d)",
          static_cast<int>(reconstruction_->NumRegImages()),
          static_cast<int>(vt_report.num_tracks),
          static_cast<int>(vt_report.num_tracks_with_reg_observations),
          static_cast<int>(vt_report.num_observations_registered),
          static_cast<int>(vt_report.num_tracks_triangulated),
          static_cast<int>(vt_report.num_residuals),
          static_cast<int>(vt_report.num_observations_dropped_angular),
          options.virtual_max_angular_error_deg,
          options.virtual_min_tri_angle_deg,
          static_cast<int>(vt_report.num_observations_dropped_cap),
          options.virtual_max_num_per_image);
      if (vt_report.num_residuals > 0) {
        LOG(INFO) << "Virtual tracks (global BA) before solve: "
                  << FormatVirtualTrackResiduals(
                         ComputeVirtualTrackResiduals(virtual_loss_scale_px_),
                         virtual_loss_scale_px_);
      }
      virtual_tracks = &virtual_tracks_;
    }
    bundle_adjuster = CreateDefaultBundleAdjuster(std::move(custom_ba_options),
                                                  std::move(ba_config),
                                                  *reconstruction_,
                                                  virtual_tracks);
    const ceres::Solver::Summary summary = bundle_adjuster->Solve();
    if (virtual_tracks != nullptr) {
      LOG(INFO) << "Virtual tracks (global BA) after solve: "
                << FormatVirtualTrackResiduals(
                       ComputeVirtualTrackResiduals(virtual_loss_scale_px_),
                       virtual_loss_scale_px_);
    }
    return summary.termination_type != ceres::FAILURE;
  } else {
    if (options.apply_virtual_tracks && !virtual_tracks_.empty()) {
      LOG_FIRST_N(WARNING, 1)
          << "Virtual tracks are not supported together with pose priors; "
             "ignoring them in global bundle adjustment.";
    }
    PosePriorBundleAdjustmentOptions prior_options;
    prior_options.use_robust_loss_on_prior_position =
        options.use_robust_loss_on_prior_position;
    prior_options.prior_position_loss_scale = options.prior_position_loss_scale;
    bundle_adjuster =
        CreatePosePriorBundleAdjuster(std::move(custom_ba_options),
                                      prior_options,
                                      std::move(ba_config),
                                      database_cache_->PosePriors(),
                                      *reconstruction_);
  }

  return bundle_adjuster->Solve().termination_type != ceres::FAILURE;
}

void IncrementalMapper::IterativeLocalRefinement(
    const int max_num_refinements,
    const double max_refinement_change,
    const Options& options,
    const BundleAdjustmentOptions& ba_options,
    const IncrementalTriangulator::Options& tri_options,
    const image_t image_id) {
  BundleAdjustmentOptions custom_ba_options = ba_options;
  for (int i = 0; i < max_num_refinements; ++i) {
    const auto report = AdjustLocalBundle(options,
                                          custom_ba_options,
                                          tri_options,
                                          image_id,
                                          GetModifiedPoints3D());
    VLOG(1) << "=> Merged observations: " << report.num_merged_observations;
    VLOG(1) << "=> Completed observations: "
            << report.num_completed_observations;
    VLOG(1) << "=> Filtered observations: " << report.num_filtered_observations;
    const double changed =
        report.num_adjusted_observations == 0
            ? 0
            : (report.num_merged_observations +
               report.num_completed_observations +
               report.num_filtered_observations) /
                  static_cast<double>(report.num_adjusted_observations);
    VLOG(1) << StringPrintf("=> Changed observations: %.6f", changed);
    if (changed < max_refinement_change) {
      break;
    }
    // Only use robust cost function for first iteration.
    custom_ba_options.loss_function_type =
        BundleAdjustmentOptions::LossFunctionType::TRIVIAL;
  }
  ClearModifiedPoints3D();
}

void IncrementalMapper::IterativeGlobalRefinement(
    const int max_num_refinements,
    const double max_refinement_change,
    const Options& options,
    const BundleAdjustmentOptions& ba_options,
    const IncrementalTriangulator::Options& tri_options,
    const bool normalize_reconstruction) {
  CompleteAndMergeTracks(tri_options);
  VLOG(1) << "=> Retriangulated observations: " << Retriangulate(tri_options);
  for (int i = 0; i < max_num_refinements; ++i) {
    const size_t num_observations = reconstruction_->ComputeNumObservations();
    AdjustGlobalBundle(options, ba_options);
    if (normalize_reconstruction && !options.use_prior_position) {
      // Normalize scene for numerical stability and
      // to avoid large scale changes in the viewer.
      reconstruction_->Normalize();
    }
    size_t num_changed_observations = CompleteAndMergeTracks(tri_options);
    num_changed_observations += FilterPoints(options);
    const double changed =
        num_observations == 0
            ? 0
            : static_cast<double>(num_changed_observations) / num_observations;
    VLOG(1) << StringPrintf("=> Changed observations: %.6f", changed);
    if (changed < max_refinement_change) {
      break;
    }
  }
  ClearModifiedPoints3D();
}

size_t IncrementalMapper::FilterImages(const Options& options) {
  THROW_CHECK_NOTNULL(reconstruction_);
  THROW_CHECK_NOTNULL(obs_manager_);
  THROW_CHECK(options.Check());

  // Do not filter images in the early stage of the reconstruction, since the
  // calibration is often still refining a lot. Hence, the camera parameters
  // are not stable in the beginning.
  const size_t kMinNumImages = 20;
  if (reconstruction_->NumRegImages() < kMinNumImages) {
    return {};
  }

  const std::vector<image_t> image_ids =
      obs_manager_->FilterImages(options.min_focal_length_ratio,
                                 options.max_focal_length_ratio,
                                 options.max_extra_param);

  for (const image_t image_id : image_ids) {
    DeRegisterImageEvent(image_id);
    filtered_images_.insert(image_id);
  }

  const size_t num_filtered_images = image_ids.size();
  VLOG(1) << "=> Filtered images: " << num_filtered_images;
  return num_filtered_images;
}

size_t IncrementalMapper::FilterPoints(const Options& options) {
  THROW_CHECK_NOTNULL(obs_manager_);
  THROW_CHECK(options.Check());
  const size_t num_filtered_observations = obs_manager_->FilterAllPoints3D(
      options.filter_max_reproj_error, options.filter_min_tri_angle);
  VLOG(1) << "=> Filtered observations: " << num_filtered_observations;
  return num_filtered_observations;
}

std::shared_ptr<class Reconstruction> IncrementalMapper::Reconstruction()
    const {
  return reconstruction_;
}

class ObservationManager& IncrementalMapper::ObservationManager() const {
  THROW_CHECK_NOTNULL(obs_manager_);
  return *obs_manager_;
}

IncrementalTriangulator& IncrementalMapper::Triangulator() const {
  THROW_CHECK_NOTNULL(triangulator_);
  return *triangulator_;
}

const std::unordered_set<image_t>& IncrementalMapper::FilteredImages() const {
  return filtered_images_;
}

const std::unordered_set<image_t>& IncrementalMapper::ExistingImageIds() const {
  return existing_image_ids_;
}

const std::unordered_map<camera_t, size_t>&
IncrementalMapper::NumRegImagesPerCamera() const {
  return num_reg_images_per_camera_;
}

size_t IncrementalMapper::NumTotalRegImages() const {
  return num_total_reg_images_;
}

size_t IncrementalMapper::NumSharedRegImages() const {
  return num_shared_reg_images_;
}

const std::unordered_set<point3D_t>& IncrementalMapper::GetModifiedPoints3D() {
  return triangulator_->GetModifiedPoints3D();
}

void IncrementalMapper::ClearModifiedPoints3D() {
  triangulator_->ClearModifiedPoints3D();
}

std::vector<image_t> IncrementalMapper::FindFirstInitialImage(
    const Options& options) const {
  // Struct to hold meta-data for ranking images.
  struct ImageInfo {
    image_t image_id;
    bool prior_focal_length;
    image_t num_correspondences;
  };

  const size_t init_max_reg_trials =
      static_cast<size_t>(options.init_max_reg_trials);

  // Collect information of all not yet registered images with
  // correspondences.
  std::vector<ImageInfo> image_infos;
  image_infos.reserve(reconstruction_->NumImages());
  for (const auto& image : reconstruction_->Images()) {
    // Only images with correspondences can be registered.
    if (obs_manager_->NumCorrespondences(image.first) == 0) {
      continue;
    }

    // Only use images for initialization a maximum number of times.
    if (init_num_reg_trials_.count(image.first) &&
        init_num_reg_trials_.at(image.first) >= init_max_reg_trials) {
      continue;
    }

    // Only use images for initialization that are not registered in any
    // of the other reconstructions.
    if (num_registrations_.count(image.first) > 0 &&
        num_registrations_.at(image.first) > 0) {
      continue;
    }

    const Camera& camera = *image.second.CameraPtr();
    ImageInfo image_info;
    image_info.image_id = image.first;
    image_info.prior_focal_length = camera.has_prior_focal_length;
    image_info.num_correspondences =
        obs_manager_->NumCorrespondences(image.first);
    image_infos.push_back(image_info);
  }

  // Sort images such that images with a prior focal length and more
  // correspondences are preferred, i.e. they appear in the front of the list.
  std::sort(
      image_infos.begin(),
      image_infos.end(),
      [](const ImageInfo& image_info1, const ImageInfo& image_info2) {
        if (image_info1.prior_focal_length && !image_info2.prior_focal_length) {
          return true;
        } else if (!image_info1.prior_focal_length &&
                   image_info2.prior_focal_length) {
          return false;
        } else {
          return image_info1.num_correspondences >
                 image_info2.num_correspondences;
        }
      });

  // Extract image identifiers in sorted order.
  std::vector<image_t> image_ids;
  image_ids.reserve(image_infos.size());
  for (const ImageInfo& image_info : image_infos) {
    image_ids.push_back(image_info.image_id);
  }

  return image_ids;
}

std::vector<image_t> IncrementalMapper::FindSecondInitialImage(
    const Options& options, const image_t image_id1) const {
  const std::shared_ptr<const CorrespondenceGraph> correspondence_graph =
      database_cache_->CorrespondenceGraph();
  // Collect images that are connected to the first seed image and have
  // not been registered before in other reconstructions.
  const class Image& image1 = reconstruction_->Image(image_id1);
  std::unordered_map<image_t, point2D_t> num_correspondences;
  for (point2D_t point2D_idx = 0; point2D_idx < image1.NumPoints2D();
       ++point2D_idx) {
    const auto corr_range =
        correspondence_graph->FindCorrespondences(image_id1, point2D_idx);
    for (const auto* corr = corr_range.beg; corr < corr_range.end; ++corr) {
      if (num_registrations_.count(corr->image_id) == 0 ||
          num_registrations_.at(corr->image_id) == 0) {
        num_correspondences[corr->image_id] += 1;
      }
    }
  }

  // Struct to hold meta-data for ranking images.
  struct ImageInfo {
    image_t image_id;
    bool prior_focal_length;
    point2D_t num_correspondences;
  };

  const size_t init_min_num_inliers =
      static_cast<size_t>(options.init_min_num_inliers);

  // Compose image information in a compact form for sorting.
  std::vector<ImageInfo> image_infos;
  image_infos.reserve(reconstruction_->NumImages());
  for (const auto elem : num_correspondences) {
    if (elem.second >= init_min_num_inliers) {
      const Image& image = reconstruction_->Image(elem.first);
      const Camera& camera = *image.CameraPtr();
      ImageInfo image_info;
      image_info.image_id = elem.first;
      image_info.prior_focal_length = camera.has_prior_focal_length;
      image_info.num_correspondences = elem.second;
      image_infos.push_back(image_info);
    }
  }

  // Sort images such that images with a prior focal length and more
  // correspondences are preferred, i.e. they appear in the front of the list.
  std::sort(
      image_infos.begin(),
      image_infos.end(),
      [](const ImageInfo& image_info1, const ImageInfo& image_info2) {
        if (image_info1.prior_focal_length && !image_info2.prior_focal_length) {
          return true;
        } else if (!image_info1.prior_focal_length &&
                   image_info2.prior_focal_length) {
          return false;
        } else {
          return image_info1.num_correspondences >
                 image_info2.num_correspondences;
        }
      });

  // Extract image identifiers in sorted order.
  std::vector<image_t> image_ids;
  image_ids.reserve(image_infos.size());
  for (const ImageInfo& image_info : image_infos) {
    image_ids.push_back(image_info.image_id);
  }

  return image_ids;
}

std::vector<image_t> IncrementalMapper::FindLocalBundle(
    const Options& options, const image_t image_id) const {
  THROW_CHECK(options.Check());

  const Image& image = reconstruction_->Image(image_id);
  THROW_CHECK(image.HasPose());

  // Extract all images that have at least one 3D point with the query image
  // in common, and simultaneously count the number of common 3D points.

  std::unordered_map<image_t, size_t> shared_observations;

  std::unordered_set<point3D_t> point3D_ids;
  point3D_ids.reserve(image.NumPoints3D());

  for (const Point2D& point2D : image.Points2D()) {
    if (point2D.HasPoint3D()) {
      point3D_ids.insert(point2D.point3D_id);
      const Point3D& point3D = reconstruction_->Point3D(point2D.point3D_id);
      for (const TrackElement& track_el : point3D.track.Elements()) {
        if (track_el.image_id != image_id) {
          shared_observations[track_el.image_id] += 1;
        }
      }
    }
  }

  // Sort overlapping images according to number of shared observations.

  std::vector<std::pair<image_t, size_t>> overlapping_images(
      shared_observations.begin(), shared_observations.end());
  std::sort(overlapping_images.begin(),
            overlapping_images.end(),
            [](const std::pair<image_t, size_t>& image1,
               const std::pair<image_t, size_t>& image2) {
              return image1.second > image2.second;
            });

  // The local bundle is composed of the given image and its most connected
  // neighbor images, hence the subtraction of 1.

  const size_t num_images =
      static_cast<size_t>(options.local_ba_num_images - 1);
  const size_t num_eff_images = std::min(num_images, overlapping_images.size());

  // Extract most connected images and ensure sufficient triangulation angle.

  std::vector<image_t> local_bundle_image_ids;
  local_bundle_image_ids.reserve(num_eff_images);

  // If the number of overlapping images equals the number of desired images in
  // the local bundle, then simply copy over the image identifiers.
  if (overlapping_images.size() == num_eff_images) {
    for (const auto& overlapping_image : overlapping_images) {
      local_bundle_image_ids.push_back(overlapping_image.first);
    }
    return local_bundle_image_ids;
  }

  // In the following iteration, we start with the most overlapping images and
  // check whether it has sufficient triangulation angle. If none of the
  // overlapping images has sufficient triangulation angle, we relax the
  // triangulation angle threshold and start from the most overlapping image
  // again. In the end, if we still haven't found enough images, we simply use
  // the most overlapping images.

  const double min_tri_angle_rad = DegToRad(options.local_ba_min_tri_angle);

  // The selection thresholds (minimum triangulation angle, minimum number of
  // shared observations), which are successively relaxed.
  const std::array<std::pair<double, double>, 8> selection_thresholds = {{
      std::make_pair(min_tri_angle_rad / 1.0, 0.6 * image.NumPoints3D()),
      std::make_pair(min_tri_angle_rad / 1.5, 0.6 * image.NumPoints3D()),
      std::make_pair(min_tri_angle_rad / 2.0, 0.5 * image.NumPoints3D()),
      std::make_pair(min_tri_angle_rad / 2.5, 0.4 * image.NumPoints3D()),
      std::make_pair(min_tri_angle_rad / 3.0, 0.3 * image.NumPoints3D()),
      std::make_pair(min_tri_angle_rad / 4.0, 0.2 * image.NumPoints3D()),
      std::make_pair(min_tri_angle_rad / 5.0, 0.1 * image.NumPoints3D()),
      std::make_pair(min_tri_angle_rad / 6.0, 0.1 * image.NumPoints3D()),
  }};

  const Eigen::Vector3d proj_center = image.ProjectionCenter();
  std::vector<Eigen::Vector3d> shared_points3D;
  shared_points3D.reserve(image.NumPoints3D());
  std::vector<double> tri_angles(overlapping_images.size(), -1.0);
  std::vector<char> used_overlapping_images(overlapping_images.size(), false);

  for (const auto& selection_threshold : selection_thresholds) {
    for (size_t overlapping_image_idx = 0;
         overlapping_image_idx < overlapping_images.size();
         ++overlapping_image_idx) {
      // Check if the image has sufficient overlap. Since the images are ordered
      // based on the overlap, we can just skip the remaining ones.
      if (overlapping_images[overlapping_image_idx].second <
          selection_threshold.second) {
        break;
      }

      // Check if the image is already in the local bundle.
      if (used_overlapping_images[overlapping_image_idx]) {
        continue;
      }

      const auto& overlapping_image = reconstruction_->Image(
          overlapping_images[overlapping_image_idx].first);
      const Eigen::Vector3d overlapping_proj_center =
          overlapping_image.ProjectionCenter();

      // In the first iteration, compute the triangulation angle. In later
      // iterations, reuse the previously computed value.
      double& tri_angle = tri_angles[overlapping_image_idx];
      if (tri_angle < 0.0) {
        // Collect the commonly observed 3D points.
        shared_points3D.clear();
        for (const Point2D& point2D : overlapping_image.Points2D()) {
          if (point2D.HasPoint3D() && point3D_ids.count(point2D.point3D_id)) {
            shared_points3D.push_back(
                reconstruction_->Point3D(point2D.point3D_id).xyz);
          }
        }

        // Calculate the triangulation angle at a certain percentile.
        const double kTriangulationAnglePercentile = 75;
        tri_angle = Percentile(
            CalculateTriangulationAngles(
                proj_center, overlapping_proj_center, shared_points3D),
            kTriangulationAnglePercentile);
      }

      // Check that the image has sufficient triangulation angle.
      if (tri_angle >= selection_threshold.first) {
        local_bundle_image_ids.push_back(overlapping_image.ImageId());
        used_overlapping_images[overlapping_image_idx] = true;
        // Check if we already collected enough images.
        if (local_bundle_image_ids.size() >= num_eff_images) {
          break;
        }
      }
    }

    // Check if we already collected enough images.
    if (local_bundle_image_ids.size() >= num_eff_images) {
      break;
    }
  }

  // In case there are not enough images with sufficient triangulation angle,
  // simply fill up the rest with the most overlapping images.

  if (local_bundle_image_ids.size() < num_eff_images) {
    for (size_t overlapping_image_idx = 0;
         overlapping_image_idx < overlapping_images.size();
         ++overlapping_image_idx) {
      // Collect image if it is not yet in the local bundle.
      if (!used_overlapping_images[overlapping_image_idx]) {
        local_bundle_image_ids.push_back(
            overlapping_images[overlapping_image_idx].first);
        used_overlapping_images[overlapping_image_idx] = true;

        // Check if we already collected enough images.
        if (local_bundle_image_ids.size() >= num_eff_images) {
          break;
        }
      }
    }
  }

  return local_bundle_image_ids;
}

void IncrementalMapper::RegisterImageEvent(const image_t image_id) {
  const Image& image = reconstruction_->Image(image_id);
  size_t& num_reg_images_for_camera =
      num_reg_images_per_camera_[image.CameraId()];
  num_reg_images_for_camera += 1;

  size_t& num_regs_for_image = num_registrations_[image_id];
  num_regs_for_image += 1;
  if (num_regs_for_image == 1) {
    num_total_reg_images_ += 1;
  } else if (num_regs_for_image > 1) {
    num_shared_reg_images_ += 1;
  }
}

void IncrementalMapper::DeRegisterImageEvent(const image_t image_id) {
  const Image& image = reconstruction_->Image(image_id);
  size_t& num_reg_images_for_camera =
      num_reg_images_per_camera_.at(image.CameraId());
  THROW_CHECK_GT(num_reg_images_for_camera, 0);
  num_reg_images_for_camera -= 1;

  size_t& num_regs_for_image = num_registrations_[image_id];
  num_regs_for_image -= 1;
  if (num_regs_for_image == 0) {
    num_total_reg_images_ -= 1;
  } else if (num_regs_for_image > 0) {
    num_shared_reg_images_ -= 1;
  }
}

bool IncrementalMapper::EstimateInitialTwoViewGeometry(
    const Options& options,
    TwoViewGeometry& two_view_geometry,
    const image_t image_id1,
    const image_t image_id2) {
  const Image& image1 = database_cache_->Image(image_id1);
  const Camera& camera1 = database_cache_->Camera(image1.CameraId());

  const Image& image2 = database_cache_->Image(image_id2);
  const Camera& camera2 = database_cache_->Camera(image2.CameraId());

  const FeatureMatches matches =
      database_cache_->CorrespondenceGraph()->FindCorrespondencesBetweenImages(
          image_id1, image_id2);

  std::vector<Eigen::Vector2d> points1;
  points1.reserve(image1.NumPoints2D());
  for (const auto& point : image1.Points2D()) {
    points1.push_back(point.xy);
  }

  std::vector<Eigen::Vector2d> points2;
  points2.reserve(image2.NumPoints2D());
  for (const auto& point : image2.Points2D()) {
    points2.push_back(point.xy);
  }

  TwoViewGeometryOptions two_view_geometry_options;
  two_view_geometry_options.ransac_options.min_num_trials = 30;
  two_view_geometry_options.ransac_options.max_error = options.init_max_error;
  two_view_geometry = EstimateCalibratedTwoViewGeometry(
      camera1, points1, camera2, points2, matches, two_view_geometry_options);

  if (!EstimateTwoViewGeometryPose(
          camera1, points1, camera2, points2, &two_view_geometry)) {
    return false;
  }

  if (static_cast<int>(two_view_geometry.inlier_matches.size()) >=
          options.init_min_num_inliers &&
      std::abs(two_view_geometry.cam2_from_cam1.translation.z()) <
          options.init_max_forward_motion &&
      two_view_geometry.tri_angle > DegToRad(options.init_min_tri_angle)) {
    return true;
  }

  return false;
}

}  // namespace colmap
