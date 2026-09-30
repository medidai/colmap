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

#include "colmap/scene/virtual_tracks.h"

#include "colmap/util/file.h"
#include "colmap/util/logging.h"
#include "colmap/util/string.h"

#include <fstream>
#include <map>
#include <sstream>
#include <unordered_map>

namespace colmap {

size_t VirtualTrack::NumActiveObservations() const {
  size_t num_active = 0;
  for (const VirtualObservation& obs : observations) {
    if (obs.active) {
      ++num_active;
    }
  }
  return num_active;
}

std::vector<VirtualTrackFileObservation> ReadVirtualTrackObservations(
    const std::string& path) {
  std::ifstream file(path);
  THROW_CHECK_FILE_OPEN(file, path);

  std::vector<VirtualTrackFileObservation> observations;
  std::string line;
  size_t line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    StringTrim(&line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream line_stream(line);
    VirtualTrackFileObservation obs;
    if (!(line_stream >> obs.track_id >> obs.xy(0) >> obs.xy(1) >> obs.weight >>
          obs.image_name)) {
      LOG(FATAL_THROW) << "Malformed virtual track line " << line_number
                       << " in " << path << ": " << line;
    }
    observations.push_back(std::move(obs));
  }
  return observations;
}

std::vector<VirtualTrack> ResolveVirtualTracks(
    const std::vector<VirtualTrackFileObservation>& observations,
    const Reconstruction& reconstruction,
    size_t* num_unresolved) {
  std::unordered_map<std::string, image_t> image_ids;
  for (const auto& [image_id, image] : reconstruction.Images()) {
    image_ids.emplace(image.Name(), image_id);
  }

  // std::map keeps the output ordered by track id for deterministic BA setup.
  std::map<int64_t, VirtualTrack> tracks;
  size_t unresolved = 0;
  for (const VirtualTrackFileObservation& file_obs : observations) {
    const auto it = image_ids.find(file_obs.image_name);
    if (it == image_ids.end()) {
      ++unresolved;
      continue;
    }
    VirtualObservation obs;
    obs.image_id = it->second;
    obs.xy = file_obs.xy;
    obs.weight = file_obs.weight;
    tracks[file_obs.track_id].observations.push_back(obs);
  }

  std::vector<VirtualTrack> result;
  result.reserve(tracks.size());
  for (auto& [track_id, track] : tracks) {
    if (track.observations.size() >= 2) {
      result.push_back(std::move(track));
    }
  }
  if (num_unresolved != nullptr) {
    *num_unresolved = unresolved;
  }
  return result;
}

}  // namespace colmap
