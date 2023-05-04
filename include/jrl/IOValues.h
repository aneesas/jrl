#pragma once
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;
namespace jrl {

static const std::string Pose2Tag = "Pose2";
static const std::string Pose3Tag = "Pose3";
static const std::string Point2Tag = "Point2";
static const std::string Point3Tag = "Point3";
static const std::string VectorTag = "Vector";
static const std::string ScalarTag = "Scalar";

namespace io_values {

template <typename VALUE>
void valueAccumulator(std::function<VALUE(json)> parser, json input_json, gtsam::Key key, gtsam::Values& accum) {
  accum.insert(key, parser(input_json));
}

/**********************************************************************************************************************/
template <typename T>
T parse(json input_json) {  // Base Parse function for builtins. Specialization provided for all iovalues
  return input_json["value"].get<T>();
}

template <typename T>
json serialize(T obj) {  // Base Parse function for builtins. Specialization provided for all iovalues
  json output;
  output["type"] = ScalarTag;
  output["value"] = obj;
  return output;
}

/**********************************************************************************************************************/
// POSE2
template <>
gtsam::Pose2 parse<gtsam::Pose2>(json input_json);
template <>
json serialize<gtsam::Pose2>(gtsam::Pose2 pose);

/**********************************************************************************************************************/
// POSE3
template <>
gtsam::Pose3 parse<gtsam::Pose3>(json input_json);
template <>
json serialize<gtsam::Pose3>(gtsam::Pose3 pose);

/**********************************************************************************************************************/
// VECTOR
template <>
gtsam::Vector parse<gtsam::Vector>(json input_json);
template <>
json serialize<gtsam::Vector>(gtsam::Vector vec);

/**********************************************************************************************************************/
// Point2
template <>
gtsam::Point2 parse<gtsam::Point2>(json input_json);
template <>
json serialize<gtsam::Point2>(gtsam::Point2 point);

/**********************************************************************************************************************/
// Point3
template <>
gtsam::Point3 parse<gtsam::Point3>(json input_json);
template <>
json serialize<gtsam::Point3>(gtsam::Point3 point);

}  // namespace io_values

}  // namespace jrl