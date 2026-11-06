#ifndef BIEVR_ROS_COMMON_PUBLISHER_BASE_H_
#define BIEVR_ROS_COMMON_PUBLISHER_BASE_H_

// ROS-version-agnostic publisher logic shared by the ROS1 and ROS2 wrappers.
// Everything that genuinely differs between the two (the node handle, the
// type-erased publisher mechanism, the concrete message types and the TF
// broadcaster) is supplied by a `Backend` policy; the dispatch, lazy
// advertising, namespace handling and pipeline registration live here once.
//
// The message conversions (headerToMsg / pointCloudToMsg / transformToMsg /
// vecToMsg) are the version-agnostic templates from conversions.h, included
// below so they resolve by ordinary lookup at this template's definition. They
// build PointCloud2 messages via sensor_msgs::PointCloud2Modifier, so the TU
// that instantiates a Publisher must have included the matching
// sensor_msgs/point_cloud2_iterator header (the wrappers' conversions.h, pulled
// in by publisher.h, does this).

#include <functional>
#include <memory>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <unordered_set>

#include "bievr_lio/common.h"
#include "bievr_lio/log++.h"
#include "bievr_lio/pipeline.h"
#include "bievr_ros_common/conversions.h"

namespace bievr {

template <typename Backend>
class PublisherBase {
 public:
  using Handle = typename Backend::Handle;

  // If ns is non-empty, every topic published through this object is advertised
  // under that namespace (e.g. ns="bievr_lio" turns "odometry" into
  // "/bievr_lio/odometry"). Absolute topics (leading '/') are left untouched.
  PublisherBase(Handle handle, std::shared_ptr<Pipeline> pipeline, const std::string& ns = "")
      : backend_(std::move(handle)), ns_(ns) {
    registerTypes<Pointcloud, IntensityPointcloud, Odometry, V3>(pipeline);
  }
  virtual ~PublisherBase() = default;

  template <typename T>
  bool publish(const T& data, const Header& header, const std::string& topic,
               const std::string& child_frame = "") {
    if (disabled_topics_.count(topic)) return true;
    return publishImpl(data, header, topic, child_frame);
  }

  // Report odometry for `frame` (e.g. base_link) instead of the IMU. T_B_I is
  // the IMU pose in that frame. The odometry origin is moved with it, so the
  // output is the motion of `frame` starting from the identity, like a LIO run
  // directly in that frame: T_O_B = T_B_I * T_W_I * T_B_I^-1.
  void setOutputFrame(const std::string& frame, const Transform& T_B_I) {
    output_frame_ = frame;
    T_B_I_ = T_B_I;
  }

  void setPublishTf(bool publish_tf) { publish_tf_ = publish_tf; }

  // Pipeline topics (e.g. "points/registered") that should not be published.
  void disableTopic(const std::string& topic) { disabled_topics_.insert(topic); }

 private:
  // Pointcloud and IntensityPointcloud both go out as PointCloud2.
  bool publishImpl(const Pointcloud& cloud, const Header& header, const std::string& topic,
                   const std::string& /*child_frame*/) {
    typename Backend::PointCloud2 msg;
    if (!getOrAdvertise<typename Backend::PointCloud2>(topic)) return false;
    headerToMsg(header, msg.header);
    pointCloudToMsg(cloud, msg);
    publishers_[topic].publish(msg);
    return true;
  }

  bool publishImpl(const IntensityPointcloud& cloud, const Header& header, const std::string& topic,
                   const std::string& /*child_frame*/) {
    typename Backend::PointCloud2 msg;
    if (!getOrAdvertise<typename Backend::PointCloud2>(topic)) return false;
    headerToMsg(header, msg.header);
    pointCloudToMsg(cloud, msg);
    publishers_[topic].publish(msg);
    return true;
  }

  bool publishImpl(const Odometry& odometry, const Header& header, const std::string& topic,
                   const std::string& child_frame) {
    typename Backend::Odometry odom_msg;
    if (!getOrAdvertise<typename Backend::Odometry>(topic)) return false;
    headerToMsg(header, odom_msg.header);
    Transform pose = odometry.pose;
    V3 v = odometry.linear_velocity, w = odometry.angular_velocity;
    if (output_frame_.empty()) {
      odom_msg.child_frame_id = child_frame;
    } else {
      odom_msg.child_frame_id = output_frame_;
      pose = Transform(T_B_I_ * odometry.pose * T_B_I_.inverse());
      // Body-frame twist of B: v_B = R_BI (v_I + w_I x p_IB), w_B = R_BI w_I,
      // with p_IB the origin of B in the IMU frame.
      const V3 p_IB = T_B_I_.inverse().translation();
      v = T_B_I_.linear() * (v + w.cross(p_IB));
      w = T_B_I_.linear() * w;
    }
    transformToMsg(pose, odom_msg.pose.pose);
    // Twist is expressed in the child (body) frame.
    vecToMsg(v, odom_msg.twist.twist.linear);
    vecToMsg(w, odom_msg.twist.twist.angular);
    publishers_[topic].publish(odom_msg);

    if (publish_tf_) {
      // Mirror the pose as a TF transform.
      typename Backend::TransformStamped transform_msg;
      transform_msg.header = odom_msg.header;
      transform_msg.child_frame_id = odom_msg.child_frame_id;
      transformToMsg(pose, transform_msg.transform);
      backend_.sendTransform(transform_msg);
    }
    return true;
  }

  bool publishImpl(const V3& vec, const Header& header, const std::string& topic,
                   const std::string& /*child_frame*/) {
    typename Backend::Vector3Stamped msg;
    if (!getOrAdvertise<typename Backend::Vector3Stamped>(topic)) return false;
    headerToMsg(header, msg.header);
    vecToMsg(vec, msg.vector);
    publishers_[topic].publish(msg);
    return true;
  }

  // Look up (or lazily advertise) the publisher for `topic`, verifying that its
  // message type matches any previously advertised one. Returns false on a
  // type mismatch (the message is then dropped by the caller).
  template <typename MsgT>
  bool getOrAdvertise(const std::string& topic) {
    auto it = publishers_.find(topic);
    if (it == publishers_.end()) {
      backend_.template advertise<MsgT>(publishers_[topic], resolveTopic(topic));
    } else if (!it->second.isSameType(typeid(MsgT))) {
      LOG(W, "Publisher type mismatch for topic " << topic);
      return false;
    }
    return true;
  }

  template <typename T>
  void registerWithPipeline(std::shared_ptr<Pipeline> pipeline) {
    pipeline->registerPublisher<T>(std::bind_front(&PublisherBase::publish<T>, this));
  }

  template <typename... Ts>
  void registerTypes(std::shared_ptr<Pipeline> pipeline) {
    (registerWithPipeline<Ts>(pipeline), ...);  // fold expression
  }

  // Prepends the namespace to a relative topic. Absolute topics (leading '/')
  // and the empty-namespace case are returned unchanged.
  std::string resolveTopic(const std::string& topic) const {
    if (ns_.empty() || topic.empty() || topic.front() == '/') {
      return topic;
    }
    return "/" + ns_ + "/" + topic;
  }

  Backend backend_;
  std::string ns_;
  std::string output_frame_;
  Transform T_B_I_;
  bool publish_tf_ = true;
  std::unordered_set<std::string> disabled_topics_;
  std::unordered_map<std::string, typename Backend::TypedPublisher> publishers_;
};

}  // namespace bievr

#endif  // BIEVR_ROS_COMMON_PUBLISHER_BASE_H_
