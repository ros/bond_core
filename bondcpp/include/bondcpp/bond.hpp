/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Stuart Glaser */

#ifndef BONDCPP__BOND_HPP_
#define BONDCPP__BOND_HPP_

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "bond/msg/constants.hpp"
#include "bond/msg/status.hpp"

#include "bondcpp/BondSM_sm.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace bond
{
/** \brief Forms a bond to monitor another process.
 *
 * The bond::Bond class implements a bond, allowing you to monitor
 * another process and be notified when it dies.  In turn, it will be
 * notified when you die.
 */
class Bond
{
public:
  /** \brief Constructs a bond, but does not connect
   *
   * \param topic The topic used to exchange the bond status messages.
   * \param id The ID of the bond, which should match the ID used on
   *           the sister's end.
   * \param nh Lifecycle Node shared ptr.
   * \param on_broken callback that will be called when the bond is broken.
   * \param on_formed callback that will be called when the bond is formed.
   */
  Bond(
    const std::string & topic, const std::string & id,
    rclcpp_lifecycle::LifecycleNode::SharedPtr nh,
    std::function<void(void)> on_broken = std::function<void(void)>(),
    std::function<void(void)> on_formed = std::function<void(void)>());

  /** \brief Constructs a bond, but does not connect
   *
   * \param topic The topic used to exchange the bond status messages.
   * \param id The ID of the bond, which should match the ID used on
   *           the sister's end.
   * \param nh Node shared ptr.
   * \param on_broken callback that will be called when the bond is broken.
   * \param on_formed callback that will be called when the bond is formed.
   */
  Bond(
    const std::string & topic, const std::string & id,
    rclcpp::Node::SharedPtr nh,
    std::function<void(void)> on_broken = std::function<void(void)>(),
    std::function<void(void)> on_formed = std::function<void(void)>());

  /** \brief Destructs the object, breaking the bond if it is still formed.
   */
  ~Bond();

  void setupConnections();

  double getConnectTimeout() const {return connect_timeout_;}
  void setConnectTimeout(double dur);
  void connectTimerReset();
  void connectTimerCancel();
  double getDisconnectTimeout() const {return disconnect_timeout_;}
  void setDisconnectTimeout(double dur);
  void disconnectTimerReset();
  void disconnectTimerCancel();
  double getHeartbeatTimeout() const {return heartbeat_timeout_;}
  void setHeartbeatTimeout(double dur);
  void heartbeatTimerReset();
  void heartbeatTimerCancel();
  double getHeartbeatPeriod() const {return heartbeat_period_;}
  void setHeartbeatPeriod(double dur);
  void publishingTimerReset();
  void publishingTimerCancel();
  double getDeadPublishPeriod() const {return dead_publish_period_;}
  void setDeadPublishPeriod(double dur);
  void deadpublishingTimerReset();
  void deadpublishingTimerCancel();

  /** \brief Starts the bond and connects to the sister process.
   */
  void start();
  /** \brief Sets the formed callback.
   */
  void setFormedCallback(std::function<void(void)> on_formed);

  /** \brief Sets the broken callback
   */
  void setBrokenCallback(std::function<void(void)> on_broken);

  /** \brief Blocks until the bond is formed for at most 'duration'.
   *    Assumes the node to be spinning in the background
   *
   * \param timeout Maximum duration to wait.  If -1 then this call will not timeout.
   * \return true iff the bond has been formed.
   */
  bool waitUntilFormed(rclcpp::Duration timeout = rclcpp::Duration(-1 * 1e9));
  /** \brief Blocks until the bond is broken for at most 'duration'.
   *    Assumes the node to be spinning in the background
   *
   * \param timeout Maximum duration to wait.  If -1 then this call will not timeout.
   * \return true iff the bond has been broken, even if it has never been formed.
   */
  bool waitUntilBroken(rclcpp::Duration timeout = rclcpp::Duration(-1 * 1e9));
  /** \brief Indicates if the bond is broken.
   */
  bool isBroken();
  /** \brief Breaks the bond, notifying the other process.
   */
  void breakBond();
  std::string getTopic() {return topic_;}
  std::string getId() {return id_;}
  std::string getInstanceId() {return instance_id_;}

private:
  friend struct ::BondSM;

  void onConnectTimeout();
  void onHeartbeatTimeout();
  void onDisconnectTimeout();

  void bondStatusCB(const bond::msg::Status::ConstSharedPtr msg);

  void doPublishing();
  void publishStatus(bool active);

  std::vector<std::function<void(void)>> pending_callbacks_;
  void flushPendingCallbacks();

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

  rclcpp::TimerBase::SharedPtr connect_timer_;
  rclcpp::TimerBase::SharedPtr disconnect_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr publishing_timer_;
  rclcpp::TimerBase::SharedPtr deadpublishing_timer_;

  std::unique_ptr<BondSM> bondsm_;
  BondSMContext sm_;

  std::string topic_;
  std::string id_;
  std::string instance_id_;
  std::string sister_instance_id_;
  std::function<void(void)> on_broken_;
  std::function<void(void)> on_formed_;
  bool sisterDiedFirst_;
  bool started_;
  bool connect_timer_reset_flag_;
  bool disconnect_timer_reset_flag_;
  bool deadpublishing_timer_reset_flag_;
  bool disable_heartbeat_timeout_;
  std::mutex mutex_;
  std::condition_variable condition_;

  double connect_timeout_;
  double heartbeat_timeout_;
  double disconnect_timeout_;
  double heartbeat_period_;
  double dead_publish_period_;

  rclcpp::Subscription<bond::msg::Status>::SharedPtr sub_;
  rclcpp::Publisher<bond::msg::Status>::SharedPtr pub_;
};

}  // namespace bond


// Internal use only
struct BondSM
{
  explicit BondSM(bond::Bond * b_)
  : b(b_) {}

  void Connected();
  void SisterDied();
  void Death();
  void Heartbeat();
  void StartDying();

private:
  bond::Bond * b;
};

#endif  // BONDCPP__BOND_HPP_
