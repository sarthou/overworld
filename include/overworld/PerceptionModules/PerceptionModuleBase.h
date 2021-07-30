#ifndef OWDS_PERCEPTIONMODULEBASE_H
#define OWDS_PERCEPTIONMODULEBASE_H

#include <map>
#include <mutex>
#include <memory>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "overworld/BasicTypes/Entity.h"

namespace owds {

template<typename T>
class PerceptionModuleBase_
{
  static_assert(std::is_base_of<Entity,T>::value, "T must be derived from Entity");
public:
  PerceptionModuleBase_()
  {
    is_activated_ = true;
    updated_ = false;
  }
  virtual ~PerceptionModuleBase_() = default;

  void activate(bool is_activated) { is_activated_ = is_activated_; }
  bool isActiavted() { return is_activated_; }
  bool hasBeenUpdated() { return updated_; }

  void accessPercepts(const std::function<void(const std::map<std::string, T>&)>& accessor)
  {
    mutex_.lock();
    accessor(percepts_);
    updated_ = false;
    mutex_.unlock();
  }

protected:
  std::map<std::string, T> percepts_;
  bool is_activated_;
  bool updated_;
  std::mutex mutex_;
};

template<typename T, class M>
class PerceptionModuleBase : protected PerceptionModuleBase_<T>
{
public:
  virtual ~PerceptionModuleBase() = default;
  void sendPerception(const M& msg) { privatePerceptionCallback(msg); }

protected:
  virtual void perceptionCallback(const M& msg) = 0;

private:
  void privatePerceptionCallback(const M& msg)
  {
    if(!this->is_activated_)
      return;

    this->mutex_.lock();
    perceptionCallback(msg);
    this->updated_ = true;
    this->mutex_.unlock();
  }
};

template<typename T>
class PerceptionModuleRosBase_ : protected PerceptionModuleBase_<T>
{
public:
  PerceptionModuleRosBase_(ros::NodeHandle* n) { n_ = n; }
  virtual ~PerceptionModuleRosBase_() = default;
protected:
  ros::NodeHandle* n_;
};


template<typename T, class M>
class PerceptionModuleRosBase : protected PerceptionModuleRosBase_<T>
{
public:
  PerceptionModuleRosBase(ros::NodeHandle* n, const std::string& topic_name) : PerceptionModuleRosBase_<T>(n)
  {
    sub_ = this->n_->subscribe(topic_name, 1, &PerceptionModuleRosBase::privatePerceptionCallback, this);
    std::cout << "PerceptionModuleRosBase subscribed to " << topic_name << std::endl;
  }
  virtual ~PerceptionModuleRosBase() = default;

protected:
  virtual void perceptionCallback(const M& msg) = 0;

private:
  ros::Subscriber sub_;

  void privatePerceptionCallback(const M& msg)
  {
    if(!this->is_activated_)
      return;

    this->mutex_.lock();
    perceptionCallback(msg);
    this->updated_ = true;
    this->mutex_.unlock();
  }
};

template<typename T, class M0, class M1>
class PerceptionModuleRosSyncBase : protected PerceptionModuleRosBase_<T>
{
public:
  PerceptionModuleRosSyncBase(ros::NodeHandle* n,
                              const std::string& first_topic_name,
                              const std::string& second_topic_name): PerceptionModuleRosBase_<T>(n)
  {
    sub_0_.subscribe(*n, first_topic_name, 1);
    sub_1_.subscribe(*n, second_topic_name, 1);
    sync_.reset(new Sync(SyncPolicy(10), sub_0_, sub_1_));
    sync_->registerCallback(&PerceptionModuleRosSyncBase::privatePerceptionCallback, this);
    std::cout << "PerceptionModuleRosSyncBase subscribed to " << first_topic_name << " and " << second_topic_name << std::endl;
  }
  virtual ~PerceptionModuleRosSyncBase() = default;

protected:
  virtual void perceptionCallback(const M0& first_msg, const M1& second_msg) = 0;

private:
  message_filters::Subscriber<M0> sub_0_;
  message_filters::Subscriber<M1> sub_1_;
  typedef message_filters::sync_policies::ApproximateTime<M0, M1> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  void privatePerceptionCallback(const M0& first_msg, const M1& second_msg)
  {
    if(!this->is_activated_)
      return;

    this->mutex_.lock();
    perceptionCallback(first_msg, second_msg);
    this->updated_ = true;
    this->mutex_.unlock();
  }
};

} // namespace owds

#endif // OWDS_PERCEPTIONMODULEBASE_H