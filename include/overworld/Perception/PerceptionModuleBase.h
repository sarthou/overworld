#ifndef OWDS_PERCEPTIONMODULEBASE_H
#define OWDS_PERCEPTIONMODULEBASE_H

#include <atomic>
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
  PerceptionModuleBase_(bool need_access_to_external_entities = false)
  {
    need_access_to_external_entities_ = need_access_to_external_entities;
    is_activated_ = true;
    updated_ = false;
  }
  virtual ~PerceptionModuleBase_() = default;

  void activate(bool is_activated) { is_activated_ = is_activated; }
  bool isActivated() { return is_activated_; }
  bool hasBeenUpdated() { return updated_; }

  void accessPercepts(const std::function<void(std::map<std::string, T>&)>& accessor)
  {
    mutex_perception_.lock();
    if(need_access_to_external_entities_)
      mutex_access_.lock();
    accessor(percepts_);
    updated_ = false;
    if(need_access_to_external_entities_)
      mutex_access_.unlock();
    mutex_perception_.unlock();
  }

protected:
  std::map<std::string, T> percepts_;
  std::atomic<bool> is_activated_;
  std::atomic<bool> updated_;
  std::mutex mutex_perception_;
  static std::mutex mutex_access_;
  bool need_access_to_external_entities_;

  void setAllPerceptsUnseen()
  {
    for(auto& percept : percepts_)
      percept.second.setUnseen();
  }
};

template<typename T>
std::mutex PerceptionModuleBase_<T>::mutex_access_;

template<typename T, class M>
class PerceptionModuleBase : public PerceptionModuleBase_<T>
{
public:
  PerceptionModuleBase(bool need_access_to_external_entities = false) : PerceptionModuleBase_<T>(need_access_to_external_entities) {}
  virtual ~PerceptionModuleBase() = default;
  void sendPerception(const M& msg) { privatePerceptionCallback(msg); }

protected:
  virtual bool perceptionCallback(const M& msg) = 0;

private:
  void privatePerceptionCallback(const M& msg)
  {
    if(!this->is_activated_)
      return;

    if(this->need_access_to_external_entities_)
    {
      this->mutex_perception_.lock();
      this->mutex_access_.lock();
      this->mutex_perception_.unlock();
    }
    else
      this->mutex_perception_.lock();
    
    if(perceptionCallback(msg))
      this->updated_ = true;

    if(this->need_access_to_external_entities_)
      this->mutex_access_.unlock();
    else
      this->mutex_perception_.unlock();
  }
};

template<typename T>
class PerceptionModuleRosBase_ : public PerceptionModuleBase_<T>
{
public:
  PerceptionModuleRosBase_(ros::NodeHandle* n, bool need_access_to_external_entities = false) : PerceptionModuleBase_<T>(need_access_to_external_entities)
  {
    n_ = n;
  }
  virtual ~PerceptionModuleRosBase_() = default;
protected:
  ros::NodeHandle* n_;
};


template<typename T, class M>
class PerceptionModuleRosBase : public PerceptionModuleRosBase_<T>
{
public:
  PerceptionModuleRosBase(ros::NodeHandle* n, const std::string& topic_name, bool need_access_to_external_entities = false) : PerceptionModuleRosBase_<T>(n, need_access_to_external_entities)
  {
    sub_ = this->n_->subscribe(topic_name, 1, &PerceptionModuleRosBase::privatePerceptionCallback, this);
    std::cout << "PerceptionModuleRosBase subscribed to " << topic_name << std::endl;
  }
  virtual ~PerceptionModuleRosBase() = default;

protected:
  virtual bool perceptionCallback(const M& msg) = 0;

private:
  ros::Subscriber sub_;

  void privatePerceptionCallback(const M& msg)
  {
    if(!this->is_activated_)
      return;

    if(this->need_access_to_external_entities_)
    {
      this->mutex_perception_.lock();
      this->mutex_access_.lock();
      this->mutex_perception_.unlock();
    }
    else
      this->mutex_perception_.lock();

    if(perceptionCallback(msg))
      this->updated_ = true;

    if(this->need_access_to_external_entities_)
      this->mutex_access_.unlock();
    else
      this->mutex_perception_.unlock();
  }
};

template<typename T, class M0, class M1>
class PerceptionModuleRosSyncBase : public PerceptionModuleRosBase_<T>
{
public:
  PerceptionModuleRosSyncBase(ros::NodeHandle* n,
                              const std::string& first_topic_name,
                              const std::string& second_topic_name,
                              bool need_access_to_external_entities = false): PerceptionModuleRosBase_<T>(n, need_access_to_external_entities)
  {
    sub_0_.subscribe(*n, first_topic_name, 1);
    sub_1_.subscribe(*n, second_topic_name, 1);
    sync_.reset(new Sync(SyncPolicy(10), sub_0_, sub_1_));
    sync_->registerCallback(&PerceptionModuleRosSyncBase::privatePerceptionCallback, this);
    std::cout << "PerceptionModuleRosSyncBase subscribed to " << first_topic_name << " and " << second_topic_name << std::endl;
  }
  virtual ~PerceptionModuleRosSyncBase() = default;

protected:
  virtual bool perceptionCallback(const M0& first_msg, const M1& second_msg) = 0;

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

    if(this->need_access_to_external_entities_)
    {
      this->mutex_perception_.lock();
      this->mutex_access_.lock();
      this->mutex_perception_.unlock();
    }
    else
      this->mutex_perception_.lock();

    if(perceptionCallback(first_msg, second_msg))
      this->updated_ = true;

    if(this->need_access_to_external_entities_)
      this->mutex_access_.unlock();
    else
      this->mutex_perception_.unlock();
  }
};

} // namespace owds

#endif // OWDS_PERCEPTIONMODULEBASE_H