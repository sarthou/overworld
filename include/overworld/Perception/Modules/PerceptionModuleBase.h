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

#include "ontologenius/OntologiesManipulator.h"

#include "overworld/BasicTypes/Entity.h"
#include "overworld/BasicTypes/Agent.h"

#include "overworld/Bullet/PhysicsServers.h"

#include "overworld/Utility/ShellDisplay.h"

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

  virtual void initialize(ros::NodeHandle* n,
                          BulletClient* bullet_client,
                          int robot_bullet_id,
                          Agent* robot_agent)
  {
    n_ = n;
    bullet_client_ = bullet_client;
    robot_bullet_id_ = robot_bullet_id;
    robot_agent_ = robot_agent;
  }

  virtual void setParameter(const std::string& parameter_name, const std::string& parameter_value) {}
  virtual bool closeInitialization() { return true; }

  virtual std::string getAgentName() { return ""; } 
  virtual int getAgentBulletId() { return -1; }

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

  static std::array<double, 3> getEntityColorFromOntology(OntologyManipulator* onto, const std::string& indiv_name, const std::array<double, 3>& default_value = {0.8,0.8,0.8})
  {
    auto color = onto->individuals.getOn(indiv_name, "hasColor");
    if(color.size() != 0)
    {
      auto hex_value = onto->individuals.getOn(color.front(), "hexRgbValue");
      if(hex_value.size())
      {
        int hex = 0;
        sscanf(hex_value[0].substr(hex_value[0].find("#") + 1).c_str(), "%x", &hex);
        return { ((hex >> 16) & 0xff) / 255., ((hex >> 8) & 0xff) / 255., (hex & 0xff) / 255. };
      }
    }
    
    return default_value;
  }

  static Shape_t getEntityShapeFromOntology(OntologyManipulator* onto, const std::string& indiv_name)
  {
    auto visual_meshes = onto->individuals.getOn(indiv_name, "hasVisualMesh");
    auto colision_meshes = onto->individuals.getOn(indiv_name, "hasColisionMesh");
    auto meshes = onto->individuals.getOn(indiv_name, "hasMesh");
    auto textures = onto->individuals.getOn(indiv_name, "hasTexture");

    Shape_t shape;
    if(meshes.size())
    {
      shape.type = SHAPE_MESH;
      if(visual_meshes.size())
          shape.visual_mesh_resource = visual_meshes.front().substr(visual_meshes.front().find("#") + 1);
      else
          shape.visual_mesh_resource = meshes.front().substr(meshes.front().find("#") + 1);
      if(colision_meshes.size())
          shape.colision_mesh_resource = colision_meshes.front().substr(colision_meshes.front().find("#") + 1);
      else
          shape.colision_mesh_resource = meshes.front().substr(meshes.front().find("#") + 1);
      shape.color = getEntityColorFromOntology(onto, indiv_name);
      if(textures.size())
          shape.texture = textures.front().substr(textures.front().find("#") + 1);
    }
    else
      shape.type = SHAPE_NONE;

    return shape;
  }

protected:
  std::map<std::string, T> percepts_;
  std::atomic<bool> is_activated_;
  std::atomic<bool> updated_;
  std::mutex mutex_perception_;
  static std::mutex mutex_access_;
  bool need_access_to_external_entities_;

  ros::NodeHandle* n_;
  BulletClient* bullet_client_;
  int robot_bullet_id_;
  Agent* robot_agent_;

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

template<typename T, class M>
class PerceptionModuleRosBase : public PerceptionModuleBase_<T>
{
public:
  PerceptionModuleRosBase(const std::string& topic_name, bool need_access_to_external_entities = false) : PerceptionModuleBase_<T>(need_access_to_external_entities)
  {
    topic_name_ = topic_name;
  }
  virtual ~PerceptionModuleRosBase() = default;

  virtual void initialize(ros::NodeHandle* n,
                          BulletClient* bullet_client,
                          int robot_bullet_id,
                          Agent* robot_agent)
  {
    PerceptionModuleBase_<T>::initialize(n, bullet_client, robot_bullet_id, robot_agent);
    if(topic_name_ != "")
    {
      sub_ = this->n_->subscribe(topic_name_, 1, &PerceptionModuleRosBase::privatePerceptionCallback, this);
      ShellDisplay::info("PerceptionModuleRosBase subscribed to " + topic_name_);
    }
  }

protected:
  virtual bool perceptionCallback(const M& msg) = 0;

  void setTopicName(const std::string& topic_name)
  {
    topic_name_ = topic_name;
    if(topic_name_ != "")
    {
      sub_ = this->n_->subscribe(topic_name_, 1, &PerceptionModuleRosBase::privatePerceptionCallback, this);
      ShellDisplay::info("PerceptionModuleRosBase subscribed to " + topic_name_);
    }
  }

private:
  ros::Subscriber sub_;
  std::string topic_name_;

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
class PerceptionModuleRosSyncBase : public PerceptionModuleBase_<T>
{
public:
  PerceptionModuleRosSyncBase(const std::string& first_topic_name,
                              const std::string& second_topic_name,
                              bool need_access_to_external_entities = false): PerceptionModuleBase_<T>(need_access_to_external_entities)
  {
    first_topic_name_ = first_topic_name;
    second_topic_name_ = second_topic_name;
  }
  virtual ~PerceptionModuleRosSyncBase() = default;

  virtual void initialize(ros::NodeHandle* n,
                          BulletClient* bullet_client,
                          int robot_bullet_id,
                          Agent* robot_agent)
  {
    PerceptionModuleBase_<T>::initialize(n, bullet_client, robot_bullet_id, robot_agent);
    sub_0_.subscribe(*n, first_topic_name_, 1);
    sub_1_.subscribe(*n, second_topic_name_, 1);
    sync_.reset(new Sync(SyncPolicy(10), sub_0_, sub_1_));
    sync_->registerCallback(&PerceptionModuleRosSyncBase::privatePerceptionCallback, this);
    ShellDisplay::info("PerceptionModuleRosSyncBase subscribed to " + first_topic_name_ + " and " + second_topic_name_);
  }

protected:
  virtual bool perceptionCallback(const M0& first_msg, const M1& second_msg) = 0;

private:
  std::string first_topic_name_;
  std::string second_topic_name_;
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