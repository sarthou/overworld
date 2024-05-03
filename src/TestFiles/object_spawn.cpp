#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ontologenius/OntologyManipulator.h"
#include "overworld/Perception/Modules/ObjectsModules/FakeObjectPerceptionModule.h"
#include "overworld/BasicTypes/Object.h"
#include "overworld/EntitiesPoses.h"
#include "geometry_msgs/PoseStamped.h"


onto::OntologyManipulator* onto_;
std::vector<std::string> names_;


overworld::EntityPose createObject(double x, double y, double z, const std::string& name, const std::string& type)
{
  geometry_msgs::PoseStamped pose_stamped_msg;
  
  pose_stamped_msg.header.seq = 0;
  pose_stamped_msg.header.stamp = ros::Time(0);
  pose_stamped_msg.header.frame_id = "map";
  pose_stamped_msg.pose.position.x = x;
  pose_stamped_msg.pose.position.y = y;
  pose_stamped_msg.pose.position.z = z;
  pose_stamped_msg.pose.orientation.x = 0.0;
  pose_stamped_msg.pose.orientation.y = 0.0;
  pose_stamped_msg.pose.orientation.z = 0.0;
  pose_stamped_msg.pose.orientation.w = 0.0;
  
  overworld::EntityPose entity_msg;
  entity_msg.id = name;
  entity_msg.pose = pose_stamped_msg;

  if(type != "")
  {
    onto_->feeder.waitConnected();
    onto_->feeder.addInheritage(entity_msg.id, type);
    onto_->feeder.waitUpdate(100);
  }

  return entity_msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_object_publisher");

  onto::OntologyManipulator onto("eve");
  onto_ = &onto;

  onto.close();

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<overworld::EntitiesPoses>("/overworld/fake_objects_poses", 1000);
  ros::Publisher false_pub = n.advertise<overworld::EntitiesPoses>("/overworld/fake_false_objects_poses", 1000);
  ros::Rate r(10);


  // false object passe sur un true object: merge des objects et le sylo bouge
  overworld::EntitiesPoses entities;
  entities.entities = {createObject(2.0, 4.0, 1.75, "p1", "Pen"), createObject(2.0, 4.0, 1.75, "p2", "Mug")};
  overworld::EntitiesPoses false_entities;
  false_entities.entities = {createObject(3.0, 2.0, 1.75, "f1", "")};

  for(int i = 0; i < 200; i++)
  {
    pub.publish(entities);
    entities.entities[0].pose.pose.position.y -= 0.01;
    entities.entities[1].pose.pose.position.y += 0.01;
    ros::spinOnce();
    r.sleep();
  }

  for(int i = 0; i < 200; i++)
  {
    false_pub.publish(false_entities);
    false_entities.entities[0].pose.pose.position.x -= 0.01;
    ros::spinOnce();
    r.sleep();
  }

    return 0;
}

/*  // 1 false objects croise un autre fixe
  overworld::EntitiesPoses false_entities;
  false_entities.entities = {createObject(3.0, 3.0, 1.75, "f1", ""),createObject(2.0, 3.0, 1.75, "f2", "")};

  for(int i = 0; i < 200; i++)
  {
    pub.publish(false_entities);
    false_entities.entities[0].pose.pose.position.x -= 0.01;
    ros::spinOnce();
    r.sleep();
  }
*/



/*
  // 2 false objects se croisent et ne se mergent pas 
  overworld::EntitiesPoses false_entities;
  false_entities.entities = {createObject(3.0, 3.0, 1.75, "f1", ""),createObject(2.0, 2.0, 1.75, "f2", "")};

  for(int i = 0; i < 200; i++)
  {
    pub.publish(false_entities);
    false_entities.entities[0].pose.pose.position.x -= 0.01;
    false_entities.entities[1].pose.pose.position.y += 0.01;
    ros::spinOnce();
    r.sleep();
  }
*/

/*
  //stylo qui bouge sur un false object: merge des objects
  overworld::EntitiesPoses entities;
  entities.entities = {createObject(2.0, 4.0, 1.75, "p1", "Pen"), createObject(2.0, 4.0, 1.75, "p2", "Mug")};
  overworld::EntitiesPoses false_entities;
  false_entities.entities = {createObject(3.0, 2.0, 1.75, "f1", "")};

    for(int i = 0; i < 100; i++)
  {
    false_pub.publish(false_entities);
    false_entities.entities[0].pose.pose.position.x -= 0.01;
    ros::spinOnce();
    r.sleep();
  }

  for(int i = 0; i < 300; i++)
  {
    pub.publish(entities);
    entities.entities[0].pose.pose.position.y -= 0.01;
    entities.entities[1].pose.pose.position.y += 0.01;
    ros::spinOnce();
    r.sleep();
  }
*/

/*
  // stylo qui vient se cacher derriere un mur
  overworld::EntitiesPoses entities;
  entities.entities = {createObject(5.0, 5.5, 1.4, "p1", "Pen")};

  for(int i = 0; i < 200; i++)
  {
    pub.publish(entities);
    entities.entities[0].pose.pose.position.x -= 0.01;
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
*/

/*
  // stylo cache derriere un mur
  overworld::EntitiesPoses entities;
  entities.entities = {createObject(3.0, 5.5, 1.4, "p1", "Pen")};
  pub.publish(entities);
  ros::spinOnce();
  r.sleep();

  return 0;
}
*/

// les false objects ne se mergent pas entre eux dans tous les cas