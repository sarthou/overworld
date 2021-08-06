#include "overworld/Facts/FactsCalculator.h"

namespace owds {

std::vector<Fact> FactsCalculator::computeFacts(const std::map<std::string, Object*>& objects,
                                                const std::map<std::string, Agent*>& agents)
{
  facts_.clear();

  for(auto& obj_from : objects)
  {
    if(obj_from.second->isInHand())
      continue;

    for(auto& obj_to : objects)
    {
      if(obj_to.second->isInHand())
        continue;

      if(obj_from.first != obj_to.first)
      {
        bool is_in = isInContainer(obj_to.second, obj_from.second);
        if(is_in == false)
          isOnTopfOf(obj_to.second, obj_from.second);
      }
    }
  }

  for(auto& agent_from : agents)
  {
    isInHand(agent_from.second);
    isLookingAt(agent_from.second);
    for(auto& agent_to : agents)
    {
      if(agent_from.first != agent_to.first)
      {
        isPerceiving(agent_from.second, agent_to.second);
      }
    }
  }

  return facts_;
}

bool FactsCalculator::isOnTopfOf(Object* object_under, Object* object_on)
{
  return false;
}

bool FactsCalculator::isInContainer(Object* object_around, Object* object_in)
{
  return false;
}

bool FactsCalculator::isInHand(Agent* agent)
{
  return false;
}

bool FactsCalculator::isPerceiving(Agent* agent_perceiving, Agent* agent_perceived)
{
  return false;
}

bool FactsCalculator::isLookingAt(Agent* agent/*, segmentation_image*/)
{
  return false;
}

} // namespace owds

/*
def is_on_top(bb1, bb2):
    """ For obj 1 to be on top of obj 2:
         - obj1 must be above obj 2
         - the bottom of obj 1 must be close to the top of obj 2
    """
    bb1_min, _ = bb1
    _, bb2_max = bb2

    x1,y1,z1 = bb1_min
    x2,y2,z2 = bb2_max
    return z1 < z2 + ONTOP_EPSILON and is_above(bb1, bb2)

def is_above(bb1, bb2):
    """ For obj 1 to be above obj 2:
         - the bottom of its bounding box must be higher that
           the top of obj 2's bounding box
         - the bounding box footprint of both objects must overlap
    """

    bb1_min, _ = bb1
    _, bb2_max = bb2

    x1,y1,z1 = bb1_min
    x2,y2,z2 = bb2_max
    if z1 < z2 - ISABOVE_EPSILON:
        return False

    return overlap(bb_footprint(bb1),
                   bb_footprint(bb2))


def bb_footprint(bb):
    """ Returns a rectangle that defines the bottom face of a bounding box
    """
    x1,y1,z1 = bb[0]
    x2,y2,z2 = bb[1]

    return (x1,y1), (x2,y2)

def overlap(bbox_a, bbox_b):
    """Returns the overlap ratio"""
    xa = int(max(bbox_a.xmin, bbox_b.xmin))
    ya = int(max(bbox_a.ymin, bbox_b.ymin))
    xb = int(min(bbox_a.xmax, bbox_b.xmax))
    yb = int(min(bbox_a.ymax, bbox_b.ymax))
    intersection_area = (max(0, xb-xa+1)*max(0, yb-ya+1))
    return intersection_area / bbox_a.area()

*/