#include "overworld/BasicTypes/Object.h"

namespace owds{

Object::Object(): Entity(){

}
Object::Object(const std::string& id): Entity(id){

}

void Object::setPointsOfInterest(const std::vector<Pose>& pointsOfInterest){
    pointsOfInterest_ = pointsOfInterest;
}

void Object::addPointOfInterest(const Pose& pointOfInterest){
    pointsOfInterest_.push_back(pointOfInterest);
}

void Object::emptyPointsOfInterest(){
    pointsOfInterest_.empty();
}

const std::vector<Pose>& Object::getPointsOfInterest() const{
    return pointsOfInterest_;
}
}