#ifndef OWDS_ONTOLOGY_H
#define OWDS_ONTOLOGY_H

#include <array>
#include <string>

#include "ontologenius/OntologiesManipulator.h"
#include "overworld/BasicTypes/Shape.h"

namespace owds {

  namespace ontology {

    std::array<double, 4> getEntityColor(onto::OntologyManipulator* onto, const std::string& indiv_name, const std::array<double, 4>& default_value = {0.8, 0.8, 0.8, 0.});

    Shape_t getEntityShape(onto::OntologyManipulator* onto, const std::string& indiv_name);

    double getEntityMass(onto::OntologyManipulator* onto, const std::string& indiv_name);

    void addColor(onto::OntologyManipulator* onto, const std::string& color_name, const std::string& rgb_value = "");

    void addColorToEntity(onto::OntologyManipulator* onto, const std::string& indiv_name, const std::string& color_name);

  } // namespace ontology

} // namespace owds

#endif // OWDS_ONTOLOGY_H