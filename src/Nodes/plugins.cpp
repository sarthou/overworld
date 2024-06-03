#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include "overworld/BasicTypes/Area.h"
#include "overworld/BasicTypes/BodyPart.h"
#include "overworld/BasicTypes/Object.h"
#include "overworld/Perception/Modules/PerceptionModuleBase.h"
#include "overworld/Utility/ShellDisplay.h"

void displayUsage()
{
  std::cout << std::endl
            << "usage: rosrun overworld plugins COMMAND [ARGS]" << std::endl
            << std::endl;
  std::cout << "The overworld plugins commands are:" << std::endl;
  std::cout << "\tlist           List available plugins in your current workspace" << std::endl;
  std::cout << "\tdescription    Given a module name, provide its description" << std::endl;
  std::cout << "\tlibrary_path   Given a module name, provide the path to its associated library" << std::endl;
  std::cout << "\tpackage        Given a module name, provide the name of the containing package" << std::endl;
  std::cout << std::endl;
  std::cout << "See \'rosrun overworld plugins help COMMAND\' for more information on a specific command." << std::endl;
  std::cout << std::endl;
}

void help(const std::string& command)
{
  std::cout << std::endl
            << "NAME" << std::endl;
  if(command == "list")
    std::cout << "\tplugins-list" << std::endl;
  else if(command == "description")
    std::cout << "\tplugins-description" << std::endl;
  else if(command == "library_path")
    std::cout << "\tplugins-library_path" << std::endl;
  else if(command == "package")
    std::cout << "\tplugins-package" << std::endl;
  else
  {
    std::cout << "\tUNKNOWN COMMAND" << std::endl;
    return;
  }

  std::cout << std::endl
            << "SYNOPSIS" << std::endl;
  if(command == "list")
    std::cout << "\trosrun overworld plugins list [<option>]" << std::endl;
  else if(command == "description")
    std::cout << "\trosrun overworld plugins description <module>" << std::endl;
  else if(command == "library_path")
    std::cout << "\trosrun overworld plugins library_path <module>" << std::endl;
  else if(command == "package")
    std::cout << "\trosrun overworld plugins package <module>" << std::endl;

  std::cout << std::endl
            << "DESCRIPTION" << std::endl;
  if(command == "list")
    std::cout << "\tThis command lists the available perception modules in the form of plugins." << std::endl;
  else if(command == "description")
    std::cout << "\tGiven a module name, this command provides its description." << std::endl;
  else if(command == "library_path")
    std::cout << "\tGiven a module name, this command provides the path to its associated library." << std::endl;
  else if(command == "package")
    std::cout << "\tGiven a module name, this command provides the name of the containing package" << std::endl;

  std::cout << std::endl
            << "OPTIONS" << std::endl;
  if(command == "list")
    std::cout << "\t<option>" << std::endl
              << "\t\tEither Objects, BodyParts, or Areas to focus on specific module type." << std::endl;
  else
    std::cout << "\t<module>" << std::endl
              << "\t\tThe name of an existing perception module." << std::endl;

  std::cout << std::endl;
}

void list(bool objects, bool bodyparts, bool areas)
{
  std::cout << std::endl;
  if(bodyparts)
  {
    std::cout << "Available body parts perception modules are:" << std::endl;
    pluginlib::ClassLoader<owds::PerceptionModuleBase_<owds::BodyPart>> loader_bodyparts("overworld", "owds::PerceptionModuleBase_<owds::BodyPart>");
    std::vector<std::string> modules = loader_bodyparts.getDeclaredClasses();
    for(auto& module : modules)
    {
      std::cout << "\t- " << module << std::endl;
    }
    std::cout << std::endl;
  }

  if(objects)
  {
    std::cout << "Available objects perception modules are:" << std::endl;
    pluginlib::ClassLoader<owds::PerceptionModuleBase_<owds::Object>> loader_objects("overworld", "owds::PerceptionModuleBase_<owds::Object>");
    std::vector<std::string> modules = loader_objects.getDeclaredClasses();
    for(auto& module : modules)
    {
      std::cout << "\t- " << module << std::endl;
    }
    std::cout << std::endl;
  }

  if(areas)
  {
    std::cout << "Available areas perception modules are:" << std::endl;
    pluginlib::ClassLoader<owds::PerceptionModuleBase_<owds::Area>> loader_areas("overworld", "owds::PerceptionModuleBase_<owds::Area>");
    std::vector<std::string> modules = loader_areas.getDeclaredClasses();
    for(auto& module : modules)
    {
      std::cout << "\t- " << module << std::endl;
    }
    std::cout << std::endl;
  }
}

void description(const std::string& _class)
{
  std::string description;
  pluginlib::ClassLoader<owds::PerceptionModuleBase_<owds::BodyPart>> loader_bodyparts("overworld", "owds::PerceptionModuleBase_<owds::BodyPart>");
  description = loader_bodyparts.getClassDescription(_class);

  if(description == "")
  {
    pluginlib::ClassLoader<owds::PerceptionModuleBase_<owds::Object>> loader_objects("overworld", "owds::PerceptionModuleBase_<owds::Object>");
    description = loader_objects.getClassDescription(_class);

    if(description == "")
    {
      pluginlib::ClassLoader<owds::PerceptionModuleBase_<owds::Area>> loader_areas("overworld", "owds::PerceptionModuleBase_<owds::Area>");
      description = loader_areas.getClassDescription(_class);
    }
  }

  if(description == "")
    std::cout << std::endl
              << "Module " << _class << " does not exist. See command \'list\' to see available modules." << std::endl
              << std::endl;
  else
    std::cout << std::endl
              << "Module " << _class << ":" << std::endl
              << "\t" << description << std::endl
              << std::endl;
}

void libraryPath(const std::string& _class)
{
  std::string path;
  pluginlib::ClassLoader<owds::PerceptionModuleBase_<owds::BodyPart>> loader_bodyparts("overworld", "owds::PerceptionModuleBase_<owds::BodyPart>");
  path = loader_bodyparts.getClassLibraryPath(_class);

  if(path == "")
  {
    pluginlib::ClassLoader<owds::PerceptionModuleBase_<owds::Object>> loader_objects("overworld", "owds::PerceptionModuleBase_<owds::Object>");
    path = loader_objects.getClassLibraryPath(_class);

    if(path == "")
    {
      pluginlib::ClassLoader<owds::PerceptionModuleBase_<owds::Area>> loader_areas("overworld", "owds::PerceptionModuleBase_<owds::Area>");
      path = loader_areas.getClassLibraryPath(_class);
    }
  }

  if(path == "")
    std::cout << std::endl
              << "Module " << _class << " does not exist. See command \'list\' to see available modules." << std::endl
              << std::endl;
  else
    std::cout << std::endl
              << "Module " << _class << ":" << std::endl
              << "\t" << path << std::endl
              << std::endl;
}

void classPackage(const std::string& _class)
{
  std::string package;
  pluginlib::ClassLoader<owds::PerceptionModuleBase_<owds::BodyPart>> loader_bodyparts("overworld", "owds::PerceptionModuleBase_<owds::BodyPart>");
  package = loader_bodyparts.getClassPackage(_class);

  if(package == "")
  {
    pluginlib::ClassLoader<owds::PerceptionModuleBase_<owds::Object>> loader_objects("overworld", "owds::PerceptionModuleBase_<owds::Object>");
    package = loader_objects.getClassPackage(_class);

    if(package == "")
    {
      pluginlib::ClassLoader<owds::PerceptionModuleBase_<owds::Area>> loader_areas("overworld", "owds::PerceptionModuleBase_<owds::Area>");
      package = loader_areas.getClassPackage(_class);
    }
  }

  if(package == "")
    std::cout << std::endl
              << "Module " << _class << " does not exist. See command \'list\' to see available modules." << std::endl
              << std::endl;
  else
    std::cout << std::endl
              << "Module " << _class << ":" << std::endl
              << "\t" << package << std::endl
              << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plugins");

  if(argc == 1)
    displayUsage();
  else
  {
    std::string command = std::string(argv[1]);
    if(command == "list")
    {
      if(argc == 2)
        list(true, true, true);
      else if(argc > 2)
      {
        std::string option = std::string(argv[2]);
        list(option == "Objects", option == "BodyParts", option == "Areas");
      }
    }
    else if(command == "description")
    {
      if(argc < 3)
        std::cout << std::endl
                  << "No enough arguments. See \'rosrun overworld plugins help COMMAND\' for more information on a specific command." << std::endl
                  << std::endl;
      else
        description(std::string(argv[2]));
    }
    else if(command == "library_path")
    {
      if(argc < 3)
        std::cout << std::endl
                  << "No enough arguments. See \'rosrun overworld plugins help COMMAND\' for more information on a specific command." << std::endl
                  << std::endl;
      else
        libraryPath(std::string(argv[2]));
    }
    else if(command == "package")
    {
      if(argc < 3)
        std::cout << std::endl
                  << "No enough arguments. See \'rosrun overworld plugins help COMMAND\' for more information on a specific command." << std::endl
                  << std::endl;
      else
        classPackage(std::string(argv[2]));
    }
    else if(command == "help")
    {
      if(argc == 3)
        help(std::string(argv[2]));
      else
        displayUsage();
    }
  }

  return 0;
}