#ifndef PROJECT11_NAVIGATION_PLUGIN_LOADER_H
#define PROJECT11_NAVIGATION_PLUGIN_LOADER_H

#include "project11_navigation/interfaces/task_to_task_workflow.h"
#include "project11_navigation/interfaces/task_to_twist_workflow.h"
#include "project11_navigation/interfaces/tasklist_to_tasklist_workflow.h"
#include "project11_navigation/interfaces/tasklist_to_twist_workflow.h"
//#include "project11_navigation/interfaces/task_wrapper.h"
#include <pluginlib/class_loader.h>


namespace project11_navigation
{

class PluginsLoader
{
public:
  PluginsLoader();

  template<typename T> boost::shared_ptr<T> getPlugin(std::string name)
  {
    std::shared_ptr<PluginType<T> > pt = std::dynamic_pointer_cast<PluginType<T> >(plugins_[typeid(T)]);
    if(pt->plugins.find(name) != pt->plugins.end())
      return pt->plugins[name];
    return boost::shared_ptr<T>();
  }

  void configure(std::shared_ptr<Context> context);

private:
  template<typename T> void addType(std::string base_class)
  {
    plugins_[typeid(T)] = std::make_shared<PluginType<T> >(base_class);
  }

  template<typename T> void loadPlugins(const XmlRpc::XmlRpcValue& value)
  {
    std::shared_ptr<PluginType<T> > pt = std::dynamic_pointer_cast<PluginType<T> >(plugins_[typeid(T)]);
    if(value.hasMember(pt->base_class))
    {
      auto t = value[pt->base_class];
      for(auto plugin_param: t)
      {
        if(plugin_param.second.getType() == XmlRpc::XmlRpcValue::TypeString)
        {
          std::string plugin_type = plugin_param.second;
          if(pt->loader.isClassAvailable(plugin_type))
            pt->load(plugin_param.first, plugin_type);
          else
            ROS_WARN_STREAM("Could not load " << plugin_param.first << " plugin of type " << plugin_type);
        }
        else
          ROS_WARN_STREAM("value for " << plugin_param.first << " is not a string type");
      }
    }
  }

  template<typename T> void configureType(std::shared_ptr<Context> context)
  {
    std::shared_ptr<PluginType<T> > pt = std::dynamic_pointer_cast<PluginType<T> >(plugins_[typeid(T)]); 
    for(auto p: pt->plugins)
      p.second->configure(pt->base_class+"/"+p.first, context);
  }


  struct PluginTypeBase
  {
    PluginTypeBase() {};
    virtual ~PluginTypeBase() {};
  };

  template<typename T> struct PluginType: public PluginTypeBase
  {
    PluginType(std::string base_class): loader("project11_navigation", "project11_navigation::"+base_class),base_class(base_class){}
    pluginlib::ClassLoader<T> loader;
    std::string base_class;
    std::map<std::string, boost::shared_ptr<T> > plugins;
    void load(std::string name, std::string type)
    {
      ROS_INFO_STREAM("Loading " << name << " plugin of type " << type << "...");
      try
      {
        plugins[name] =  loader.createInstance(type);
      }
      catch(const std::exception& e)
      {
        ROS_FATAL("Failed to load the %s plugin of type %s, are you sure it is properly registered and that the containing library is built? Exception: %s", name.c_str(), type.c_str(), e.what());
        exit(1);
      }
    }
  };

  std::map<std::type_index, std::shared_ptr<PluginTypeBase> > plugins_;
};

} // namespace project11_navigation

#endif
