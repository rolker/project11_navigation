#ifndef PROJECT11_NAVIGATION_WORKFLOW_H
#define PROJECT11_NAVIGATION_WORKFLOW_H

#include <project11_navigation/context.h>

namespace project11_navigation
{

template <typename InType, typename OutType>
class Workflow
{
public:
  // Called once to initiate to workflow. Name may be used for parameter server lookups.
  virtual void configure(std::string name, Context::Ptr context) = 0;
  
  // Called when the initial goal is set or when it's changed.
  virtual void setGoal(const InType& input) = 0;


  // Returns true if working towards a goal.
  virtual bool running() = 0;

  // Get current result if available.
  // output: Updated with result
  // Returns true if output is available or false if output was not updated.
  virtual bool getResult(OutType& output) = 0;
};

}  // namespace project11_navigation

#endif
