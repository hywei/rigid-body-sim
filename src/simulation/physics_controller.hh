#ifndef PHYSICS_CONTROLLER_HPP
#define PHYSICS_CONTROLLER_HPP

#include "types.hh"

class Physics_controller
{
public:
  virtual ~Physics_controller() {}

  /// implement this to apply whatever forces are needed to the objects this controls
  virtual void update(double dt) = 0;
};

#endif
