#ifndef PHYSICS_CONTROLLER_HPP
#define PHYSICS_CONTROLLER_HPP

#include "types.hh"

class PhysicsController
{
public:
  virtual ~PhysicsController() {}

  /// implement this to apply whatever forces are needed to the objects this controls
  virtual void update(double dt) = 0;
};

#endif
