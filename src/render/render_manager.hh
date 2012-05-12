#ifndef RENDER_MANAGER_HPP
#define RENDER_MANAGER_HPP

#include "types.hh"

#include <vector>

class RenderObject;
class SimpleHeightMap;

class RenderManager{
public:
  RenderManager();

  void add_object(RenderObject* object);
  bool remove_object(RenderObject* object);

  void set_terrain(SimpleHeightMap* terrain_);

  void display_all_objects();
private:
  std::vector<RenderObject*> m_objects;
  SimpleHeightMap* terrain_;
};

#endif
