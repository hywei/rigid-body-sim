#include "render_manager.hh"

#include "log_trace.hh"
#include "render_object.hh"

#include "glut_utils.hh"
#include "simple_heightmap.hh"

using namespace std;

// RenderManager
RenderManager::RenderManager(){
  TRACE_METHOD_ONLY(ONCE_1);
}


void RenderManager::add_object(RenderObject * object){
  TRACE_METHOD_ONLY(ONCE_2);
  m_objects.push_back(object);
}


bool RenderManager::remove_object(RenderObject * object){
  TRACE_METHOD_ONLY(ONCE_2);
  TRACE("Not implemented!\n");
  return true;
}


void RenderManager::set_terrain(SimpleHeightMap* terrain){
  TRACE_METHOD_ONLY(ONCE_1);
  if(!terrain_) delete terrain_;
  terrain_ = terrain;
}

void RenderManager::display_all_objects(){
  TRACE_METHOD_ONLY(FRAME_1);
  for (size_t i = 0 ; i <m_objects.size(); ++i){
    Save_GL_state state;
    m_objects[i]->display_object();
  }

  Save_GL_state state;
  terrain_->draw();
}
