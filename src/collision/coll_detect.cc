#include "coll_detect.hh"
#include "rigid_body.hh"
#include "sim_heightmap.hh"
#include "log_trace.hh"

using namespace std;

static void detect_heightmap_collision(
    CollBody* body, SimHeightMap* hmap, vector<CollInfo>& colls){
  
  double height;
  vec3d norm;
  
  // basic check
  coord center;
  double radius;
  body->getBoundingSphere(center, radius);
  hmap->get_z_and_normal(center[0], center[1], height, norm);
  // hmmm that normal[2] is a bit of a fudge to try to catch steeply
  // sloping issues
  if (center[2]-height> radius/norm[2]) return;

  // points detect
  const vector<coord>& points = body->getWorldPoints();
  for(size_t i=0; i<points.size(); ++i) {
    coord p = points[i];
    hmap->get_z_and_normal(p[0], p[1], height, norm);
    if(p[2] < height) {
      double depth = (height - p[2]) * norm[2];
      colls.push_back(CollInfo(body, 0, hmap, norm, p, depth));
      body->getCollisions().push_back(colls.size()-1);
    }
  }
  // edge detect
  const vector<CollEdge>& edges = body->getCollEdges();
  for(size_t i=0; i<edges.size(); ++i){
    const CollEdge& e = edges[i];
    coord ps = body->getWorldEdgePoints()[e.i0];
    coord pe = body->getWorldEdgePoints()[e.i1];
    vec3d ed = pe - ps; // edge dir
    double max_depth = -1000000.0;
    coord deepest_pos, deepest_norm;
    for(size_t j=0; j<e.num_samples; ++j){
      coord p = ps + (j*ed)/e.num_samples;
      hmap->get_z_and_normal(p[0], p[1], height, norm);
      double depth = (height - p[2]) * norm[2];
      if(depth > max_depth) {
        deepest_pos = p; 
        deepest_norm = norm;
        max_depth = depth;
      }
    }
    if(max_depth > 0.0){ //!FIXME: should be height not 0.0
      colls.push_back(CollInfo(body, 0, hmap, deepest_norm, deepest_pos, max_depth));
      body->getCollisions().push_back(colls.size() -1);
    }
  }
}

// detect_body_collision
static void detect_body_collision(
    CollBody* body0, CollBody* body1, vector<CollInfo>& colls) {
  // transfrom body1 into body0 frame, and detect by checking the
  // points of body 1 against the mesh of body 0  
  coord c0;
  double r0;
  body0->getBoundingSphere(c0, r0);  
  
  // returned in the collision check
  vec3d dir;
  double dist;

  // points detect
  const vector<coord>& points1 = body1->getWorldPoints();
  for(size_t i=0; i<points1.size(); ++i) {
    coord p1 =  points1[i];
    if((p1 - c0).mag2() < r0*r0) {
      //  transfrom p1 to body0's frame
      coord trans_p1 = body0->getInvOrientation()*(p1 - body0->getPosition());
      if(body0->getPointInfo(trans_p1, dir, dist)){
        // note that dir is the normal direction on body 0. Collision
        // handling wants the opposing normal.
        
        // Got intersection - return values were in body0 reference
        // frame, so need to put into world coords
        vec3d norm = body0->getOrientation()*dir;
        coord mid_point = p1 + norm * (0.5 * dist);
        colls.push_back(CollInfo(body0, body1, 0, -norm, mid_point, dist));
        body0->getCollisions().push_back(colls.size()-1);
        body1->getCollisions().push_back(colls.size()-1);
      }
    }
  }    

  // edge detect
  const vector<CollEdge>& edges = body1->getCollEdges();
  for(size_t i=0; i<edges.size(); ++i) {
    const CollEdge& e = edges[i];
    coord ps = body1->getWorldEdgePoints()[e.i0];
    coord pe = body1->getWorldEdgePoints()[e.i1];
    vec3d ed = pe - ps; // edge dir
    for(size_t j=0; j< e.num_samples; ++j) {
      coord p1 = ps + ((j*1.0)/e.num_samples)*ed;
      if((p1 - c0).mag2() < r0*r0) {
        coord trans_p1 = body0->getInvOrientation()*(p1 - body0->getPosition());
        if(body0->getPointInfo(trans_p1, dir, dist)) {
          vec3d norm = body0->getOrientation()*dir;
          coord mid_point = p1 + norm*(0.5*dist);
          colls.push_back(CollInfo(body0, body1, 0, -norm, mid_point, dist));
          body0->getCollisions().push_back(colls.size()-1);
          body1->getCollisions().push_back(colls.size()-1);
        }
      }
    }
  }                    
}

inline bool is_collidable(CollBody* body, vector<CollBody*>& nc){
  if (nc.empty()) return true;
  for (size_t i = 0 ; i < nc.size() ; ++i) {
    if (nc[i] == body) return false;
  }
  return true;
}

inline bool is_collidable(RigidBody* body) {
  if(body==0) return false;
  return (!body->get_immovable()) || body->get_activity_state() == RigidBody::ACTIVE;
}

void detect_all_collisions(const vector<CollBody*>& bodies,
                           SimHeightMap* hmap, vector<CollInfo>& colls){
  for(size_t i=0; i<bodies.size(); ++i) {
    RigidBody* rb = bodies[i]->rigid_body();
    if(!rb->get_immovable() && rb->get_activity_state() == RigidBody::ACTIVE){
      bodies[i]->calculateWorldProperties();
      detect_heightmap_collision(bodies[i],  hmap, colls);
    }
  }
  
  coord c0, c1;
  double r0, r1;
  for(size_t i=0; i<bodies.size(); ++i) {
    RigidBody* rb1 = bodies[i]->rigid_body();
    for(size_t j=0; j<i; ++j) {
      RigidBody* rb2 = bodies[j]->rigid_body();
      if(!is_collidable(rb1) && !is_collidable(rb2)) return;
      // if(!rb1->get_immovable() || !rb2->get_immovable()) {
      //   if(rb1->get_activity_state() == RigidBody::ACTIVE ||
      //      rb2->get_activity_state() == RigidBody::ACTIVE) {
      bodies[i]->getBoundingSphere(c0, r0);
      bodies[j]->getBoundingSphere(c1, r1);
      if((c1-c0).mag2() < ((r0+r1)*(r0+r1))) {
        // translate pos i to body j's frame
        coord p_ij = bodies[j]->getInvOrientation()*(c0 - bodies[j]->getPosition());
        coord p_ji = bodies[i]->getInvOrientation()*(c1 - bodies[i]->getPosition());
        // get distance of i to j' bounding box 
        double dist_ij = bodies[j]->getSqrDist2BoundingBox(p_ij);
        double dist_ji = bodies[i]->getSqrDist2BoundingBox(p_ji);
        if(dist_ij < r0*r0 && dist_ji < r1*r1) {                       
          detect_body_collision(bodies[i], bodies[j], colls);
          detect_body_collision(bodies[j], bodies[i], colls);
        }
      }
    }
  }
}

// detect_collisions_with_body
void detect_frozen_collisions_with_body(CollBody * collision_body,
                                        const std::vector<CollBody *> & collision_bodies,
                                        const std::vector<SimHeightMap *>  & heightmaps,
                                        std::vector<CollInfo> & collisions){
  if (collision_body->rigid_body()->get_immovable())
    return;

  int num_bodies = collision_bodies.size();
  int num_heightmaps = heightmaps.size(); 
  
  collision_body->calculateWorldProperties();

  // heightmaps first
  for (int j = 0 ; j < num_heightmaps ; ++j){
    detect_heightmap_collision(collision_body,
                               heightmaps[j], collisions);
  }
  
  // body-body Note the world properties are already calculated
  coord pos0, pos1;
  double r0, r1;

  collision_body->getBoundingSphere(pos0, r0);

  for(int j = 0 ; j < num_bodies ; ++j){
    if (collision_bodies[j]->rigid_body() != collision_body->rigid_body()){
      if ( (collision_bodies[j]->rigid_body()->get_activity_state() == RigidBody::FROZEN ) ||
           (!collision_bodies[j]->rigid_body()->get_immovable()) ){
        // both bodies should be in each others lists... so just check one
        collision_bodies[j]->getBoundingSphere(pos1, r1);
        if ((pos1 - pos0).mag2() < ((r0 + r1) * (r0 + r1))){
          // only continue if the bounding sphere of one intersects 
          // the bounding box of the other
          const coord posi_in_bodyj_frame = 
              collision_bodies[j]->getInvOrientation() * 
              (pos0 - collision_bodies[j]->getPosition());
        
          double sqr_dist_bodyi_to_bodyj_box = 
              collision_bodies[j]->getSqrDist2BoundingBox(
                  posi_in_bodyj_frame);
        
          if (sqr_dist_bodyi_to_bodyj_box < (r0 * r0)){
            const coord posj_in_bodyi_frame = 
                collision_body->getInvOrientation() * 
                (pos1 - collision_body->getPosition());
          
            double sqr_dist_bodyj_to_bodyi_box = 
                collision_body->getSqrDist2BoundingBox(
                    posj_in_bodyi_frame);
          
            if (sqr_dist_bodyj_to_bodyi_box < (r1 * r1)){
              detect_body_collision(collision_body, 
                                    collision_bodies[j], collisions);
              detect_body_collision(collision_bodies[j], 
                                    collision_body, collisions);
            }
          }
        }
      }
    }
    
  }
}
