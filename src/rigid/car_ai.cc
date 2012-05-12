#include "car_ai.hh"
#include "misc.hh"
#include "log_trace.hh"

using namespace std;

//==============================================================
// CarAI
//==============================================================
CarAI::CarAI(double range) : m_range(range), 
                               m_min_timer(10.0), 
                               m_max_timer(30.0)
{
  TRACE_METHOD_ONLY(ONCE_2);
}

//==============================================================
// add_controlled_car
//==============================================================
void CarAI::add_controlled_car(Car * car)
{
  m_car_records.push_back(Car_record(car,
                                     coord(ranged_random(-m_range, m_range),
                                              ranged_random(-m_range, m_range),
                                              ranged_random(-m_range, m_range)),
                                     ranged_random(m_min_timer, m_max_timer)));
}

//==============================================================
// remove_controlled_car
//==============================================================
bool CarAI::remove_controlled_car(const Car * car)
{
  vector<Car_record>::iterator it;
  for (it = m_car_records.begin() ; 
       it != m_car_records.end() ;
       ++it)
  {
    if (it->car == car)
    {
      m_car_records.erase(it);
      return true;
    }
  }
  return false;
}


//==============================================================
// choose_new_target
//==============================================================
void CarAI::choose_new_target(Car_record & record)
{
  record.target = coord(ranged_random(-m_range, m_range),
                           ranged_random(-m_range, m_range),
                           ranged_random(-m_range, m_range));
}


//==============================================================
// update_controls
//==============================================================
void CarAI::update(double dt)
{
  int index;
  
  for (index = 0 ; index < (int) m_car_records.size() ; ++index)
  {
    // update target according to time
    m_car_records[index].timer -= dt;
    if (m_car_records[index].timer < 0.0)
    {
      TRACE_FILE_IF(ONCE_2)
        TRACE("choosing new target\n");
      
      choose_new_target(m_car_records[index]);
      m_car_records[index].timer = ranged_random(m_min_timer, m_max_timer);
    }

    // check for crashing
    Car * car = m_car_records[index].car;
    car->set_activity_state(RigidBody::ACTIVE);
    if (car->get_orientation().get_col(2)[2] < 0.5)
    {
      m_car_records[index].crash_timer += dt;
    }
    else
    {
      m_car_records[index].crash_timer = 0.0;
    }
    
    if (m_car_records[index].crash_timer > 4.0)
    {
      TRACE("AI crash of car %p\n", car);
      // choose a new target in case the target was in a bad place
      choose_new_target(m_car_records[index]);
      
      car->set_orientation(matrix3_identity());
      car->set_position(car->get_position() + 
                        coord(ranged_random(-2.0, 2.0), 
                                 ranged_random(-2.0, 2.0),
                                 2.0));
    }
    
    

    // steering
    const coord & target = m_car_records[index].target;

    while ( (car->get_position() - target).mag() < 3.0)
    {
      TRACE_FILE_IF(ONCE_3)
        TRACE("choosing new target due to proximity\n");
      
      choose_new_target(m_car_records[index]);
    }
    
    vec3d cur_dir = car->get_orientation().get_col(0);
    cur_dir[2] = 0.0;
    cur_dir.normalise();

    double cur_heading = atan2_deg(cur_dir[1], cur_dir[0]);

    vec3d dir_to_target = target - car->get_position();
    double target_heading = atan2_deg(dir_to_target[1], dir_to_target[0]);

    double target_offset = target_heading - cur_heading;

    while (target_offset < -180.0)
      target_offset += 360.0;
    while (target_offset > 180.0)
      target_offset -= 360.0;
    
    double steer_frac = target_offset / 45.0;
    if (steer_frac < -1.0)
      steer_frac = -1.0;
    else if (steer_frac > 1.0)
      steer_frac = 1.0;
    
    // speed

    double ideal_speed = 20.0;
    
    double cur_speed = car->get_velocity().mag();
    
    double acc_frac = (ideal_speed - cur_speed) / 5.0;
    
    if (acc_frac < -1.0)
    {
      acc_frac = -1.0;
    }
    else if (acc_frac > 1.0)
    {
      acc_frac = 1.0;
    }
    
    // reverse turn if dest is directly behind
    double fwd_speed = dot(car->get_velocity(), cur_dir);
    if (m_car_records[index].allow_reverse_turn)
    {
      if ( ( (fwd_speed > 0.0) && (fabs(target_offset) > 179.0) ) ||
           ( (fwd_speed <= 0.0) && (fabs(target_offset) > 90.0) ) )
      {
        acc_frac = -1.0;
        m_car_records[index].reversing_timer += dt;
        m_car_records[index].not_reversing_timer = 0.0f;
        if (fwd_speed < -5.0)
          steer_frac *= -1.0;
        else
          steer_frac = 0.0;
      }
      else
      {
        m_car_records[index].not_reversing_timer += dt;
      }
    }
    else
    {
      m_car_records[index].not_reversing_timer += dt;
      m_car_records[index].reversing_timer = 0.0f;
    }
    
    if (m_car_records[index].not_reversing_timer > 3.0f)
      m_car_records[index].allow_reverse_turn = true;
    else if (m_car_records[index].reversing_timer > 10.0f)
      m_car_records[index].allow_reverse_turn = false;
    

    if (car->get_num_wheels_on_floor() == 2)
      acc_frac = -1.0;

    // apply steering
    car->set_steer(steer_frac);
    
    // apply acceleration
    car->set_accelerate(acc_frac);
  }
}

