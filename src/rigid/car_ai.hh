#include "../common/types.hh"
#include "car.hh"
#include "../simulation/physics_controller.hh"

#include <vector>

class Car;

// deals with all the cars - very simple - just makes them move around
// a bit.
class CarAI : public PhysicsController{
public:
  // range indicates how far away from the origin the targets are
  CarAI(double range);
  void add_controlled_car(Car * car);

  // rv indicates success
  bool remove_controlled_car(const Car * car);
  void update(double dt);
  
private:
  struct Car_record{
    Car_record(Car * car, const coord & target, double timer) :
      car(car), target(target), timer(timer), not_reversing_timer(0),
      reversing_timer(0), allow_reverse_turn(false) {}
    
    Car* car;
    coord target;
    double timer; // when < zero choose new target
    double crash_timer; // increments when not OK

    // how long we've not been reversing for
    double not_reversing_timer;
    double reversing_timer;
    bool allow_reverse_turn;    
  };
  
  void choose_new_target(Car_record & record);
  
  double m_range;
  double m_min_timer, m_max_timer;
  
  std::vector<Car_record> m_car_records;
};

