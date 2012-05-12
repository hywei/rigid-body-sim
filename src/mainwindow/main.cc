#include "simulator.hh"
using namespace std;

int main(int argc, char * argv[]){
  Simulator simulator(argc, argv);
  simulator.initialise("Simulator");
  simulator.start_main_loop();
  return 0;
}
