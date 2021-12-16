#include <PBC.h>
#include <cstdio>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "PBC_Walk");
  ros::NodeHandle nh;

  PBC pbc(nh, 25);

  // pbc.SimReset();

  // pbc.Reset();
  // ros::Rate(0.3).sleep();

  // pbc.moveCOM(-0.05);
  // pbc.moveCOM(0.0);
  // pbc.moveCOM(0.05);
  // pbc.moveCOM(0.0);

  pbc.firstStep();

  pbc.step(false);

  pbc.step(true);

  pbc.lastStep(false);

  // pbc.Reset();
  pbc.waitPubQueEmpty();

  return 0;
}
