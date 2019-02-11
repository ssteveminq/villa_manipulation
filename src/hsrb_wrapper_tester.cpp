#include <ros/ros.h>
#include <villa_manipulation/manipulator.h>

using namespace std;
int main(int argc, char**argv) {
  ros::init(argc, argv, "hsrb_wrapper_tester");


  // Note that the gripper controller will cause problems in simulation!
  // Pass true to make the wrapper behave in the simulator
  villa_manipulation::Manipulator hsrb(true);
  hsrb.move_to_neutral(true);
  {
    bool success = true;
    success &= hsrb.turn_base(M_PI / 2, true);
    success &= hsrb.turn_base(-M_PI / 2, false);
    assert (success);
  }
  {
    hsrb.open_gripper();
    assert(!hsrb.gripper_has_object());
    cout << "Place something in the robot's gripper. Press enter to commence grip countdown" << endl;
    cin.ignore(std::numeric_limits<streamsize>::max(), '\n');
    sleep(3);

    hsrb.close_gripper();
    // This is unlikely to work in simulation
    assert (hsrb.gripper_has_object());
  }

  {
    hsrb.move_to_go(false);
    hsrb.move_to_neutral(false);
  }

  {
    hsrb.move_head(-1.1, 0.0);
    hsrb.move_head(1.1, 0.0);
    hsrb.move_head(0.0, 0.3);
    hsrb.move_head(0.0, -0.3);
    hsrb.move_head(0.0, 0.0);
  }

  {
    bool success = hsrb.move_to_joint_positions({{"head_tilt_joint", 1.0}}, true);
    assert(!success);
    hsrb.move_to_neutral(false);
  }

  {
    bool success = hsrb.move_to_joint_positions({{"arm_lift_joint",   0.30},
                                            {"arm_flex_joint",   -0.42},
                                            {"arm_roll_joint",   -1.55},
                                            {"wrist_flex_joint", -1.57},
                                            {"wrist_roll_joint", 2.05},
                                            {"head_pan_joint",   -1.5},
                                            {
                                             "head_tilt_joint",  -0.40}}, false);
    assert(success);
    hsrb.move_base_relative(0, 0, -1.57);
    // All joints should move at the same time
    hsrb.move_to_neutral(false);
    hsrb.move_base_relative(0, 0, 1.57);
  }


}