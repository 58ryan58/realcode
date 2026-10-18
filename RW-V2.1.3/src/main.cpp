#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "pid.h"
#include "motor-control.h"
#include "robot-config.h"
#include "utils.h"
#include "vex.h"
#include "autotune.h"
 
using namespace vex;
 
extern bool   tuner_active;
extern bool   tuner_finished;
extern bool   tuner_done_early;
extern double tuner_kp;
extern double tuner_kd;
extern double tuner_cost;
extern char   tuner_label[16];
 
// ============================================================================
// V5 MACROS
// ============================================================================
#define waitUntil(condition)                                                  \
  do {                                                                        \
    wait(5, msec);                                                            \
  } while (!(condition))
 
#define repeat(iterations)                                                    \
  for (int iterator = 0; iterator < iterations; iterator++)
 
// ============================================================================
// UTILITY
// ============================================================================
 
// Seed rand() using battery/timer noise so autons using random behavior
// (if any) don't repeat the same sequence every run.
void initializeRandomSeed() {
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;
  srand(seed);
}
 
// ============================================================================
// DISPLAY / STATUS TASK
// Runs continuously in the background, printing basic telemetry to the
// controller screen. Add your own subsystem status lines here as needed.
// ============================================================================
 
void drawWatermark() {
  Brain.Screen.setFont(monoXXL);
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(330, 230, "RW V2"); // bottom-right corner (480x272 screen)
  Brain.Screen.setFont(mono15);            // reset font for other prints
}
 
bool last_tuner_active = false;
 
void update() {
  double heading = normalizeTarget(getInertialHeading()); 
 
  bool showingTuner = tuner_active || tuner_finished;
 
  bool modeChanged = (showingTuner != last_tuner_active);
  if (modeChanged) {
    controller_1.Screen.clearScreen();
    last_tuner_active = showingTuner;
  }
 
  if (showingTuner) {
    controller_1.Screen.setCursor(1, 1);
    controller_1.Screen.print(tuner_done_early ? "GOOD VALUES!   " : (tuner_finished ? "TUNE DONE      " : "TUNING...      "));
    controller_1.Screen.setCursor(2, 1);
    controller_1.Screen.print("Kp:%.3f Kd:%.3f  ", tuner_kp, tuner_kd);
  } else {
    controller_1.Screen.setCursor(1, 1);
    controller_1.Screen.print("Hdg: %.1f      ", inertial_sensor.heading());
 
    controller_1.Screen.setCursor(2, 1);
    controller_1.Screen.print("X:%.1f Y:%.1f    ", x_pos, y_pos);
 
    controller_1.Screen.setCursor(3, 1);
    controller_1.Screen.print("Batt: %d%%    ", Brain.Battery.capacity());
  }
 
  Brain.Screen.clearScreen();
 
  if (showingTuner) {
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(tuner_done_early ? "GOOD VALUES FOUND" : (tuner_finished ? "TUNING COMPLETE" : "TUNING IN PROGRESS"));
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("%s  cost: %.3f", tuner_label, tuner_cost);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Kp: %.4f", tuner_kp);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Kd: %.4f", tuner_kd);
  } else {
    Brain.Screen.print("ODOM  x:%.1f y:%.1f h:%.1f\n", x_pos, y_pos, heading);
  }
 
  drawWatermark();
}
 
int updateTask() {
  while (true) {
    update();
    this_thread::sleep_for(100);
  }
  return 0;
}
 
// Optional: prints odometry position to the terminal for debugging.
// REQURIES CONTROLLER TO BE PLUGGED IN.
void logPositionHandler() {
  printf("X: %.2f Y: %.2f Heading: %.2f\n", x_pos, y_pos, inertial_sensor.heading());
}
 
int logPosition() {
  printf("\nBEGIN RUN\n");
  while (true) {
    logPositionHandler();
    this_thread::sleep_for(100);
  }
  return 0;
}
 
// Starts the correct odometry-tracking thread based on which tracking
// wheels are enabled in robot-config.cpp.
void trackPosition() {
  resetChassis();
  trackOdom();
}
 
// ============================================================================
// SUBSYSTEM PID TASKS
// ----------------------------------------------------------------------------
// Add one block like this per mechanism (arm, lift, wrist, etc). Pattern:
//
//double arm_pid_target = 0;
//PID pidArm(0.5, 0.0, 2.0);
 
void armPID(double target) {
 // pidArm.setTarget(target);
  //pidArm.setIntegralMax(0);
 // pidArm.setIntegralRange(1);
 // pidArm.setSmallBigErrorTolerance(1, 1);
 // pidArm.setSmallBigErrorDuration(0, 0);
 // pidArm.setDerivativeTolerance(100);
 // pidArm.setArrive(true);
  //arm.spin(fwd, pidArm.update(arm.position(deg)), volt);
  // or read a rotation sensor instead: arm_rotation.position(deg)
}
 
int armPIDLoop() {
  while (true) {
  //  armPID(arm_pid_target);
    wait(10, msec);
  }
  return 0;
}
//
// Then start it as a task in telop()/auton() with: task myTask(mySubsystemPIDLoop);
// ============================================================================
 
// ============================================================================
// DRIVER CONTROL
// ============================================================================
// claw control (R1):
//  - claw starts open (turned to a set angle, so the motor doesn't strain)
//  - press R1 to start closing: the motor spins for as long as R1 is held
//  - let go and it stops; the ratchet keeps the grip
//  - press R1 again to open back to the set angle
const double CLAW_OPEN_DEG   = 35;  // how far the motor turns to open
const int    CLAW_OPEN_SIGN  = 1;   // 1 or -1: flip if "open" turns the wrong way
const int    CLAW_CLOSE_SIGN = -1;  // 1 or -1: direction the motor spins to close
 
bool clawIsOpen = false;
bool lastR1 = false;
 
void openClaw() {
  intake.setPosition(0, degrees);   // closed position is always 0
  intake.spinToPosition(CLAW_OPEN_DEG * CLAW_OPEN_SIGN, degrees, 100, velocityUnits::pct, false);
  clawIsOpen = true;
}
 
void driverClaw() {
  bool r1 = controller_1.ButtonR1.pressing();
 
  if (r1 && !lastR1) {              // fresh press only
    if (clawIsOpen) {
      clawIsOpen = false;           // start closing
    } else {
      openClaw();                   // closed -> open
    }
  }
  lastR1 = r1;
 
  if (!clawIsOpen) {
    if (r1) {
      intake.spin(CLAW_CLOSE_SIGN > 0 ? forward : reverse, 12, volt);
    } else {
      intake.stop(coast);           // the ratchet holds the grip
    }
  }
}
 
void telop() {
  vex::task updater(updateTask);
  vex::task logposition(logPosition);
  update();
  inertial_sensor.setRotation(0, degrees);
  left_chassis.setStopping(coast);
  right_chassis.setStopping(coast);
 
  double l1_press_start = -1;
  bool arm_task_started = false;
 
       intake.setPosition(0, degrees);
     intake.setStopping(coast);
 
  openClaw();   // claw starts open for driver control
  lastR1 = controller_1.ButtonR1.pressing();
 
  while (true) {
    int left_pct  = 0;
    int right_pct = 0;
 
    switch (drive_scheme) {
      case TANK:
        left_pct  = controller_1.Axis3.position();
        right_pct = controller_1.Axis2.position();
        break;
 
      case ARCADE: {
        int fwd  = controller_1.Axis3.position();
        int turn = controller_1.Axis4.position();
        left_pct  = fwd + turn;
        right_pct = fwd - turn;
        break;
      }
 
      case SPLIT_ARCADE:
      default: {
        int fwd  = controller_1.Axis3.position();
        int turn = controller_1.Axis1.position();
        left_pct  = fwd + turn;
        right_pct = fwd - turn;
        break;
      }
    }
    if (left_pct  >  100) left_pct  =  100;
    if (left_pct  < -100) left_pct  = -100;
    if (right_pct >  100) right_pct =  100;
    if (right_pct < -100) right_pct = -100;
    left_chassis.setVelocity(left_pct,  percent);
    right_chassis.setVelocity(right_pct, percent);
    left_chassis.spin(forward);
    right_chassis.spin(forward);
 
    // Claw
    driverClaw();
 
    // Lift Control
if (controller_1.ButtonL1.pressing()) {
  liftgroup.spin(forward, 12, volt);
} else if (controller_1.ButtonL2.pressing()) {
if (liftgroup.position(degrees) > 35) {   // high up: coast
    liftgroup.stop(coast);
  } else {                                   // low: motors drive it down
    liftgroup.spin(reverse, 12, volt);
  }
} else {
  liftgroup.stop(hold);
}
 
// Claw lift control
if (controller_1.ButtonA.pressing()) {
  // Upward motion
  rotate.spin(forward, 12, volt);
} else if (controller_1.ButtonR2.pressing()) {
  // Downward motion
  rotate.spin(reverse, 12, volt);
} else {
  rotate.stop(coast);
}
 
  if (controller_1.ButtonUp.pressing()) {
  claw.set(true);  // Closes or opens the claw
  } else  if (controller_1.ButtonRight.pressing()) {
  claw.set(false); // Does the opposite action
}
 
 
   // controller_1.ButtonY.pressed(runDistanceAutoTune);
  }
}
// ============================================================================
// AUTONOMOUS
// ============================================================================
void auton() {
  intake.spin(forward, 12, volt);
  wait(1, seconds);
  intake.stop(coast);
 
  trackPosition();
  vex::task updater(updateTask);
  vex::task logposition(logPosition);
  update();
  inertial_sensor.setRotation(0, degrees);
  correct_angle = normalizeTarget(0);
 
  // Start subsystem PID task(s) and reset subsystem encoders here, e.g.:
  // task armTask(armPIDLoop);
 
  // Write your autonomous routine here.
}
 
int main() {
  competition Competition = competition();
  Competition.autonomous(auton);
  Competition.drivercontrol(telop); 
  return 0;
}
 