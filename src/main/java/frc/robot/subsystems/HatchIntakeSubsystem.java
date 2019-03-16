/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 * @author Max, Krishna
 */
public class HatchIntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // Moving velcro pistons forward
  public void hatchExtendPistons() {
    RobotMap.HATCH_TOGGLE = !RobotMap.HATCH_TOGGLE;
    RobotMap.HATCH_EXTEND_SOLENOID.set(RobotMap.HATCH_TOGGLE);
  }

  // Moving velcro pistons forward for controller
  public void hatchExtendXboxPistons() {
    RobotMap.HATCH_EXTEND_SOLENOID.set(RobotMap.HATCH_TOGGLE);
  }

  // Moving non-velcro pistons forward
  public void hatchGrabberPistons(boolean value) {
    RobotMap.HATCH_GRABBER_SOLENOID.set(value);
  }

  // Moving the hatch intake in and out from the robot
  public void hatchPosition(boolean value) {
    // RobotMap.HATCH_POSITION_SOLENOID.set(value); // Not using right now
  }

  // Check if there is Super Suit (hatch) in front of the robot with limit switch
  public void getHatch() {
    boolean hatchDetected = RobotMap.LEFT_CLIMBER_MOTOR.getSensorCollection().isFwdLimitSwitchClosed();

    // Display on SmartDashboard on whether Super Suit (hatch) is there or not
    SmartDashboard.putBoolean("Hatch Present:", !hatchDetected);
  }
}
