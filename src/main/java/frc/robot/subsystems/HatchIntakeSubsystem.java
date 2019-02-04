/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
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
  public void hatchIntakePistons(boolean value) {
    RobotMap.HATCH_INTAKE_SOLENOID.set(value);
  }

  // Moving non-velcro pistons forward
  public void hatchOuttakePistons(boolean value) {
    RobotMap.HATCH_OUTTAKE_SOLENOID.set(value);
  }

  // Moving the hatch intake in and out from the robot
  public void hatchPosition(boolean value) {
    RobotMap.HATCH_POSITION_SOLENOID.set(value);
  }
}
