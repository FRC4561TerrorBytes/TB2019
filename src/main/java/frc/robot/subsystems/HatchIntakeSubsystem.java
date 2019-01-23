/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
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
  public void hatchIntakePistonsForward() {
    RobotMap.HATCH_INTAKE_SOLENOID.set(true);
  }

  // Moving velcro pistons backwards
  public void hatchIntakePistonsReverse() {
    RobotMap.HATCH_INTAKE_SOLENOID.set(false);
  }

  // Moving non-velcro pistons forward
  public void hatchOuttakePistonsForward() {
    RobotMap.HATCH_OUTTAKE_SOLENOID.set(true);
  }

  // Moving non-velcro pistons backwards
  public void hatchOuttakePistonsReverse() {
    RobotMap.HATCH_OUTTAKE_SOLENOID.set(false);
  }

  // Allows the intake to move vertically.
  public void hatchVertical() {
    RobotMap.HATCH_POSITION_SOLENOID.set(DoubleSolenoid.Value.kReverse);
  }
// Allows the intake to move horizontally.
  public void hatchHorizontal() {
    RobotMap.HATCH_POSITION_SOLENOID.set(DoubleSolenoid.Value.kForward);
  }

}
