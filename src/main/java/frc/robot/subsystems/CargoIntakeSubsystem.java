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
 * @author Lucas
 */
public class CargoIntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  
  public CargoIntakeSubsystem(){
  }

  @Override
  public void initDefaultCommand() {
  }

  public void intakeCargo(){
    // move both motors to intake cargo
    RobotMap.CARGO_INTAKE_MOTOR_GROUP.set(1);
  }

  public void outtakeCargoFast(){
    // move both motors to release cargo
    RobotMap.CARGO_INTAKE_MOTOR_GROUP.set(-1);
  }

  public void outtakeCargoSlow(){
    // move both motors to release cargo slow
    RobotMap.CARGO_INTAKE_MOTOR_GROUP.set(-0.5);
  }

  public void stop(){
    // stop all cargo motors
    RobotMap.CARGO_INTAKE_MOTOR_GROUP.set(0);
  }
}
