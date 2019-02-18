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
    RobotMap.CARGO_BOTTOM_ROLLER_MOTOR.set(1);
    RobotMap.CARGO_TOP_ROLLER_MOTOR.set(1);
  }

  public void outtakeCargoFast(){
    RobotMap.CARGO_BOTTOM_ROLLER_MOTOR.set(-1);
    RobotMap.CARGO_TOP_ROLLER_MOTOR.set(-1);
  }

  public void outtakeCargoSlow(){
    RobotMap.CARGO_BOTTOM_ROLLER_MOTOR.set(-0.5);
    RobotMap.CARGO_TOP_ROLLER_MOTOR.set(-0.5);
  }


  public void stop(){
    RobotMap.CARGO_BOTTOM_ROLLER_MOTOR.set(0);
    RobotMap.CARGO_TOP_ROLLER_MOTOR.set(0);
  }
}
