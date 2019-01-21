/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * @author Karthik
 */

public class ClimberSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //This method makes the motors on both sides make the robot ascend
  public void ascend () {
    RobotMap.LEFT_CLIMBER_MOTOR.set(1);
    RobotMap.RIGHT_CLIMBER_MOTOR.set(1);
  }

  //This method makes the motors on both sides make the robot descend
  public void descend () {
    RobotMap.LEFT_CLIMBER_MOTOR.set(-1);
     RobotMap.RIGHT_CLIMBER_MOTOR.set(-1);
  }

  //This method makes the motors on both sides stop.
  public void stop () {
    RobotMap.LEFT_CLIMBER_MOTOR.set(0);
     RobotMap.RIGHT_CLIMBER_MOTOR.set(0);
  }
}