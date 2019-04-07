/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class CargoArmManualCommand extends Command {

  public CargoArmManualCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.cargoArmSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // set how accurate the PID needs to be in absolute accuracy
    Robot.cargoArmSubsystem.setAbsoluteTolerance(10);
    Robot.cargoArmSubsystem.disable();
    RobotMap.MAGIC_POS = RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // squares the input from the xbox left joystick to make the cargo arm ovement slower and more precise, but keeps the sign the same as the original input
    Robot.cargoArmSubsystem.armManual((Math.copySign(Math.pow(RobotMap.GAME_PAD.getY(Hand.kLeft), 2), -RobotMap.GAME_PAD.getY(Hand.kLeft))) * 0.5);
    /*
    double targetSpeed = RobotMap.GAME_PAD.getY(Hand.kLeft) * 100;
    RobotMap.CARGO_ARM_MOTOR.set(ControlMode.MotionMagic, targetSpeed);
    SmartDashboard.putNumber("Magic Position: ", targetSpeed);
    /*
    
    RobotMap.MAGIC_POS += RobotMap.GAME_PAD.getY(Hand.kLeft) * 100;
    // set setpoint
    Robot.cargoArmSubsystem.setSetpoint(RobotMap.MAGIC_POS);
    SmartDashboard.putNumber("Magic Position: ", RobotMap.MAGIC_POS);
    */
    
    // Reset the encoder value to the right position when the according limit switch is pressed
    /*
    if (Robot.cargoArmSubsystem.getTopSwitch()) {
      Robot.cargoArmSubsystem.resetEncoderTop();
    }
    if (Robot.cargoArmSubsystem.getBottomSwitch()) {
      Robot.cargoArmSubsystem.resetEncoderBot();
    }
    */
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // If we aren't moving the arm, keep it where it is
    Robot.cargoArmSubsystem.setSetpoint(RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition());
    Robot.cargoArmSubsystem.enable(); // starts PID loop
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
