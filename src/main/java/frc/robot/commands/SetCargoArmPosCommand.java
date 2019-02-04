/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SetCargoArmPosCommand extends Command {
  int setpoint;
  public SetCargoArmPosCommand(int setpoint) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.cargoArmSubsystem);
    this.setpoint = setpoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // set PID tolerance in encoder ticks
    Robot.cargoArmSubsystem.setAbsoluteTolerance(5);
    // set setpoint
    Robot.cargoArmSubsystem.setSetpoint(this.setpoint);
    // Start PID loop
    Robot.cargoArmSubsystem.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //System.out.println(RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    // Stop PID loop
    Robot.cargoArmSubsystem.disable();
  }
}
