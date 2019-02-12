/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * @author Karthik, Max
 */

public class ClimberManualCommand extends Command {
  double climberSpeed = 0;

  public ClimberManualCommand(double climberSpeed) {
    requires(Robot.climber);
    this.climberSpeed = climberSpeed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  // Called when the robot needs to ascend into space like it's the Enterprise
  @Override
  protected void execute() {
    Robot.climber.set(this.climberSpeed);
  }

  // Make this return true when this Command no longer needs to run execute()
  // It is set to false since the robot does not need to always ascend
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  // This command calls the stop method in the Climber subsytem, which stops the motors
  @Override
  protected void end() {
    Robot.climber.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  // This calls the end() method above
  @Override
  protected void interrupted() {
    end();
  }
}