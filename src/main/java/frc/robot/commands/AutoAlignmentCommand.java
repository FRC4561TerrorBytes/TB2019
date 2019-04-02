/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class AutoAlignmentCommand extends Command {
  public AutoAlignmentCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivetrain.initAngleDrive();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    RobotMap.navx.reset();
    double visionAngle = Robot.networkTable.getEntry("centerangle").getDouble(0);
    // If we see a valid vision target aim at it
    if(Robot.networkTable.getEntry("validleft").getBoolean(false)&&Robot.networkTable.getEntry("validright").getBoolean(false)){
      //System.out.println("target acquired");
      Robot.drivetrain.angleDriveStraight(visionAngle);
    } else { //Else keep driving as usual
      Robot.drivetrain.curvatureDrive();
    }
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
  }
}
