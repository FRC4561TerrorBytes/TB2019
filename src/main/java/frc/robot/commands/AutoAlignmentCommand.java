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
  /* TODO: Kongo seems to always veer to right when he gets close, 
	   and the x-center value remains real although it certianly can't 
	   see the target. Probably caused due to the pi not getting enough
	   power because the battery was very low in my garage test
	*/ 
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
    // If we see a valid vision target aim at it
    if(Robot.networkTable.getEntry("validleft").getBoolean(false)&&Robot.networkTable.getEntry("validright").getBoolean(false)){
      //System.out.println("target acquired");
      double visionAngle = Robot.networkTable.getEntry("centerangle").getDouble(0);
      RobotMap.navx.setAngleAdjustment(visionAngle); // adjust our current gyro heading by our current vision angle
      /**
      * Note: this works because the adjustment angle is not reset by the reset() method
      * Essentially we are just fudging the real heading (in this case 0)
      * by what our current vision angle is
      */
      RobotMap.navx.reset(); // set gyro value to visionAngle
      double desiredHeading = RobotMap.navx.getAngle();
      Robot.drivetrain.angleDriveStraight(desiredHeading);
    } else { // Else keep driving as usual
      RobotMap.navx.setAngleAdjustment(0);
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
