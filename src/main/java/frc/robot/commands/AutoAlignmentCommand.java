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
  double prevPixyAngle = 0;
  double prevVisionAngle = 0;
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
    double pixyAngle = Robot.networkTable.getEntry("pixyAngle").getDouble(0);
    double visionAngle = Robot.networkTable.getEntry("centerangle").getDouble(0);
    if(Robot.networkTable.getEntry("validleft").getBoolean(false)&&Robot.networkTable.getEntry("validright").getBoolean(false)){
      //If the robot is not going the right way, go the other direction
      //System.out.println("target acquired");
      if(visionAngle>prevVisionAngle && visionAngle>0 || visionAngle<prevVisionAngle && visionAngle<0){
        prevVisionAngle = visionAngle;
        if(Math.abs(visionAngle)>1){
          visionAngle=Math.copySign(1, visionAngle);
        }
        //Robot.drivetrain.angleDriveStraight(-visionAngle);
      }
      //The robot is going the right way, keep going
      else{
        prevVisionAngle = visionAngle;
        if(Math.abs(visionAngle)>1){
          pixyAngle=Math.copySign(1, visionAngle);
        }
        //Robot.drivetrain.angleDriveStraight(visionAngle);
      }
    }
    else{
      //If the robot is not going the right way, go the other direction
      if(pixyAngle>prevPixyAngle && pixyAngle>0 || pixyAngle<prevPixyAngle && pixyAngle<0){
        prevPixyAngle = pixyAngle;
        if(Math.abs(pixyAngle)>1){
          pixyAngle=Math.copySign(1, pixyAngle);
        }
        //Robot.drivetrain.angleDriveStraight(-pixyAngle);
      }
      //The robot is going the right way, keep going
      else{
        prevPixyAngle = pixyAngle;
        if(Math.abs(pixyAngle)>1){
          pixyAngle=Math.copySign(1, pixyAngle);
        }
        //Robot.drivetrain.angleDriveStraight(pixyAngle);
      }
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
