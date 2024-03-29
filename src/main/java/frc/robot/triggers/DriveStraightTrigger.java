/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveStraightTrigger extends Trigger {
  @Override
  public boolean get() {
    // Return when driver is not trying to turn the robot
    return (Math.abs(RobotMap.RIGHT_STICK.getX()) < 0.01) && (Math.abs(RobotMap.LEFT_STICK.getY()) > 0.01);
  }
}
