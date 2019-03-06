/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ClimberTrigger extends Trigger {
  @Override
  public boolean get() {
    return !(Math.abs(RobotMap.GAME_PAD.getY(Hand.kRight)) > 0.04); // right xbox joystick is active when moved out of deadzone
  }
}
