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
 * @author Max
 */
public class CargoIntakeTrigger extends Trigger {
  @Override
  public boolean get() {
    return (RobotMap.GAME_PAD.getTriggerAxis(Hand.kRight) > 0.25); // Returns true when right trigger is held.
  }
}
