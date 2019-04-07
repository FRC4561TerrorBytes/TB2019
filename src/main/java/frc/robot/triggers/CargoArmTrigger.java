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
 * @author Snehil
 */
public class CargoArmTrigger extends Trigger {
  @Override
  public boolean get() {
    return Math.abs(RobotMap.GAME_PAD.getY(Hand.kLeft)) > 0.08; // If the xbox left joystick is moved noticably, this returns true
  }
}
