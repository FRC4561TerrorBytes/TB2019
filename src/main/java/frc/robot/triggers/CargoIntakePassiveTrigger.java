/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class CargoIntakePassiveTrigger extends Trigger {
  @Override
  public boolean get() {
    return Robot.cargoArmSubsystem.getTopSwitch() && RobotMap.GAME_PAD.getTriggerAxis(Hand.kLeft) < 0.1 && RobotMap.GAME_PAD.getTriggerAxis(Hand.kRight) < 0.1 && !RobotMap.RIGHT_STICK.getRawButton(1) && !RobotMap.LEFT_STICK.getRawButtonPressed(2) && !RobotMap.LEFT_STICK.getRawButtonPressed(4);
  }
}
