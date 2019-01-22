/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class CargoArmSubsystem extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public CargoArmSubsystem(){
    // values: P,I,D,F TODO: tune
    super(0, 0, 0, 0);
    RobotMap.CARGO_ARM_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    RobotMap.CARGO_ARM_MOTOR.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		RobotMap.CARGO_ARM_MOTOR.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    return RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition(0);
  }

  @Override
  public void usePIDOutput(double goal) {
    RobotMap.CARGO_ARM_MOTOR.set(ControlMode.Position, goal);
  }

  public void armUp(){
    RobotMap.CARGO_ARM_MOTOR.set(ControlMode.PercentOutput, 1);
  }

  public void armDown(){
    RobotMap.CARGO_ARM_MOTOR.set(ControlMode.PercentOutput, -1);
  }

  public void stop(){
    RobotMap.CARGO_ARM_MOTOR.set(ControlMode.PercentOutput, 0);
  }
  public boolean getFwdSwitch(){
		return RobotMap.CARGO_ARM_MOTOR.getSensorCollection().isFwdLimitSwitchClosed();
	}
	public boolean getRevSwitch(){
		return RobotMap.CARGO_ARM_MOTOR.getSensorCollection().isRevLimitSwitchClosed();
	}
}
