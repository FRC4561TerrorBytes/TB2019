/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * @author Lucas
 * TODO: Comment code
 */

public class CargoArmSubsystem extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  final double ARM_WEIGHT = 14; // lbs
  final double ARM_LENGTH = 16; // in
  final double TORQUE_CONSTANT = 6.284029 / 12; // in*lbs / volts
  final double TICKS_TO_DEGREES = 360 / 4096; // degrees / pulses per revolution

  public CargoArmSubsystem() {
    /* values: P,I,D,F,Period (stays constant); Robot_006 Values: TODO: find values for comp robot */ 
    /* Delta Values: 4, 0.0055, 1023, 3.41*/
    super("CargoArmSubsystem", 1.0, 0.0, 0.0);
    //Setup sensors
    RobotMap.CARGO_ARM_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    RobotMap.CARGO_ARM_MOTOR.setSelectedSensorPosition(RobotMap.ARM_TOP_LOC);
    this.resetEncoder();
    // Make it so that the PID will recognize that it has upper and lower limits
    getPIDController().setContinuous(false);
    setInputRange(RobotMap.ARM_BOT_LOC, RobotMap.ARM_TOP_LOC);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

  @Override
  protected double returnPIDInput() {
    SmartDashboard.putNumber("PID Input", RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition(0));
    return RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition(0);
  }

  @Override
  protected void usePIDOutput(double output) {
    double angle = (RobotMap.ARM_STRAIGHT_LOC - this.getSetpoint()) * TICKS_TO_DEGREES;
    double feedForward = (ARM_WEIGHT * ARM_LENGTH / TORQUE_CONSTANT) * Math.cos(angle);
    getPIDController().setF(feedForward);
    RobotMap.CARGO_ARM_MOTOR.set(output);
  }

  /** For manual movement of the arm using the controller */
  public void armManual(double power) {
    RobotMap.CARGO_ARM_MOTOR.set(power);
  }
  
  /** Method to stop the motor from moving*/
  public void stop() {
    RobotMap.CARGO_ARM_MOTOR.stopMotor();
  }

  public boolean getTopSwitch() {
    //return RobotMap.CARGO_ARM_MOTOR.getSensorCollection().isFwdLimitSwitchClosed();
    return !RobotMap.ARM_LIMIT_SWITCH_TOP.get();
	}
	public boolean getBottomSwitch() {
    //return RobotMap.CARGO_ARM_MOTOR.getSensorCollection().isRevLimitSwitchClosed();
    return !RobotMap.ARM_LIMIT_SWITCH_BOT.get();
  }

  public void resetEncoder() {
    // Reset the encoder values to 0
    RobotMap.CARGO_ARM_MOTOR.setSelectedSensorPosition(RobotMap.ARM_TOP_LOC);
  }

  public void setEncoder() {
    // Set encoder value to the bottom position
    RobotMap.CARGO_ARM_MOTOR.setSelectedSensorPosition(RobotMap.ARM_BOT_LOC);
  }
}