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
 */

public class CargoArmSubsystem extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  final double ARM_WEIGHT = 14; // lbs
  final double ARM_LENGTH = 16; // in
  final double TORQUE_CONSTANT = 6.284029 / 12; // in*lbs / volts
  final double TICKS_TO_DEGREES = 360 / 4096; // degrees / pulses per revolution

  public CargoArmSubsystem() {
    /* values: P,I,D*/ 
    /* Delta Values: 4, 0.0055, 1023, 3.41*/
    //0.0005, 0.0, 0.0005
    super("CargoArmSubsystem", 0.0005, 0.0, 0.003);
    //Setup sensors
    RobotMap.CARGO_ARM_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    RobotMap.CARGO_ARM_MOTOR.setSelectedSensorPosition(RobotMap.ARM_TOP_LOC);
    // Set PID values for Motion Magic.
    RobotMap.CARGO_ARM_MOTOR.config_kP(0, 1);
    RobotMap.CARGO_ARM_MOTOR.config_kI(0, 0);
    RobotMap.CARGO_ARM_MOTOR.config_kD(0, 0);

    RobotMap.CARGO_ARM_MOTOR.configOpenloopRamp(.5);

    this.resetEncoder();
    // Make it so that the PID will recognize that it has upper and lower limits
    getPIDController().setContinuous(false);
    setInputRange(RobotMap.ARM_BOT_LOC, RobotMap.ARM_TOP_LOC);
    // Set PID max speed
    setOutputRange(-0.4, 0.4);

    // Synchronise encoder
    int absolutePosition = RobotMap.CARGO_ARM_MOTOR.getSensorCollection().getPulseWidthPosition();
    absolutePosition &= 0xFFF;
    RobotMap.CARGO_ARM_MOTOR.setSelectedSensorPosition(absolutePosition);

    // Used to invert the encoder sensor phase WIP
    RobotMap.CARGO_ARM_MOTOR.setSensorPhase(true); // TODO: Should be set as false on robot, true on test platform.
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }

  @Override
  protected double returnPIDInput() {
    // Use cargo arm encoder position for PID input
    SmartDashboard.putNumber("PID Input", RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition());
    return RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition();
  }

  @Override
  protected void usePIDOutput(double output) {
    // int setpointPosition = (int) this.getSetpoint();
    // setpointPosition &= 0xFFF;
    // double angle = (RobotMap.ARM_STRAIGHT_LOC - setpointPosition) *
    // TICKS_TO_DEGREES;
    // double feedForward = (ARM_WEIGHT * ARM_LENGTH / TORQUE_CONSTANT) *
    // Math.cos(angle);
    // getPIDController().setF(feedForward);

    // limit the output of the cargo arm to keep it from slamming around the robot
    if (output > 0.4) RobotMap.CARGO_ARM_MOTOR.set(0.4);
    else if (output < -0.4) RobotMap.CARGO_ARM_MOTOR.set(-0.4);
    else RobotMap.CARGO_ARM_MOTOR.set(output);

    // limit the output of the cargo arm when moving where it would be most affected by gravity
    /*
    if (RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition() > RobotMap.ARM_CARGO_LOC || RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition() < RobotMap.ARM_ROCKET_LOC) {
      RobotMap.CARGO_ARM_MOTOR.set(output / 2);
    }
    */

    SmartDashboard.putNumber("PID Output", output);
  }

  /** For manual movement of the arm using the controller */
  public void armManual(double power) {
    RobotMap.CARGO_ARM_MOTOR.set(power);
  }

  // Method for moving the arm to a position with motion magic WIP
  public void armMotionMagic(double position) {
    RobotMap.CARGO_ARM_MOTOR.set(ControlMode.MotionMagic, position);
  }
  
  /** Method to stop the motor from moving*/
  public void stop() {
    RobotMap.CARGO_ARM_MOTOR.stopMotor();
  }

  public boolean getTopSwitch() {
    return RobotMap.CARGO_ARM_MOTOR.getSensorCollection().isFwdLimitSwitchClosed();
    // gives the status of the top limit switch (true is pressed, false is not
    // pressed)
    // return !RobotMap.ARM_LIMIT_SWITCH_TOP.get();
  }

  public boolean getBottomSwitch() {
    return RobotMap.CARGO_ARM_MOTOR.getSensorCollection().isRevLimitSwitchClosed();
    // gives the status of the bottom limit switch (true is pressed, false is not
    // pressed)
    //return !RobotMap.ARM_LIMIT_SWITCH_BOT.get();
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