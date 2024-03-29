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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * @author Lucas
 */

public class CargoArmSubsystem extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  final double ARM_WEIGHT = 12; // lbs
  final double ARM_LENGTH = 16; // in
  final double TORQUE_CONSTANT = 6.284029 / 12; // in*lbs / volts
  final double TICKS_TO_DEGREES = 180 / 9000; // degrees / pulses per revolution

  public CargoArmSubsystem() {
    /* values: P,I,D*/ 
    /* Delta Values: 4, 0.0055, 1023, 3.41*/
    //0.000
    // test arm values: 0.0004, 0.0, 0.003
    // test arm value with terror magic with good feedforward: 0.0009, 0.000, .01
    // Orion v1 arm values: 0.0008, 0.0, 0.008
    super("CargoArmSubsystem", 0.0009, 0.000, .008);
    //Setup sensors0
    RobotMap.CARGO_ARM_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    RobotMap.CARGO_ARM_MOTOR.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    RobotMap.CARGO_ARM_MOTOR.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // Set PID values for Motion Magic.
    /*
    RobotMap.CARGO_ARM_MOTOR.config_kF(0, 2.273);
    RobotMap.CARGO_ARM_MOTOR.config_kP(0, 0.5);
    RobotMap.CARGO_ARM_MOTOR.config_kI(0, 0);
    RobotMap.CARGO_ARM_MOTOR.config_kD(0, 0);
    RobotMap.CARGO_ARM_MOTOR.configMotionAcceleration(23);
    RobotMap.CARGO_ARM_MOTOR.configMotionCruiseVelocity(23);
    */

    RobotMap.CARGO_ARM_MOTOR.configOpenloopRamp(.5);

    // Make it so that the PID will recognize that it has upper and lower limits
    getPIDController().setContinuous(false);
    setInputRange(RobotMap.ARM_TOP_LOC, RobotMap.ARM_BOT_LOC);
    // Set PID max speed
    // -0.7, 0.7
    //setOutputRange(-0.6, 0.6); //TODO: Increase maximum output bc Namh said it was too slow

    // Synchronise encoder
    int absolutePosition = RobotMap.CARGO_ARM_MOTOR.getSensorCollection().getPulseWidthPosition();
    absolutePosition &= 0xFFF; //TODO: Ask Vignesh what this does
    RobotMap.CARGO_ARM_MOTOR.setSelectedSensorPosition(absolutePosition);

    // Used to invert the encoder sensor phase WIP
    RobotMap.CARGO_ARM_MOTOR.setSensorPhase(true); // TODO: 
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
    int setpointPosition = (int) this.getSetpoint();
    setpointPosition &= 0xFFF;
    double angle = (RobotMap.ARM_STRAIGHT_LOC - setpointPosition) *
    TICKS_TO_DEGREES;
    double feedForward = (ARM_WEIGHT * ARM_LENGTH / TORQUE_CONSTANT) *
    Math.cos(angle);
    getPIDController().setF(feedForward);

    if(RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition() > -5500 && RobotMap.CARGO_ARM_MOTOR.get() < 0
    || RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition() < -7500 && RobotMap.CARGO_ARM_MOTOR.get() > 0) {
        RobotMap.CARGO_ARM_MOTOR.set(-output * 0.001);
      System.out.println("super gravity");
    } else {
        RobotMap.CARGO_ARM_MOTOR.set(-output * 0.5);
    }

    // limit the output of the cargo arm to keep it from slamming around the robot TODO: probably redundant
    /*
    // * Math.cos(RobotMap.ARM_ANGLE)
    if(RobotMap.ARM_ANGLE_COS < 0.5) {
      RobotMap.ARM_ANGLE_COS = 0.5;
    }
    if(RobotMap.ARM_LAG < 250) {
      output = output * 2;
    }
    RobotMap.CARGO_ARM_MOTOR.set(output * RobotMap.ARM_ANGLE_COS / 1.5); // negative for Orion, positive on test board
    */
    /*
    // limit the output of the cargo arm when moving where it would be most affected by gravity
    if (RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition() > -1000 && RobotMap.CARGO_ARM_MOTOR.get() < 0 || 
    RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition() < -8500 && RobotMap.CARGO_ARM_MOTOR.get() > 0) {
      RobotMap.CARGO_ARM_MOTOR.set(output * .5);
    }

    SmartDashboard.putNumber("PID Output", output);
    */
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
  }

  public boolean getBottomSwitch() {
    return RobotMap.CARGO_ARM_MOTOR.getSensorCollection().isRevLimitSwitchClosed();
    // gives the status of the bottom limit switch (true is pressed, false is not
    // pressed)
    //return !RobotMap.ARM_LIMIT_SWITCH_BOT.get();
  }
  

  public void resetEncoderTop() {
    // Reset the encoder values to the top position
    //System.out.println("Top set");
    RobotMap.CARGO_ARM_MOTOR.setSelectedSensorPosition(RobotMap.ARM_CARGO_LOC);
  }

  public void resetEncoderBot() {
    // Set encoder value to the bottom position
    //System.out.println("Bot set");
    RobotMap.CARGO_ARM_MOTOR.setSelectedSensorPosition(RobotMap.ARM_BOT_LOC);
  }
}