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
import frc.robot.commands.CargoArmManualCommand;

/**
 * @author Lucas
 */

public class CargoArmSubsystem extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private int goalPos = 0;

  public CargoArmSubsystem(){
    /* values: P,I,D,F,Period (stays constant); Robot_006 Values: TODO: find values for comp robot */ 
    /* Delta Values: 4, 0.0055, 1023, 3.41*/
    super(0, 0.0, 0, 0.0, 0.01666666667);
    //Setup sensors
    RobotMap.CARGO_ARM_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    RobotMap.CARGO_ARM_MOTOR.setSelectedSensorPosition(RobotMap.ARM_TOP_LOC);
    RobotMap.CARGO_ARM_MOTOR.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    RobotMap.CARGO_ARM_MOTOR.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    //Make it so that the PID will recognize that it has upper and lower limits
    this.getPIDController().setContinuous(false);
    setInputRange(RobotMap.ARM_BOT_LOC, RobotMap.ARM_TOP_LOC);
    //Set the encoder value to it's maximum value when it hits the top limit switch and vice versa
    if(getFwdSwitch()){ 
      RobotMap.CARGO_ARM_MOTOR.setSelectedSensorPosition(RobotMap.ARM_TOP_LOC);
    }
    if(getRevSwitch()){
      RobotMap.CARGO_ARM_MOTOR.setSelectedSensorPosition(RobotMap.ARM_BOT_LOC);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    
       setDefaultCommand(new CargoArmManualCommand());
    
  }

  @Override
  protected double returnPIDInput() {
    return RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition(0);
  }

  @Override
  protected void usePIDOutput(double output) {
    RobotMap.CARGO_ARM_MOTOR.set(ControlMode.PercentOutput, output);
  }

  public void setToGoal() {
    RobotMap.CARGO_ARM_MOTOR.set(ControlMode.Position, goalPos);
  }

  /** For manual movement of the arm using the controller */
  public void armManual(double power){
    RobotMap.CARGO_ARM_MOTOR.set(-power);
  }
  /** Method to stop the motor from moving*/
  public void stop(){
    RobotMap.CARGO_ARM_MOTOR.set(ControlMode.PercentOutput, 0);
  }
  public boolean getFwdSwitch(){
		return RobotMap.CARGO_ARM_MOTOR.getSensorCollection().isFwdLimitSwitchClosed();
	}
	public boolean getRevSwitch(){
		return RobotMap.CARGO_ARM_MOTOR.getSensorCollection().isRevLimitSwitchClosed();
  }

  public void groundPosition() {
    goalPos = RobotMap.ARM_BOT_LOC;
    setToGoal();

  }
  public void shipPosition() {
    goalPos = RobotMap.ARM_BOT_LOC;
    setToGoal();
  }
  public void rocketPosition() {
    goalPos = RobotMap.ARM_BOT_LOC;
    setToGoal();
  }
  public void storagePosition() {
    goalPos = RobotMap.ARM_BOT_LOC;
    setToGoal();
  }
}
