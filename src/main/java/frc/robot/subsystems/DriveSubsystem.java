/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.CurvatureDriveCommand;
import frc.robot.commands.TankDriveCommand;

/**
* @author Snehil
*/

public class DriveSubsystem extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	// PID Values set for Delta:
	// double kP = 0.21, kI = 0.6, kD = 0.01375;
	// Ku: Maximum kP value that gives ocillation
	// Tu: Time for full ocillation on the robot
	// PID Values: kP = 0.6Ku  kI = 1.2Ku/Tu  kD = 3KuTu/40
	double kP = 0.1, kI = 0.0, kD = 0.0003;
	double integral = 0, previous_error = 0;
	DifferentialDrive differentialDrive;
	int invertDrive = 1;
	
	public DriveSubsystem() {
		// Seting differentialDrive object to control the motor groups
		RobotMap.BACK_RIGHT_MOTOR.setInverted(false); // TODO: Fix this.
		differentialDrive = new DifferentialDrive(RobotMap.LEFT_SIDE_MOTOR_GROUP, RobotMap.RIGHT_SIDE_MOTOR_GROUP);
	}
	
	@Override
	public void initDefaultCommand() {
		// Sets DriveTrain's initial command to use TankDrive or CurvatureDrive based on DRIVE_MODE in RobotMap
		
		if (RobotMap.DRIVE_MODE == 1) setDefaultCommand(new CurvatureDriveCommand());
		else setDefaultCommand(new TankDriveCommand());
	}
	
	// Create tank drive method to move each side of motor based on each joystick y position. true - Decreases input sensitivity at low speeds 
	public void tankDrive() {
		differentialDrive.tankDrive(RobotMap.LEFT_STICK.getY(), RobotMap.RIGHT_STICK.getY(), true);
	}
	
	// Create curve drive method to use the left joystick for speed and right joystick for rotation. true - allows in-place turning manuevers, or "QuickTurn"
	public void curvatureDrive() {
		// Square inputs to curvatureDrive while maintaining sign
		differentialDrive.curvatureDrive(
		invertDrive * Math.copySign(Math.pow(RobotMap.LEFT_STICK.getY(), 2), RobotMap.LEFT_STICK.getY()),
		Math.copySign(Math.pow(RobotMap.RIGHT_STICK.getX(), 2), RobotMap.RIGHT_STICK.getX()), 
		true);
	}
	
	public void invertDrive() {
		// invert the front of the robot
		invertDrive *= -1;
	}
	
	// Test Drivetrain control with XBox Controller. Same as curvatureDrive but instead of joystick inputs, it uses game-pad inputs.
	public void controllerDrive() {
		// If you need to use this code, replace curvatureDrive() with controllerDrive() in CurvatureDriveCommand.java
		differentialDrive.curvatureDrive(RobotMap.GAME_PAD.getY(Hand.kLeft), -RobotMap.GAME_PAD.getX(Hand.kRight), true);
	}
	
	// Initialise gyro PID
	public void initAngleDrive() {
		// reset integral accumulator and previous_error
		this.integral = 0;
		this.previous_error = 0;
		// reset NavX to 0 degrees
		RobotMap.navx.reset();
	}
	
	// Gyro Driving Straight
	public void angleDriveStraight(double angle) {
		// drive toward 0 degrees
		double error = angle;
		// accumulate error between calls, method is called approx. every 16.67 ms
		this.integral += error * RobotMap.ROBOT_CONTROL_LOOP_INTERVAL;
		// see how the error is changing over time, method is called approx every 16.67 ms
		double derivative = (error - this.previous_error) / RobotMap.ROBOT_CONTROL_LOOP_INTERVAL;
		// combine P, I, D terms to produce turning output
		double turn_power = this.kP * error + this.kI * this.integral + this.kD * derivative;
		// update previous_error
		this.previous_error = error;
		// drive with 'LEFT_STICK' throttle, and 'turn_power' rotation; no squared inputs
		if(RobotMap.PIXY_DRIVE_TOGGLE){
			differentialDrive.arcadeDrive(invertDrive * RobotMap.LEFT_STICK.getY(), -turn_power, false);
		} else {
			differentialDrive.arcadeDrive(invertDrive * RobotMap.LEFT_STICK.getY(), turn_power, false);
		}
	}
	
	// Creates stop method to stop all motors on drivetrain
	public void stop() {
		differentialDrive.stopMotor();
	}
	
	// Get Yaw angle of drivetrain from NavX
	public double getYawAngle() {
		return RobotMap.navx.getYaw();
	}	
}