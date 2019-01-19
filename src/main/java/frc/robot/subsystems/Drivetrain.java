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

public class Drivetrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	double kP = 0.01, kI = 0.0, kD = 0.0;
	double integral = 0, previous_error = 0;
	DifferentialDrive differentialDrive;

	public Drivetrain() {
		// Seting differentialDrive object to control the motor groups
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
		differentialDrive.curvatureDrive(RobotMap.LEFT_STICK.getY(), -RobotMap.RIGHT_STICK.getX(), true);
	}

	// Test Drivetrain control with XBox Controller. Same as curvatureDrive but instead of joystick inputs, it uses game-pad inputs.
	public void controllerDrive() {
		// If you need to use this code, replace curvatureDrive() with controllerDrive() in CurvatureDriveCommand.java
		differentialDrive.curvatureDrive(RobotMap.GAME_PAD.getY(Hand.kLeft), -RobotMap.GAME_PAD.getX(Hand.kRight), true);
	}

	// Initialise gyro PID
	public void initGyroDrive() {
		// reset integral accumulator and previous_error
		this.integral = 0;
		this.previous_error = 0;
		// reset NavX to 0 degrees
		RobotMap.navx.reset();
	}

	// Gyro Driving Straight
	public void gyroDriveStraight() {
		// drive toward 0 degrees
		double error = -this.getAngle();
		// accumulate error between calls, method is called approx. every 16.67 ms
		this.integral += error * 0.01666666667;
		// see how the error is changing over time, method is called approx every 16.67 ms
		double derivative = (error - this.previous_error) / 0.01666666667;
		// combine P, I, D terms to produce turning output
		double turn_power = this.kP * error + this.kI * this.integral + this.kD * derivative;
		// update previous_error
		this.previous_error = error;
		// drive with 'LEFT_STICK' throttle, and 'turn_power' rotation; no squared inputs
		differentialDrive.arcadeDrive(RobotMap.LEFT_STICK.getY(), turn_power, false);
	}

	// Creates stop method to stop all motors on drivetrain
	public void stop() {
		differentialDrive.stopMotor();
	}

	// Get angle of drivetrain from NavX correcting for accumulation
	public double getAngle() {
		double angle = RobotMap.navx.getAngle();
		
		// correct for values beyond 360 and -360
		if (angle >= 0) return angle % 360;
		else return angle % -360;
	}

}