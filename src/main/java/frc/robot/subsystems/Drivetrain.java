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

	// Creates stop method to stop all motors on drivetrain
	public void stop() {
		differentialDrive.stopMotor();
	}

}
