/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  // Drive mode
	public static final int DRIVE_MODE = 1; // 1 is curve drive, 0 is tank drive

  // Joystick Ports
  public static final int RIGHT_JOYSTICK_PORT = 0;  // TODO: Assign values to the engine, Scotty.
  public static final int LEFT_JOYSTICK_PORT = 1;

  //Right motor ports
  public static final int FRONT_RIGHT_MOTOR_PORT = 8;
  public static final int MID_RIGHT_MOTOR_PORT = 9;
  public static final int BOT_RIGHT_MOTOR_PORT = 10;

  // Left motor ports		
  public static final int FRONT_LEFT_MOTOR_PORT = 7;
  public static final int MID_LEFT_MOTOR_PORT = 11;
  public static final int BOT_LEFT_MOTOR_PORT = 12;

  // Intake motor ports
  private static final int INTAKE_HORIZ_PORT = 0; //TODO: change the numbers
  private static final int INTAKE_VERT_FORWARD_PORT = 1;//TODO: change the numbers 
  private static final int INTAKE_VERT_BACK_PORT = 2;//TODO: change the numbers 
  private static final int INTAKE_OUT_PORT = 3; //TODO: change the numbers

  // Xbox Controller Port
  public static final int XBOX_CONTROLLER_PORT = 2;

  // Cargo Intake motor ports
  private static final int CARGO_ARM_MOTOR_PORT = 6;
  private static final int CARGO_TOP_ROLLER_PORT = 0;
  private static final int CARGO_BOT_ROLLER_PORT = 0;

  //Climber motor ports
  private static final int CLIMBER_LEFT_PORT = 0;
  private static final int CLIMBER_RIGHT_PORT = 1;

  //Max/Min arm encoder positions/limit switch locations TODO: find real values
  public static final int ARM_BOT_LIMIT_SWITCH_LOC = 0;
  public static final int ARM_TOP_LIMIT_SWITCH_LOC = 0;

  //Ski solenoid ports
  private static final int LEFT_FORWARD_SKI_PORT = 0; //TODO: change the numbers
  private static final int LEFT_REVERSE_SKI_PORT = 0; //TODO: change the numbers
  private static final int RIGHT_FORWARD_SKI_PORT = 0; //TODO: change the numbers
  private static final int RIGHT_REVERSE_SKI_PORT = 0; //TODO: change the numbers

  // Declraing all Drivetrian TalonSRX
  private static final WPI_TalonSRX FRONT_LEFT_MOTOR = new WPI_TalonSRX(FRONT_LEFT_MOTOR_PORT);
  private static final WPI_TalonSRX FRONT_RIGHT_MOTOR = new WPI_TalonSRX(FRONT_RIGHT_MOTOR_PORT);
  private static final WPI_TalonSRX MID_RIGHT_MOTOR = new WPI_TalonSRX(MID_RIGHT_MOTOR_PORT); 
  private static final WPI_TalonSRX BACK_LEFT_MOTOR = new WPI_TalonSRX(BOT_LEFT_MOTOR_PORT);
  private static final WPI_TalonSRX MID_LEFT_MOTOR = new WPI_TalonSRX(MID_LEFT_MOTOR_PORT);
  private static final WPI_TalonSRX BACK_RIGHT_MOTOR = new WPI_TalonSRX(BOT_RIGHT_MOTOR_PORT);

  //Cargo intake variables
  public static final WPI_TalonSRX CARGO_ARM_MOTOR = new WPI_TalonSRX(CARGO_ARM_MOTOR_PORT);
  private static final WPI_TalonSRX CARGO_TOP_ROLLER_MOTOR = new WPI_TalonSRX(CARGO_TOP_ROLLER_PORT);
  private static final WPI_TalonSRX CARGO_BOTTOM_ROLLER_MOTOR = new WPI_TalonSRX(CARGO_BOT_ROLLER_PORT);

  //Hatch intake variables
  public static final DoubleSolenoid HATCH_POSITION_SOLENOID = new DoubleSolenoid(INTAKE_VERT_FORWARD_PORT, INTAKE_VERT_BACK_PORT);
  public static final Solenoid HATCH_INTAKE_SOLENOID = new Solenoid(INTAKE_HORIZ_PORT);
  public static final Solenoid HATCH_OUTTAKE_SOLENOID = new Solenoid(INTAKE_OUT_PORT);

  //Climber subsystem variables
  private static final WPI_TalonSRX LEFT_CLIMBER_MOTOR = new WPI_TalonSRX(CLIMBER_LEFT_PORT);
  private static final WPI_TalonSRX RIGHT_CLIMBER_MOTOR = new WPI_TalonSRX(CLIMBER_RIGHT_PORT);

  //Ski solenoids objects
  public static final DoubleSolenoid RIGHT_DOUBLE_SKI = new DoubleSolenoid(RIGHT_FORWARD_SKI_PORT, RIGHT_REVERSE_SKI_PORT);
  public static final DoubleSolenoid LEFT_DOUBLE_SKI = new DoubleSolenoid(LEFT_FORWARD_SKI_PORT, LEFT_REVERSE_SKI_PORT);

  //Creates SpeedController object to combine left and right side motors
  public static final SpeedControllerGroup LEFT_SIDE_MOTOR_GROUP = new SpeedControllerGroup(FRONT_LEFT_MOTOR, MID_LEFT_MOTOR, BACK_LEFT_MOTOR);
  public static final SpeedControllerGroup RIGHT_SIDE_MOTOR_GROUP = new SpeedControllerGroup(FRONT_RIGHT_MOTOR, MID_RIGHT_MOTOR, BACK_RIGHT_MOTOR);
  
  // Cargo roller motor group
  public static final SpeedControllerGroup CARGO_ROLLERS_MOTOR_GROUP = new SpeedControllerGroup(CARGO_TOP_ROLLER_MOTOR, CARGO_BOTTOM_ROLLER_MOTOR);
  
  // Climber motor group
  public static final SpeedControllerGroup CLIMBER_MOTOR_GROUP = new SpeedControllerGroup(LEFT_CLIMBER_MOTOR, RIGHT_CLIMBER_MOTOR);

  //Declaring NavX MXP
  public static final AHRS navx = new AHRS(SPI.Port.kMXP);

  //Creates Joystick objects for each joystick
  public static final Joystick LEFT_STICK = new Joystick (LEFT_JOYSTICK_PORT);
  public static final Joystick RIGHT_STICK = new Joystick (RIGHT_JOYSTICK_PORT);

  //Creates Xbox Controller object with the controller
  public static final XboxController GAME_PAD = new XboxController(XBOX_CONTROLLER_PORT);

}
