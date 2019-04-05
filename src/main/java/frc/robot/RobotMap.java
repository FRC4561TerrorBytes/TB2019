/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
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

  // Drive Mode
  public static int DRIVE_MODE = 1; // 1 is curve drive, 0 is tank drive

  public static boolean CLIMBER_PASSIVE_TOGGLE = true; // false is disabled(passive is off) true is enabled(passive is on)

  //Manual Setpoint for Motion Magic
  public static int MAGIC_POS = 0;

  // Drivetrain toggle
  public static boolean DRIVE_PID_TOGGLE = false; // false is disabled, true is enabled

  // Drive invert toggle
  public static int DRIVE_INVERT_TOGGLE = 1;

  // Hatch Intake toggle
  public static boolean HATCH_TOGGLE = false; // false is retracted, true is extended
  
  // Pixy/Vision toggle Booleans
  public static final boolean PIXY_DRIVE_TOGGLE = true; //make true if you want to use pixy guidance
  
  // Toggle for using one camera (false) or two cameras (true)
  public static final boolean TWO_CAMERAS = true;

  // Robot control loop frequency, in seconds
  public static final double ROBOT_CONTROL_LOOP_INTERVAL = 0.01666666667;
  
  // Joystick Ports
  public static final int RIGHT_JOYSTICK_PORT = 1;
  public static final int LEFT_JOYSTICK_PORT = 0;

  // Right motor ports. Delta Ports: 8,9,10 Kongo Ports: 1
  public static int FRONT_RIGHT_MOTOR_PORT = 1;
  public static int MID_RIGHT_MOTOR_PORT = 4;
  public static int BOT_RIGHT_MOTOR_PORT = 5;

  // Left motor ports	Delta Ports: 7,11,12 Kongo Ports: 2
  public static int FRONT_LEFT_MOTOR_PORT = 2;
  public static int MID_LEFT_MOTOR_PORT = 3;
  public static int BOT_LEFT_MOTOR_PORT = 6;

  // hatch intake solenoid ports
  // private static final int INTAKE_POS_PORT = 2; // Not using right now.
  private static final int INTAKE_EXTEND_PORT = 2;
  private static final int INTAKE_GRAB_PORT = 3;

  // Xbox Controller Port
  public static final int XBOX_CONTROLLER_PORT = 2;

  // Cargo Intake motor ports
  private static final int CARGO_ARM_MOTOR_PORT = 7;
  private static final int CARGO_TOP_ROLLER_PORT = 9;
  private static final int CARGO_BOT_ROLLER_PORT = 10;

  // Climber motor ports
  private static final int CLIMBER_LEFT_PORT = 0;
  private static final int CLIMBER_RIGHT_PORT = 8;

  // Max/Min arm encoder positions/limit switch locations TODO: find real values
  public static final int ARM_BOT_LOC = 0;
  public static final int ARM_TOP_LOC = -9000;
  public static final int ARM_CARGO_LOC = -8500;
  public static final int ARM_ROCKET_LOC = -3000;
  public static final int ARM_DEPOT_LOC = -600;
  public static final int ARM_ROCKET_2_LOC = -7200;

  // Ski double solenoid ports
  private static final int SKI_PORT_IN = 4;
  private static final int SKI_PORT_OUT = 5;

  // Infrared Sensor port
  private static final int INFRARED_PORT = 3;

  // Arm limit switch ports
  private static final int LIMIT_SWITCH_ARM_TOP_PORT = 1;
  private static final int LIMIT_SWITCH_ARM_BOT_PORT = 0;

  // Climber limit switch ports
  private static final int LIMIT_SWITCH_CLIMBER_PORT = 2;
  
  // Declraing all Drivetrian TalonSRX
  private static final WPI_TalonSRX FRONT_LEFT_MOTOR = new WPI_TalonSRX(FRONT_LEFT_MOTOR_PORT);
  private static final WPI_TalonSRX FRONT_RIGHT_MOTOR = new WPI_TalonSRX(FRONT_RIGHT_MOTOR_PORT);
  private static final WPI_TalonSRX MID_LEFT_MOTOR = new WPI_TalonSRX(MID_LEFT_MOTOR_PORT);
  public static final WPI_TalonSRX MID_RIGHT_MOTOR = new WPI_TalonSRX(MID_RIGHT_MOTOR_PORT); 
  private static final WPI_TalonSRX BACK_LEFT_MOTOR = new WPI_TalonSRX(BOT_LEFT_MOTOR_PORT);
  public static final WPI_TalonSRX BACK_RIGHT_MOTOR = new WPI_TalonSRX(BOT_RIGHT_MOTOR_PORT); // TODO: Fix this.

  // Cargo intake variables
  public static final WPI_TalonSRX CARGO_ARM_MOTOR = new WPI_TalonSRX(CARGO_ARM_MOTOR_PORT);
  public static final WPI_VictorSPX CARGO_TOP_ROLLER_MOTOR = new WPI_VictorSPX(CARGO_TOP_ROLLER_PORT);
  public static final WPI_VictorSPX CARGO_BOTTOM_ROLLER_MOTOR = new WPI_VictorSPX(CARGO_BOT_ROLLER_PORT);

  // Hatch intake variables
  // public static final Solenoid HATCH_POSITION_SOLENOID = new Solenoid(INTAKE_POS_PORT); // Not using right now
  public static final Solenoid HATCH_EXTEND_SOLENOID = new Solenoid(INTAKE_EXTEND_PORT);
  public static final Solenoid HATCH_GRABBER_SOLENOID = new Solenoid(INTAKE_GRAB_PORT);

  // Climber subsystem variables
  public static final WPI_TalonSRX LEFT_CLIMBER_MOTOR = new WPI_TalonSRX(CLIMBER_LEFT_PORT);
  private static final WPI_TalonSRX RIGHT_CLIMBER_MOTOR = new WPI_TalonSRX(CLIMBER_RIGHT_PORT);

  // Ski solenoids objects
  public static final DoubleSolenoid SKI_SOLENOID = new DoubleSolenoid(SKI_PORT_OUT, SKI_PORT_IN);

  // Creates SpeedController object to combine left and right side motors
  public static final SpeedControllerGroup LEFT_SIDE_MOTOR_GROUP = new SpeedControllerGroup(FRONT_LEFT_MOTOR, MID_LEFT_MOTOR, BACK_LEFT_MOTOR);
  public static final SpeedControllerGroup RIGHT_SIDE_MOTOR_GROUP = new SpeedControllerGroup(FRONT_RIGHT_MOTOR, MID_RIGHT_MOTOR, BACK_RIGHT_MOTOR);
    
  // Climber motor group
  public static final SpeedControllerGroup CLIMBER_MOTOR_GROUP = new SpeedControllerGroup(LEFT_CLIMBER_MOTOR, RIGHT_CLIMBER_MOTOR);

  // Cargo Intake motor group
  public static final SpeedControllerGroup CARGO_INTAKE_MOTOR_GROUP = new SpeedControllerGroup(CARGO_BOTTOM_ROLLER_MOTOR, CARGO_TOP_ROLLER_MOTOR);

  // Declaring NavX MXP
  public static final AHRS navx = new AHRS(SPI.Port.kMXP);

  // Creates Joystick objects for each joystick
  public static final Joystick LEFT_STICK = new Joystick (LEFT_JOYSTICK_PORT);
  public static final Joystick RIGHT_STICK = new Joystick (RIGHT_JOYSTICK_PORT);

  // Creates Xbox Controller object with the controller
  public static final XboxController GAME_PAD = new XboxController(XBOX_CONTROLLER_PORT);

  // Creates object for Infrared Sensor
  public static final DigitalInput HATCH_DETECTOR = new DigitalInput(INFRARED_PORT);

  // Creates object for Arm limit switches
  public static final DigitalInput ARM_LIMIT_SWITCH_TOP = new DigitalInput(LIMIT_SWITCH_ARM_TOP_PORT);
  public static final DigitalInput ARM_LIMIT_SWITCH_BOT = new DigitalInput(LIMIT_SWITCH_ARM_BOT_PORT);

  // Creates object for Climber limit switches
  public static final DigitalInput CLIMBER_LIMIT_SWITCH = new DigitalInput(LIMIT_SWITCH_CLIMBER_PORT);
}