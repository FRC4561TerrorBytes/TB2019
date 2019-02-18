/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
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
  public static final int DRIVE_MODE = 1; // 1 is curve drive, 0 is tank drive

  // Invert Drive. -1 : Hatch is front, 1 : Cargo is front

  public static int INVERT_DRIVE = -1;
  
  // Pixy/Vision toggle Booleans
  public static final boolean PIXY_DRIVE_TOGGLE = true; //make true if you want to use pixy guidance

  // Toggle for switching between various settings different Delta and the competition robot
  public static final boolean ON_DELTA = false;

  // Robot control loop frequency, in seconds
  public static final double ROBOT_CONTROL_LOOP_INTERVAL = 0.01666666667;
  
  // Joystick Ports
  public static final int RIGHT_JOYSTICK_PORT = 0;  // TODO: Assign values to the engine, Scotty.
  public static final int LEFT_JOYSTICK_PORT = 1;

  // Right motor ports. Delta Ports: 8,9,10
  public static int FRONT_RIGHT_MOTOR_PORT;
  public static int MID_RIGHT_MOTOR_PORT;
  public static int BOT_RIGHT_MOTOR_PORT;

  // Left motor ports	Delta Ports: 7,11,12
  public static int FRONT_LEFT_MOTOR_PORT;
  public static int MID_LEFT_MOTOR_PORT;
  public static int BOT_LEFT_MOTOR_PORT;

  public static void checkIfDelta(){
    if(ON_DELTA) {
      FRONT_RIGHT_MOTOR_PORT = 8;
      MID_RIGHT_MOTOR_PORT = 9;
      BOT_RIGHT_MOTOR_PORT = 10;

      FRONT_LEFT_MOTOR_PORT = 7;
      MID_LEFT_MOTOR_PORT = 11;
      BOT_LEFT_MOTOR_PORT = 12;
    }
    else {
      FRONT_RIGHT_MOTOR_PORT = 3;
      MID_RIGHT_MOTOR_PORT = 4;
      BOT_RIGHT_MOTOR_PORT = 5;

      FRONT_LEFT_MOTOR_PORT = 6;
      MID_LEFT_MOTOR_PORT = 1;
      BOT_LEFT_MOTOR_PORT = 2;
    }
  }
  // hatch intake solenoid ports
  private static final int INTAKE_POS_PORT = 2;
  private static final int INTAKE_IN_PORT = 3;
  private static final int INTAKE_OUT_PORT = 5;

  // Xbox Controller Port
  public static final int XBOX_CONTROLLER_PORT = 2;

  // Cargo Intake motor ports
  private static final int CARGO_ARM_MOTOR_PORT = 7;
  private static final int CARGO_TOP_ROLLER_PORT = 10;
  private static final int CARGO_BOT_ROLLER_PORT = 9;

  // Climber motor ports
  private static final int CLIMBER_LEFT_PORT = 0;
  private static final int CLIMBER_RIGHT_PORT = 8;

  // Max/Min arm encoder positions/limit switch locations
  public static final int ARM_BOT_LOC = -7119;
  public static final int ARM_TOP_LOC = 2025;
  public static final int ARM_CARGO_LOC = 1448;
  public static final int ARM_ROCKET_LOC = 2025;

  // Ski solenoid ports
  private static final int SKI_PORT = 4;

  // Infrared Sensor port
  private static final int INFRARED_PORT = 3;

  // Arm limit switch ports
  private static final int LIMIT_SWITCH_ARM_BOT_PORT = 1;
  private static final int LIMIT_SWITCH_ARM_TOP_PORT = 0;

  // Climber limit switch ports
  private static final int LIMIT_SWITCH_CLIMBER_PORT = 2;
  

  // Declraing all Drivetrian TalonSRX
  private static final WPI_TalonSRX FRONT_LEFT_MOTOR = new WPI_TalonSRX(FRONT_LEFT_MOTOR_PORT);
  private static final WPI_TalonSRX FRONT_RIGHT_MOTOR = new WPI_TalonSRX(FRONT_RIGHT_MOTOR_PORT);
  private static final WPI_TalonSRX MID_LEFT_MOTOR = new WPI_TalonSRX(MID_LEFT_MOTOR_PORT);
  private static final WPI_TalonSRX MID_RIGHT_MOTOR = new WPI_TalonSRX(MID_RIGHT_MOTOR_PORT); 
  private static final WPI_TalonSRX BACK_LEFT_MOTOR = new WPI_TalonSRX(BOT_LEFT_MOTOR_PORT);
  private static final WPI_TalonSRX BACK_RIGHT_MOTOR = new WPI_TalonSRX(BOT_RIGHT_MOTOR_PORT);

  // Cargo intake variables
  public static final WPI_TalonSRX CARGO_ARM_MOTOR = new WPI_TalonSRX(CARGO_ARM_MOTOR_PORT);
  public static final WPI_VictorSPX CARGO_TOP_ROLLER_MOTOR = new WPI_VictorSPX(CARGO_TOP_ROLLER_PORT);
  public static final WPI_VictorSPX CARGO_BOTTOM_ROLLER_MOTOR = new WPI_VictorSPX(CARGO_BOT_ROLLER_PORT);

  // Hatch intake variables
  public static final Solenoid HATCH_POSITION_SOLENOID = new Solenoid(INTAKE_POS_PORT);
  public static final Solenoid HATCH_INTAKE_SOLENOID = new Solenoid(INTAKE_IN_PORT);
  public static final Solenoid HATCH_OUTTAKE_SOLENOID = new Solenoid(INTAKE_OUT_PORT);

  // Climber subsystem variables
  private static final WPI_TalonSRX LEFT_CLIMBER_MOTOR = new WPI_TalonSRX(CLIMBER_LEFT_PORT);
  private static final WPI_TalonSRX RIGHT_CLIMBER_MOTOR = new WPI_TalonSRX(CLIMBER_RIGHT_PORT);

  // Ski solenoids objects
  public static final Solenoid SKI_SOLENOID = new Solenoid(SKI_PORT);

  // Creates SpeedController object to combine left and right side motors
  public static final SpeedControllerGroup LEFT_SIDE_MOTOR_GROUP = new SpeedControllerGroup(FRONT_LEFT_MOTOR, MID_LEFT_MOTOR, BACK_LEFT_MOTOR);
  public static final SpeedControllerGroup RIGHT_SIDE_MOTOR_GROUP = new SpeedControllerGroup(FRONT_RIGHT_MOTOR, MID_RIGHT_MOTOR, BACK_RIGHT_MOTOR);
    
  // Climber motor group
  public static final SpeedControllerGroup CLIMBER_MOTOR_GROUP = new SpeedControllerGroup(LEFT_CLIMBER_MOTOR, RIGHT_CLIMBER_MOTOR);

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
