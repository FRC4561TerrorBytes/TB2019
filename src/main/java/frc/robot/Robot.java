/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/**
 * TODO:
 * Test drivetain PID toggle
 * Test cargo intake passive
 * Test climber passive
 * 
 * For the new arm:
 * make sure encoder is going the right way
 * start tuning PID
 */

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CrosshairCommand;
import frc.robot.subsystems.CargoArmSubsystem;
import frc.robot.subsystems.CargoIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HatchIntakeSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static DriveSubsystem drivetrain;
  public static HatchIntakeSubsystem hatchIntake = new HatchIntakeSubsystem();
  public static CargoIntakeSubsystem cargoIntake = new CargoIntakeSubsystem();
  public static CargoArmSubsystem cargoArmSubsystem = new CargoArmSubsystem();
  public static ClimberSubsystem climber = new ClimberSubsystem();
  public static OI oi;
  public static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  public static edu.wpi.first.networktables.NetworkTable networkTable;
  public static UsbCamera camera1;
  public static UsbCamera camera2;
  Command autonomousCommand;
  Command crosshairCommand = new CrosshairCommand();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    drivetrain = new DriveSubsystem();
    oi = new OI();
    RobotMap.navx.reset();
    //cargoArmSubsystem.resetEncoder();
    //RobotMap.MAGIC_POS = Robot.cargoArmSubsystem.getPosition();
    networkTableInstance.startServer();
    /*
     * All values sent to the robot MUST be sent on
     * this table (networkTable)
     */
    networkTable = networkTableInstance.getTable("networkTable");
    // start cameras and configure settings
    if (RobotMap.TWO_CAMERAS) {
      // To make the crosshair camera work we need to create a new thread. Why? Who knows.
      // if we are using to cameras create 2 cameras running at 10 fps
      camera1 = CameraServer.getInstance().startAutomaticCapture();
      camera2 = CameraServer.getInstance().startAutomaticCapture();
      camera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
      camera2.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
      camera1.setResolution(176, 144);
      camera2.setResolution(176, 144);
      camera1.setFPS(15);
      camera2.setFPS(15);
      camera1.setBrightness(25);
      camera2.setBrightness(25);
      camera1.setExposureManual(10);
      camera2.setExposureManual(10);
      camera1.setWhiteBalanceManual(10);
      camera2.setWhiteBalanceManual(10);
    } else {
      // To make the crosshair camera work we need to create a new thread. Why? Who knows.
      // if we are not using two cameras, create one camera running at 30 fps
      camera1 = CameraServer.getInstance().startAutomaticCapture();
      camera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
      camera1.setResolution(176, 144);
      camera1.setFPS(30);
      camera1.setBrightness(25);
      camera1.setExposureManual(10);
      camera1.setWhiteBalanceManual(10);
    }
    // Run command to create a modified stream of camera1 (Hatch side cam) with crosshairs and vision target aid
    //crosshairCommand.start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diag ostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // numbers retrieved from raspi
    SmartDashboard.putNumber("PixyAngle", networkTable.getEntry("pixyAngle").getDouble(4561));
    double xCenter = networkTable.getEntry("xcenter").getDouble(4561);
    SmartDashboard.putNumber("X-Center", xCenter);
    SmartDashboard.putNumber("Y-Center", networkTable.getEntry("ycenter").getDouble(4561));
    SmartDashboard.putNumber("VisionAngle", Robot.networkTable.getEntry("centerangle").getDouble(0));
    // Boolean which is true when we have a target to aim at
    boolean validTarget = Robot.networkTable.getEntry("validleft").getBoolean(false)&&
    Robot.networkTable.getEntry("validright").getBoolean(false);
    // Display whether or not we have a target
    SmartDashboard.putBoolean("Target Aquired", validTarget);
    // numbers retrived from robot
    SmartDashboard.putNumber("Left Stick Y axis", RobotMap.GAME_PAD.getY(Hand.kLeft));
    SmartDashboard.putNumber("CargoArmEncoderPos", RobotMap.CARGO_ARM_MOTOR.getSelectedSensorPosition());
    SmartDashboard.putNumber("GyroYawAngle", drivetrain.getYawAngle());
    RobotMap.ARM_ANGLE = Math.toRadians((Robot.cargoArmSubsystem.getPosition() * 3)/-200 - 45);
    SmartDashboard.putNumber("Arm Angle:", Math.toDegrees(RobotMap.ARM_ANGLE));
    RobotMap.ARM_ANGLE_COS = Math.cos(RobotMap.ARM_ANGLE);
    SmartDashboard.putNumber("Cosine of Arm Angle:", RobotMap.ARM_ANGLE_COS);
    RobotMap.ARM_LAG = Math.abs(Robot.cargoArmSubsystem.getPosition() - Robot.cargoArmSubsystem.getSetpoint());
		SmartDashboard.putNumber("Arm Lag", RobotMap.ARM_LAG);
    SmartDashboard.putBoolean("Arm Bottom Limit Switch", RobotMap.CARGO_ARM_MOTOR.getSensorCollection().isRevLimitSwitchClosed());
    SmartDashboard.putBoolean("Arm Top Limit Switch", RobotMap.CARGO_ARM_MOTOR.getSensorCollection().isFwdLimitSwitchClosed());
    SmartDashboard.putNumber("Arm Velocity", RobotMap.CARGO_ARM_MOTOR.get());
    SmartDashboard.putBoolean("DriveTrain PID Toggle", RobotMap.DRIVE_PID_TOGGLE);
    SmartDashboard.putNumber("Back Right Motor Value", RobotMap.BACK_RIGHT_MOTOR.get());
    SmartDashboard.putNumber("Middle Right Motor Value", RobotMap.MID_RIGHT_MOTOR.get());
    SmartDashboard.putNumber("Front Right Motor Value", RobotMap.MID_RIGHT_MOTOR.get());
    SmartDashboard.putNumber("Cargo Arm value", RobotMap.CARGO_ARM_MOTOR.get());
    SmartDashboard.putBoolean("Climber Passive Toggle", RobotMap.CLIMBER_PASSIVE_TOGGLE);
    SmartDashboard.putNumber("intake top:", RobotMap.CARGO_TOP_ROLLER_MOTOR.get());
    SmartDashboard.putNumber("intake bottom:", RobotMap.CARGO_BOTTOM_ROLLER_MOTOR.get());
    //SmartDashboard.putBoolean("PID Enabled?", cargoArmSubsystem.g);


    //RobotMap.CARGO_ARM_MOTOR.setNeutralMode();

    // Reset the encoder value to the right position when the according limit switch is pressed
    
    if (Robot.cargoArmSubsystem.getTopSwitch()) {
      //System.out.println("top switch tripped");
      Robot.cargoArmSubsystem.resetEncoderTop();
      //System.out.println("Top limit switch pressed");
    }
    
    if (Robot.cargoArmSubsystem.getBottomSwitch()) {
      //System.out.println("bottom switch tripped");
      Robot.cargoArmSubsystem.resetEncoderBot();
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    RobotMap.navx.reset();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}