/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/**
 * TODO: Do these below
 * Check axes for gyro
 * check encoder position for top and bottom for climber cargo arm
 * tune and bind cargo arm PID
 * PID buttons
 */
package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public static DriveSubsystem drivetrain = new DriveSubsystem();
  public static HatchIntakeSubsystem hatchIntake = new HatchIntakeSubsystem();
  public static CargoIntakeSubsystem cargoIntake = new CargoIntakeSubsystem();
  public static CargoArmSubsystem cargoArmSubsystem = new CargoArmSubsystem();
  public static ClimberSubsystem climber = new ClimberSubsystem();
  public static OI oi;
  public static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  public static edu.wpi.first.networktables.NetworkTable networkTable;
  Command autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI();
    RobotMap.navx.reset();
    networkTableInstance.startServer();
    networkTable = networkTableInstance.getTable("networkTable");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //numbers retrieved from raspi
    SmartDashboard.putNumber("PixyAngle", networkTable.getEntry("pixyAngle").getDouble(4561));
    SmartDashboard.putNumber("X-Center", networkTable.getEntry("xcenter").getDouble(0));
    SmartDashboard.putNumber("Y-Center", networkTable.getEntry("ycenter").getDouble(0));
    //numbers retrived from robot
    SmartDashboard.putNumber("Right Trigger Y axis", RobotMap.GAME_PAD.getY(Hand.kRight));
    SmartDashboard.putNumber("CargoArmEncoderPos", cargoArmSubsystem.getPosition());
    SmartDashboard.putNumber("GyroYawAngle", drivetrain.getYawAngle());
    SmartDashboard.putNumber("GyroPitchAngle", drivetrain.getPitchAngle());
    SmartDashboard.putNumber("GyroRollAngle", drivetrain.getRollAngle());
    SmartDashboard.putBoolean("Arm Bottom Limit Switch", RobotMap.ARM_LIMIT_SWITCH_BOT.get());
    SmartDashboard.putBoolean("Arm Top Limit Switch", RobotMap.ARM_LIMIT_SWITCH_TOP.get());
    //RobotMap.CARGO_ARM_MOTOR.setNeutralMode();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
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
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
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
