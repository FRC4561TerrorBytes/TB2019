/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.commands.*;
import frc.robot.triggers.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  Button lButton1 = new JoystickButton(RobotMap.LEFT_STICK, 1); // Hatch outtake
  Button lButton2 = new JoystickButton(RobotMap.LEFT_STICK, 2); // Cargo outtake (slow)
  Button lButton3 = new JoystickButton(RobotMap.LEFT_STICK, 3); // Auto Align
  Button lButton4 = new JoystickButton(RobotMap.LEFT_STICK, 4); // Cargo outtake (fast)
  Button rButton1 = new JoystickButton(RobotMap.RIGHT_STICK, 1); // Cargo intake in.
  Button rButton2 = new JoystickButton(RobotMap.RIGHT_STICK, 2); // Invert drive
  Button rButton3 = new JoystickButton(RobotMap.RIGHT_STICK, 3); // Hatch extend/retract.
  Button rButton5 = new JoystickButton(RobotMap.RIGHT_STICK, 5); // Disable Drivetrain PID
  Button rButton14 = new JoystickButton(RobotMap.RIGHT_STICK, 14); // Switch to Camera 1
  Button rButton16 = new JoystickButton(RobotMap.RIGHT_STICK, 16); // Score cargo on rocket lvl 2
  Button rButton8 = new JoystickButton(RobotMap.RIGHT_STICK, 8); // Switch to camera 2
  POVButton lPovLeft = new POVButton(RobotMap.LEFT_STICK, 270); // Extend Hatch mechanism.
  POVButton lPovRight = new POVButton(RobotMap.LEFT_STICK, 90); // Retract Hatch mechanism.
  Button buttonXboxA = new JoystickButton(RobotMap.GAME_PAD, 1); // Arm at intake/bottom location
  Button buttonXboxB = new JoystickButton(RobotMap.GAME_PAD, 2); // Arm at Cargo Level 1 rocket position.
  Button buttonXboxX = new JoystickButton(RobotMap.GAME_PAD, 3); // Arm at Cargoship position.
  Button buttonXboxY = new JoystickButton(RobotMap.GAME_PAD, 4); // Arm at Storage position.
  Button buttonXboxLB = new JoystickButton(RobotMap.GAME_PAD, 5); // Deploy skis.
  Button buttonXboxRB = new JoystickButton(RobotMap.GAME_PAD, 6); // Retract skis.
  Button buttonXboxStart = new JoystickButton(RobotMap.GAME_PAD, 8); // Depot Arm Position.
  Button buttonXboxRS = new JoystickButton(RobotMap.GAME_PAD, 10); // Climber Toggle (NON-FUNCTIONAL)
  POVButton buttonXboxUp = new POVButton(RobotMap.GAME_PAD, 0); // Hatch mechanism up.
  POVButton buttonXboxDown = new POVButton(RobotMap.GAME_PAD, 180); // Hatch mechanism down.
  POVButton buttonXboxLeft = new POVButton(RobotMap.GAME_PAD, 270); // Extend Hatch mechanism.
  POVButton buttonXboxRight = new POVButton(RobotMap.GAME_PAD, 90); // Retract Hatch mechanism.
  Trigger triggerXboxLeft = new CargoIntakeTrigger(); // Cargo outtake.
  Trigger triggerXboxRight = new CargoOuttakeTrigger(); // Cargo intake.
  Trigger xboxStickLeft = new CargoArmTrigger(); // xbox left joystick is active when moved out of deadzone
  Trigger xboxStickRight = new ClimberTrigger(); // xbox right joystick is active when moved out of deadzone
  Trigger driveStraight = new DriveStraightTrigger(); // left joystick is active when moved out of deadzone
  Trigger topLimSwitch = new TopLimitSwitchTrigger(); // when top limit switch is hit
  Trigger botLimSwitch = new BotLimitSwitchTrigger(); // when bottom limit switch is hit

  public OI() {
    lButton1.whileHeld(new HatchOuttakeCommand(true)); // While held HatchOuttake pushes out.
    lButton1.whenReleased(new HatchOuttakeCommand(false)); // When released HatchOuttake pulls in.
    lButton2.whileHeld(new ReleaseCargoSlowCommand()); // While held Cargo outtakes slowly.
    lButton2.whenReleased(new StopCargoCommand()); // when released, stop the cargo intake
    lButton3.whileHeld(new AutoAlignmentCommand()); // aligns to target using vision and pixy line tracking TODO: link to rpi code
    lButton4.whileHeld(new ReleaseCargoCommand()); // While held cargo outtakes.
    lButton4.whenReleased(new StopCargoCommand()); // when released, stop the cargo intake
    rButton1.whileHeld(new IntakeCargoCommand()); // While held cargo intakes.
    rButton1.whenReleased(new StopCargoCommand()); // when released, stop the cargo intake
    //rButton2.whenPressed(new InvertDriveCommand()); // invert the front of the robot
    rButton3.whenPressed(new HatchIntakeCommand()); // When pressed, change hatch intake position
    rButton5.whenPressed(new DrivetrainPIDToggleCommand());
    lPovLeft.whenPressed(new SwitchToCamera1Command()); // Switch to viewing camera1
    lPovRight.whenPressed(new SwitchToCamera2Command()); // Switch to viewing camera2
    buttonXboxLB.whileHeld(new SkiOutCommand(true)); // When pressed Ski comes out.
    buttonXboxRB.whileHeld(new SkiOutCommand(false)); // When pressed Ski comes in.
    // buttonXboxLB.whenPressed(new SetEncoderCommand());
    // buttonXboxRB.whenPressed(new ResetEncoderCommand());
    topLimSwitch.whileActive(new ResetEncoderBotCommand());
    botLimSwitch.whileActive(new ResetEncoderTopCommand());
    //buttonXboxUp.whenPressed(new HatchPositionCommand(false)); // When pressed Hatch comes up.
    //buttonXboxDown.whenPressed(new HatchPositionCommand(true)); // When pressed hatch goes down.
    buttonXboxLeft.whenPressed(new HatchIntakeXboxCommand(true)); // When pressed hatch intake goes out.
    buttonXboxRight.whenPressed(new HatchIntakeXboxCommand(false)); // When pressed hatch intake comes in.
    triggerXboxLeft.whenActive(new IntakeCargoCommand()); // When held cargo outtakes.
    triggerXboxLeft.whenInactive(new StopCargoCommand()); // when the trigger is inactive, or not held, stop the cargo intake
    triggerXboxRight.whenActive(new ReleaseCargoCommand()); // When held cargo intakes.
    triggerXboxRight.whenInactive(new StopCargoCommand()); // when the trigger is inactive, or not held, stop the cargo intake
    buttonXboxA.whenPressed(new SetCargoArmPosCommand(RobotMap.ARM_BOT_LOC)); // when the A button is clicked, move the cargo arm to the bottom location.
    buttonXboxB.whenPressed(new SetCargoArmPosCommand(RobotMap.ARM_ROCKET_LOC)); // when the B button is clicked, move the cargo arm to the rocket level one location.
    buttonXboxX.whenPressed(new SetCargoArmPosCommand(RobotMap.ARM_CARGO_LOC)); // when the X button is clicked, move the cargo arm to the cargo location.
    buttonXboxY.whenPressed(new SetCargoArmPosCommand(RobotMap.ARM_TOP_LOC)); // when the Y button is clicked, move the cargo arm to the top, or storage, location.
    buttonXboxStart.whenPressed(new SetCargoArmPosCommand(RobotMap.ARM_DEPOT_LOC)); // When the START button is pressed, move the cargo arm to the Depot loction.
    rButton16.whenPressed(new SetCargoArmPosCommand(RobotMap.ARM_ROCKET_2_LOC)); // When the START button is pressed, move the cargo arm to the Depot loction.
    //buttonXboxRS.whenPressed(new ToggleClimberCommand()); // When the right stick is pressed, toggle on/off Climber toggle.
    xboxStickLeft.whileActive(new CargoArmManualCommand()); // move the cargo arm with the xbox left stick
    //xboxStickRight.whileActive(new ClimberManualCommand()); // Have the climber keeping itself up when the climber is not being controlled
    //if (RobotMap.CLIMBER_TOGGLE) xboxStickRight.whenInactive(new PassiveClimberPowerCommand());
    if (RobotMap.DRIVE_PID_TOGGLE) driveStraight.whileActive(new DriveStraightCommand()); // Drive straight using gyro when only the left stick is active
  }
}
