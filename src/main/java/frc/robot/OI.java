/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

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

  public OI() {
    Button lButton1 = new JoystickButton(RobotMap.LEFT_STICK, 1);     // Hatch outtake
    Button lButton2 = new JoystickButton(RobotMap.LEFT_STICK, 2);     // Cargo outtake (slow)
    Button lButton3 = new JoystickButton(RobotMap.LEFT_STICK, 3);     // Auto Align
    Button lButton4 = new JoystickButton(RobotMap.LEFT_STICK, 4);     // Cargo outtake (fast)
    Button rButton1 = new JoystickButton(RobotMap.RIGHT_STICK, 1);    // Cargo intake in.
    Button rButton2 = new JoystickButton(RobotMap.RIGHT_STICK, 2);
    Button rButton3 = new JoystickButton(RobotMap.RIGHT_STICK, 3);    // Hatch extend/retract.
    Button rButton4 = new JoystickButton(RobotMap.RIGHT_STICK, 4);
    Button rButton5 = new JoystickButton(RobotMap.RIGHT_STICK, 5);
    Button buttonXboxA = new JoystickButton(RobotMap.GAME_PAD, 1);    // TODO: chage to Arm at Cargo ground/intake posistion.
    Button buttonXboxB = new JoystickButton(RobotMap.GAME_PAD, 2);    // Arm at Cargo Level 1 rocket position.
    Button buttonXboxX = new JoystickButton(RobotMap.GAME_PAD, 3);    // Arm at Cargoship position.
    Button buttonXboxY = new JoystickButton(RobotMap.GAME_PAD, 4);    // Arm at Storage position.
    Button buttonXboxLB = new JoystickButton(RobotMap.GAME_PAD, 5);   // Deploy skis.
    Button buttonXboxRB = new JoystickButton(RobotMap.GAME_PAD, 6);   // Retract skis.
    POVButton buttonXboxUp = new POVButton(RobotMap.GAME_PAD, 0);     // Hatch mechanism up.
    POVButton buttonXboxDown = new POVButton(RobotMap.GAME_PAD, 180); // Hatch mechanism down.
    POVButton buttonXboxLeft = new POVButton(RobotMap.GAME_PAD, 270); // Extend Hatch mechanism.
    POVButton buttonXboxRight = new POVButton(RobotMap.GAME_PAD, 90); // Retract Hatch mechanism.
    Trigger triggerXboxLeft = new CargoIntakeTrigger(); // Cargo outtake.
    Trigger triggerXboxRight = new CargoOuttakeTrigger(); // Cargo intake.

    if (RobotMap.PIXY_DRIVE_TOGGLE) {
      lButton3.whileHeld(new DriveStraightPixyInputCommand());
    }
    else {
      lButton3.whileHeld(new DriveStraightCommand());
    }

    lButton1.whileHeld(new HatchOuttakeCommand(true)); // While held HatchOuttake pushes out.
    lButton1.whenReleased(new HatchOuttakeCommand(false)); // When released HatchOuttake pulls in.
    lButton2.whileHeld(new ReleaseCargoSlowCommand()); // While held Cargo outtakes slowly.
    lButton2.whenReleased(new StopCargoCommand());
    lButton4.whileHeld(new ReleaseCargoCommand()); // While held cargo outtakes.
    lButton4.whenReleased(new StopCargoCommand());
    rButton1.whileHeld(new IntakeCargoCommand()); // While held cargo intakes.
    rButton1.whenReleased(new StopCargoCommand());
    rButton2.whenPressed(new InvertDriveCommand());
    rButton3.whileHeld(new HatchIntakeCommand(true)); // While held Hatchintake pushes out.
    rButton3.whenReleased(new HatchIntakeCommand(false)); // When released Hatchintake pulls in.
    rButton4.whenPressed(new SwitchToCamera1Command()); // Switch to viewing camera1
    rButton5.whenPressed(new SwitchToCamera2Command()); // Switch to viewing camera1
    buttonXboxLB.whileHeld(new SkiOutCommand(true)); // When pressed Ski comes out.
    buttonXboxRB.whileHeld(new SkiOutCommand(false)); // When pressed Ski comes in.
    buttonXboxA.whenPressed(new HatchPresentCommand()); // (placeholder) When pressed, check if Super Suit (hatch) is present
    buttonXboxUp.whenPressed(new HatchPositionCommand(false)); // When pressed Hatch comes up.
    buttonXboxDown.whenPressed(new HatchPositionCommand(true)); // When pressed hatch goes down.
    buttonXboxLeft.whenPressed(new HatchIntakeCommand(true)); // When pressed hatch intake goes out.
    buttonXboxRight.whenPressed(new HatchIntakeCommand(false)); // When pressed hatch intake comes in.
    triggerXboxLeft.whenActive(new IntakeCargoCommand()); // When held cargo outtakes.
    triggerXboxLeft.whenInactive(new StopCargoCommand());
    triggerXboxRight.whenActive(new ReleaseCargoCommand()); // When held cargo intakes.
    triggerXboxRight.whenInactive(new StopCargoCommand());
    buttonXboxB.whileHeld(new IntakeCargoCommand());
    buttonXboxB.whenReleased(new IntakeCargoCommand());
  }
}
