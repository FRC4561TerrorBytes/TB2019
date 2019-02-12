/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;

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
    Button lButton1 = new JoystickButton(RobotMap.LEFT_STICK, 1);     // (real: Hatch outtake) Placeholder for Pixy line tracking.
    Button lButton2 = new JoystickButton(RobotMap.LEFT_STICK, 2);     // (real: Cargo outtake (slow)) Placeholder for HatchOuttake.
    Button lButton3 = new JoystickButton(RobotMap.LEFT_STICK, 3);     // (real: Auto Align)
    Button lButton4 = new JoystickButton(RobotMap.LEFT_STICK, 4);     // (real: Cargo outtake (fast)) Placeholder for hatchPosition down.
    Button lButton5 = new JoystickButton(RobotMap.LEFT_STICK, 5);     // (real: N/A) Placeholder for hatchPosition up.
    Button rButton1 = new JoystickButton(RobotMap.RIGHT_STICK, 1);    // Cargo intake in.
    Button rButton2 = new JoystickButton(RobotMap.RIGHT_STICK, 2);    // Hatch extend/retract.
    Button buttonXboxA = new JoystickButton(RobotMap.GAME_PAD, 1);    // TODO: chage to Arm at Cargo ground/intake posistion.
    Button buttonXboxB = new JoystickButton(RobotMap.GAME_PAD, 2);    // Arm at Cargo Level 1 rocket position.
    Button buttonXboxX = new JoystickButton(RobotMap.GAME_PAD, 3);    // Arm at Cargoship position.
    Button buttonXboxY = new JoystickButton(RobotMap.GAME_PAD, 4);    // Arm at Storage position.
    Button buttonXboxLB = new JoystickButton(RobotMap.GAME_PAD, 5);   // Deploy skis.
    Button buttonXboxRB = new JoystickButton(RobotMap.GAME_PAD, 6);   // Retract skis.
    POVButton buttonXboxUp = new POVButton(RobotMap.GAME_PAD, 0);     // Xbox d-pad up 
    POVButton buttonXboxDown = new POVButton(RobotMap.GAME_PAD, 180); // Xbox d-pad down
    POVButton buttonXboxLeft = new POVButton(RobotMap.GAME_PAD, 270); // Xbox d-pad left
    POVButton buttonXboxRight = new POVButton(RobotMap.GAME_PAD, 90); // Xbox d-pad right

    if (RobotMap.PIXY_DRIVE_TOGGLE) {
      lButton1.whileHeld(new DriveStraightPixyInputCommand());
    }
    else {
      lButton1.whileHeld(new DriveStraightCommand());
    }

    // If Left trigger is held, release cargo.
    if (RobotMap.GAME_PAD.getTriggerAxis(Hand.kLeft) > 0.25) {
      new ReleaseCargoCommand();
      SmartDashboard.putString("Trigger:", "Left");
    } 

    // If right trigger is held, intake cargo.
    if (RobotMap.GAME_PAD.getTriggerAxis(Hand.kRight) > 0.25) {
      new IntakeCargoCommand();
      SmartDashboard.putString("Trigger:", "Right");
    }

    lButton2.whileHeld(new HatchOuttakeCommand(true)); // (placeholder) When held HatchOuttake pushes out.
    lButton2.whenReleased(new HatchOuttakeCommand(false)); // (placeholder) When released HatchOuttake pulls in.
    lButton4.whenPressed(new HatchPositionCommand(false)); // When pressed Hatchintake goes down.
    lButton5.whenPressed(new HatchPositionCommand(true)); // (placeholder) When pressed Hatchintake goes up.
    rButton1.whileHeld(new IntakeCargoCommand()); // When held cargo intakes.
    rButton2.whileHeld(new HatchIntakeCommand(true)); // (placeholder) When held Hatchintake pushes out.
    rButton2.whenReleased(new HatchIntakeCommand(false)); // When released Hatchintake pulls in.
    buttonXboxLB.whileHeld(new SkiOutCommand(true)); // When pressed Ski comes out.
    buttonXboxRB.whileHeld(new SkiOutCommand(false)); // When pressed Ski comes in.
    buttonXboxA.whenPressed(new HatchPresentCommand()); // (placeholder) When pressed, check if Super Suit (hatch) is present
    buttonXboxUp.whenPressed(new HatchPositionCommand(false));
    buttonXboxDown.whenPressed(new HatchPositionCommand(true));
    buttonXboxLeft.whenPressed(new HatchPositionCommand(true));
    buttonXboxRight.whenPressed(new HatchPositionCommand(false));
  }
}
