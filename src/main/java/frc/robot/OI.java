/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
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
    Button button1 = new JoystickButton(RobotMap.LEFT_STICK, 1); // Placeholder for hatchOuttake.
    Button button2 = new JoystickButton(RobotMap.LEFT_STICK, 2); // Placeholder for driveForward.
    Button button3 = new JoystickButton(RobotMap.LEFT_STICK, 3); // Placeholder for hatchIntake.
    Button button4 = new JoystickButton(RobotMap.LEFT_STICK, 4); // Placeholder for hatchPosition down.
    Button button5 = new JoystickButton(RobotMap.LEFT_STICK, 5); // Placeholder for hatchPosition up.
    Button button6 = new JoystickButton(RobotMap.LEFT_STICK, 6); // Placeholder for skiOutPosition.
    Button button7 = new JoystickButton(RobotMap.LEFT_STICK, 7); // 
    Button button8 = new JoystickButton(RobotMap.LEFT_STICK, 8); // 
    Button buttonXboxA = new JoystickButton(RobotMap.GAME_PAD, 1);
    
    button1.whileHeld(new HatchOuttakeCommand(true)); // (placeholder) When held HatchOuttake pushes out.
    button1.whenReleased(new HatchOuttakeCommand(false)); // (placeholder) When released HatchOuttake pulls in.
    if(RobotMap.PIXY_DRIVE_TOGGLE){
      button2.whileHeld(new DriveStraightPixyInputCommand());
    }
    else{
      button2.whileHeld(new DriveStraightCommand());
    }
    button3.whileHeld(new HatchIntakeCommand(true)); // (placeholder) When held Hatchintake pushes out.
    button3.whenReleased(new HatchIntakeCommand(false)); // (placeholder) When released Hatchintake pulls in.
    button4.whenPressed(new HatchPositionCommand(false)); // (placeholder) When pressed Hatchintake goes down.
    button5.whenPressed(new HatchPositionCommand(true)); // (placeholder) When pressed Hatchintake goes up.
    button6.whileHeld(new SkiOutCommand(true)); // (placeholder) When pressed Ski comes out.
    button6.whenReleased(new SkiOutCommand(false)); // (placeholder) When released Ski comes in.
    button7.whenReleased(new SkiOutCommand(false)); // (placeholder) When released Ski comes in.
    buttonXboxA.whenPressed(new HatchPresentCommand()); // (placeholder) When pressed, check if Super Suit (hatch) is present
  }
}
