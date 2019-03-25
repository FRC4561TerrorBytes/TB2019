/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ClimberManualCommand;

/**
 * @author Karthik, Snehil, Lucas, Max
 */

public class ClimberSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ClimberManualCommand());
  }

  // This method makes the motors on both sides make the robot ascend
  public void set(double power) {
    RobotMap.CLIMBER_MOTOR_GROUP.set(power);
  }

  // Moving the skis onto the hab or back
  public void skiOutPosition(Boolean activate) {
    if (activate) {
      RobotMap.SKI_SOLENOID.set(Value.kForward);
    } else {
      RobotMap.SKI_SOLENOID.set(Value.kReverse);
    }

  }

  // Procides passive power to the climber to keep it from draging on the ground
  public void passivePower() {
    RobotMap.CLIMBER_MOTOR_GROUP.set(0.1);
  }

  // Toggle Climber
  public void toggleClimber() {
		RobotMap.CLIMBER_PASSIVE_TOGGLE = !RobotMap.CLIMBER_PASSIVE_TOGGLE;
	}

  // This method makes the motors on both sides stop.
  public void stop () {
    RobotMap.CLIMBER_MOTOR_GROUP.set(0);
  }
}