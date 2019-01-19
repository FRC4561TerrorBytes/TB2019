/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class CargoIntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX cargoIntakeLeftMotor;
	private WPI_TalonSRX cargoIntakeRightMotor; 

  public CargoIntakeSubsystem(){
    this.cargoIntakeLeftMotor = new WPI_TalonSRX((Integer) null); //TODO: find real values
    this.cargoIntakeRightMotor = new WPI_TalonSRX((Integer) null);
  }

  @Override
  public void initDefaultCommand() {
  }

  public void intakeCargo(){
    cargoIntakeLeftMotor.set(1);
    cargoIntakeRightMotor.set(1);
  }

  public void outtakeCargo(){
    cargoIntakeLeftMotor.set(-1);
    cargoIntakeRightMotor.set(-1);
  }

  public void stop(){
    cargoIntakeLeftMotor.set(0);
    cargoIntakeRightMotor.set(0);
  }
}
