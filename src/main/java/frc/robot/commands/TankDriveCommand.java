package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
/**
 * @author Snehil
 */
public class TankDriveCommand extends Command {

	public TankDriveCommand(){
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}
	
	// Called repeatedly when this Command is scheduled to run
	protected void execute(){
		Robot.drivetrain.tankDrive();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}
	
	// Called once after isFinished returns true
	protected void stop(){
		Robot.drivetrain.stop();
	}

	
}
