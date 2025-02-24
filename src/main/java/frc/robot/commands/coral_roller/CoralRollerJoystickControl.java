
package frc.robot.commands.coral_roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 *
 */
public class CoralRollerJoystickControl extends Command {

	private CoralRoller coral_roller;
	//private SwerveDrivetrain drivetrain;
	private Joystick joystick;

	public CoralRollerJoystickControl(CoralRoller roller, SwerveDrivetrain drivetrain, Joystick joystick) {
		this.coral_roller = coral_roller;
		//this.drivetrain = drivetrain;
		this.joystick = joystick;
		
		addRequirements(
			roller,
			drivetrain); // this is needed so that the default drivetrain command does not run at the same time
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("RollerJoystickControl: initialize");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		coral_roller.joystickControl(joystick);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("CoralRollerJoystickControl: end");
		coral_roller.stop();
	}
}
