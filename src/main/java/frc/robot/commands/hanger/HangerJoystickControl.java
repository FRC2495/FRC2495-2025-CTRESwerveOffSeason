
package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 *
 */
public class HangerJoystickControl extends Command {

	private Hanger hanger;
	//private SwerveDrivetrain drivetrain;
	private Joystick joystick;

	public HangerJoystickControl(Hanger hanger, SwerveDrivetrain drivetrain, Joystick joystick) {
		this.hanger = hanger;
		//this.drivetrain = drivetrain;
		this.joystick = joystick;
		
		addRequirements(
			hanger,
			drivetrain); // this is needed so that the default drivetrain command does not run at the same time
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("HangerJoystickControl: initialize");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		hanger.joystickControl(joystick);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("HangerJoystickControl: end");
		hanger.stop();
	}
}
