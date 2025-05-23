
package frc.robot.commands.neck;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.Neck;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 *
 */
public class NeckJoystickControl extends Command {

	private Neck neck;
	//private CommandSwerveDrivetrain drivetrain;
	private Joystick joystick;

	public NeckJoystickControl(Neck neck, CommandSwerveDrivetrain drivetrain, Joystick joystick) {
		this.neck = neck;
		this.joystick = joystick;
		
		addRequirements(
			neck,
			drivetrain); // this is needed so that the default drivetrain command does not run at the same time
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("NeckJoystickControl: initialize");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		neck.joystickControl(joystick);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("NeckJoystickControl: end");
		neck.stop();
	}
}
