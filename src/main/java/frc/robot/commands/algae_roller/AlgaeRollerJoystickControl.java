
package frc.robot.commands.algae_roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 *
 */
public class AlgaeRollerJoystickControl extends Command {

	private AlgaeRoller algae_roller;
	private Joystick joystick;

	public AlgaeRollerJoystickControl(AlgaeRoller algae_roller, SwerveDrivetrain drivetrain, Joystick joystick) {
		this.algae_roller = algae_roller;
		this.joystick = joystick;
		
		addRequirements(
			algae_roller,
			drivetrain); // this is needed so that the default drivetrain command does not run at the same time
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("AlgaeRollerJoystickControl: initialize");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		algae_roller.joystickControl(joystick);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("AlgaeRollerJoystickControl: end");
		algae_roller.stop();
	}
}
