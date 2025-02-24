
package frc.robot.commands.algae_roller;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AlgaeRoller;

/**
 *
 */
public class AlgaeRollerReleaseShortDistance extends Command {

	private AlgaeRoller algae_roller;

	public AlgaeRollerReleaseShortDistance(AlgaeRoller algae_roller) {
		this.algae_roller = algae_roller;
		addRequirements(algae_roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("AlgaeRollerReleaseShortDistance: initialize");
		algae_roller.releaseShortDistance();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// nothing
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return !algae_roller.tripleCheckMove();
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("AlgaeRollerReleaseShortDistance: end");
		algae_roller.stop(); // adjust if needed
	}
}
