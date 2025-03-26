
package frc.robot.commands.coral_roller;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralRoller;

/**
 *
 */
public class CoralRollerForAutoRollOut extends Command {

	private CoralRoller coral_roller;

	public CoralRollerForAutoRollOut(CoralRoller coral_roller) {
		this.coral_roller = coral_roller;
		addRequirements(coral_roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("CoralRollerForAutoRollOut: initialize");
		coral_roller.rollOut();
	}

	@Override
	public boolean isFinished() {
		return !coral_roller.isCoralExitingAuto();
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("CoralRollerForAutoRollOut: end");
		coral_roller.stop();
	}

}
