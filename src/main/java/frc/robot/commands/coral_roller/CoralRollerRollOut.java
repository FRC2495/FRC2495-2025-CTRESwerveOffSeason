
package frc.robot.commands.coral_roller;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralRoller;

/**
 *
 */
public class CoralRollerRollOut extends Command {

	private CoralRoller coral_roller;

	public CoralRollerRollOut(CoralRoller coral_roller) {
		this.coral_roller = coral_roller;
		addRequirements(coral_roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("CoralRollerRollOut: initialize");
		coral_roller.roll();
	}

}
