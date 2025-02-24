
package frc.robot.commands.coral_roller;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.CoralRoller;

/**
 *
 */
public class CoralRollerStop extends InstantCommand {

	private CoralRoller coral_roller;

	public CoralRollerStop(CoralRoller coral_roller) {
		this.coral_roller = coral_roller;
		addRequirements(coral_roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("CoralRollerStop: initialize");
		coral_roller.stop();
	
	}

}
