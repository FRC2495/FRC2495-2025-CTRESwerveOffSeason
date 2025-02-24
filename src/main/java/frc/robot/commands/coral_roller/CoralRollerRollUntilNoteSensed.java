
package frc.robot.commands.coral_roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.NoteSensor;
import frc.robot.subsystems.CoralRoller;

/**
 *
 */
public class CoralRollerRollUntilNoteSensed extends Command {

	private CoralRoller coral_roller;
	private NoteSensor notesensor;
	private NoteSensor noteSensorTwo;

	public CoralRollerRollUntilNoteSensed(CoralRoller coral_roller, NoteSensor notesensor, NoteSensor noteSensorTwo) {
		this.coral_roller = coral_roller;
		this.notesensor = notesensor;
		this.noteSensorTwo = noteSensorTwo;
		addRequirements(coral_roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("CoralRollerRollUntilNoteSensed: initialize");
		coral_roller.roll();
	}

	@Override
	public boolean isFinished() {
		return !notesensor.isEnergized() || !noteSensorTwo.isEnergized();
	}

	@Override
	public void end(boolean interupted) {
		System.out.println("CoralRollerRollUntilNoteSensed: end");
		coral_roller.stop();

	}

}
