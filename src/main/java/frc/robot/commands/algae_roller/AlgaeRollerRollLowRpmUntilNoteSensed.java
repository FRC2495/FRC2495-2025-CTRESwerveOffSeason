
package frc.robot.commands.algae_roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sensors.NoteSensor;
import frc.robot.subsystems.AlgaeRoller;

/**
 *
 */
public class AlgaeRollerRollLowRpmUntilNoteSensed extends Command {

	private AlgaeRoller algae_roller;
	private NoteSensor notesensor;
	private NoteSensor noteSensorTwo;

	public AlgaeRollerRollLowRpmUntilNoteSensed(AlgaeRoller algae_roller, NoteSensor notesensor, NoteSensor noteSensorTwo) {
		this.algae_roller = algae_roller;
		this.notesensor = notesensor;
		this.noteSensorTwo = noteSensorTwo;
		addRequirements(algae_roller);
	}


	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("AlgaeRollerRollLowRpmUntilNoteSensed: initialize");
		algae_roller.rollLowRpm();
	}

	@Override
	public boolean isFinished() {
		return !notesensor.isEnergized();
		//return !notesensor.isEnergized() || !noteSensorTwo.isEnergized();
	}

	@Override
	public void end(boolean interupted) {
		System.out.println("AlgaeRollerRollLowRpmUntilNoteSensed: end");
		algae_roller.stop();

	}

}
