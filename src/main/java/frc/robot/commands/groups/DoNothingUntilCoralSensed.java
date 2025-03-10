
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DoNothing;
import frc.robot.sensors.CoralSensor;
import frc.robot.sensors.NoteSensor;
import frc.robot.subsystems.CoralRoller;

/**
 *
 */
public class DoNothingUntilCoralSensed extends Command {

	private CoralRoller roller;
	private CoralSensor coral_sensor;


	public DoNothingUntilCoralSensed(CoralRoller roller, CoralSensor coral_sensor) {
		this.roller = roller;
		this.coral_sensor = coral_sensor;
		addRequirements(roller);
	}


	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("DoNothingUntilCoralSensed: initialize");
	}

	@Override
	public boolean isFinished() {
		return coral_sensor.isTriggered();
		//return !notesensor.isEnergized() || !noteSensorTwo.isEnergized();
	}

	@Override
	public void end(boolean interupted) {
		System.out.println("DoNothingUntilCoralSensed: end");
	}

}
