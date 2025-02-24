
package frc.robot.commands.algae_roller;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AlgaeRoller;

/**
 *
 */
public class AlgaeRollerRollLowRpm extends Command {

	private AlgaeRoller algae_roller;

	public AlgaeRollerRollLowRpm(AlgaeRoller algae_roller) {
		this.algae_roller = algae_roller;
		addRequirements(algae_roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("RollerRollLowRpm: initialize");
		algae_roller.rollLowRpm();
	}

}
