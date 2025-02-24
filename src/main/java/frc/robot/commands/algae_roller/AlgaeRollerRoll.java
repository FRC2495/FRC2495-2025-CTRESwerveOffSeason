
package frc.robot.commands.algae_roller;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AlgaeRoller;

/**
 *
 */
public class AlgaeRollerRoll extends Command {

	private AlgaeRoller algae_roller;

	public AlgaeRollerRoll(AlgaeRoller algae_roller) {
		this.algae_roller = algae_roller;
		addRequirements(algae_roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("AlgaeRollerRoll: initialize");
		algae_roller.roll();
	}

}
