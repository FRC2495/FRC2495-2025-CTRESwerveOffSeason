
package frc.robot.commands.algae_roller;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.AlgaeRoller;

/**
 *
 */
public class AlgaeRollerStop extends InstantCommand {

	private AlgaeRoller algae_roller;

	public AlgaeRollerStop(AlgaeRoller algae_roller) {
		this.algae_roller = algae_roller;
		addRequirements(algae_roller);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("AlgaeRollerStop: initialize");
		algae_roller.stop();
	
	}

}
