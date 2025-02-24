
package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Hanger;

/**
 *
 */
public class HangerStop extends InstantCommand {

    private Hanger hanger;
	public HangerStop(Hanger hanger) {
        this.hanger = hanger;
		addRequirements(hanger);
	} 

	// Called once when this command runs
	@Override
	public void initialize() {
		System.out.println("HangerStop: initialize");
		hanger.stop();
	}

}