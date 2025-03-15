
package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Hanger;

/**
 *
 */
public class HangerButtonBoxDownControl extends Command {

	private Hanger hanger;

	public HangerButtonBoxDownControl(Hanger hanger) {
		this.hanger = hanger;
		
		addRequirements(
			hanger);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("HangerButtonBoxDownControl: initialize");
		hanger.buttonBoxDownControl();
	}
}
