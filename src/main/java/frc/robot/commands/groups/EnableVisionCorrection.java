
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 *
 */
public class EnableVisionCorrection extends InstantCommand {

	private RobotContainer container;
	private boolean visionUse;

	public EnableVisionCorrection(RobotContainer container, boolean visionUse) {
		this.container = container;
		this.visionUse = visionUse;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("EnableVisionCorrection: initialize");
		container.changeVisionCorrectionEnablement(visionUse);
	}

	// This instant command can run disabled
	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
