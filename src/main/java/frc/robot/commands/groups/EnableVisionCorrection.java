
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 *
 */
public class EnableVisionCorrection extends Command {

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

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// nothing
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("EnableVisionCorrection: end");

	}
}
