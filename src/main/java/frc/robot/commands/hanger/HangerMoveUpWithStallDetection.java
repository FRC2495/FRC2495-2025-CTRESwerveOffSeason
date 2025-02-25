
package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Hanger;


/**
 *
 */
public class HangerMoveUpWithStallDetection extends Command {

	private Hanger hanger;

	public HangerMoveUpWithStallDetection(Hanger hanger) {	
		this.hanger = hanger;
		addRequirements(hanger);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("HangerMoveUpWithStallDetection: initialize");
		hanger.moveUp();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// nothing
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return !hanger.tripleCheckMove() || hanger.tripleCheckIfStalled();
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("HangerMoveUpWithStallDetection: end");
		hanger.stop(); // adjust if needed
	}
}
