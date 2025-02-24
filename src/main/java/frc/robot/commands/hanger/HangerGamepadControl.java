
package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Hanger;

/**
 *
 */
public class HangerGamepadControl extends Command {

	private Hanger hanger;
	private XboxController gamepad;

	public HangerGamepadControl(Hanger hanger, XboxController gamepad) {
		this.hanger = hanger;
		this.gamepad = gamepad;
		
		addRequirements(
			hanger);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("HangerGamepadControl: initialize");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		hanger.gamepadControl(gamepad);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("HangerGamepadControl: end");
		hanger.stop();
	}
}
