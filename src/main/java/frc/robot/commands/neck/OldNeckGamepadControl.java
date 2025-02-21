
package frc.robot.commands.neck;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.OldNeck;

/**
 *
 */
public class OldNeckGamepadControl extends Command {

	private OldNeck old_neck;
	private XboxController gamepad;

	public OldNeckGamepadControl(OldNeck old_neck, XboxController gamepad) {
		this.old_neck = old_neck;
		this.gamepad = gamepad;
		
		addRequirements(
			old_neck);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("OldNeckGamepadControl: initialize");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		old_neck.gamepadControl(gamepad);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("NeckGamepadControl: end");
		old_neck.stop();
	}
}
