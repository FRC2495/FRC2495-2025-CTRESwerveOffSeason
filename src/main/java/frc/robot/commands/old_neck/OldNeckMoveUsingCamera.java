package frc.robot.commands.old_neck;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.OldNeck;
import frc.robot.util.Magic;
import frc.robot.interfaces.ICamera;

/**
 *
 */
public class OldNeckMoveUsingCamera extends Command {

	private OldNeck old_neck;
	private ICamera camera;

	public OldNeckMoveUsingCamera(OldNeck old_neck_in, ICamera camera_in) {

		this.old_neck = old_neck_in;
		this.camera = camera_in;

		addRequirements(
			old_neck);

	}

	@Override
	public void initialize() {
		System.out.println("OldNeckMoveUsingCamera: initialize");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		double distance = camera!=null? camera.getDistanceToTarget():0.0; // call will return 0.0 by convention if no target acquired

		if (distance != 0.0) // only moves old_neck if target is acquired - noop otherwise
		{
			double magic_encoder_ticks = Magic.getEncoderCounts(distance);

			old_neck.moveCustom(magic_encoder_ticks);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("OldNeckMoveUsingCamera: end");
		old_neck.stay(); // stop()
	}
}
