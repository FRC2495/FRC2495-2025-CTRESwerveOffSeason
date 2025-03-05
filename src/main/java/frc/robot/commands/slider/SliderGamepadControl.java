
package frc.robot.commands.slider;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Slider;

/**
 *
 */
public class SliderGamepadControl extends Command {

	private Slider slider;
	private XboxController gamepad;

	public SliderGamepadControl(Slider slider, XboxController gamepad) {
		this.slider = slider;
		this.gamepad = gamepad;
		addRequirements(slider);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("SliderGamepadControl: initialize");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		slider.gamepadControl(gamepad);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("SliderGamepadControl: end");
		slider.stop();
	}
}
