
package frc.robot.commands.slider;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Slider;

/**
 *
 */
public /*public*/ class SliderExtendWithLimitSwitch extends Command {

	private Slider slider;

	public SliderExtendWithLimitSwitch(Slider slider) {	
		this.slider = slider;
		addRequirements(slider);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("SliderExtendWithLimitSwitch initialize");
		slider.extend();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// nothing
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return slider.getForwardLimitSwitchState();
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("SliderExtendWithLimitSwitch: end");
		slider.stop(); // adjust if needed
	}
}
