
package frc.robot.commands.slider;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Slider;


/**
 *
 */
public class SliderResetEncoder extends InstantCommand {

	private Slider slider;

	public SliderResetEncoder(Slider slider) {
		this.slider = slider;
		addRequirements(slider);
	}

	// This instant command can run disabled
	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("SliderResetEncoder: initialize");
		slider.resetEncoder();
	}
}
