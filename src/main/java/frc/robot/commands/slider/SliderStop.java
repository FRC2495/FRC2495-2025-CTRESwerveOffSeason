
package frc.robot.commands.slider;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.Robot;
import frc.robot.subsystems.Slider;

/**
 *
 */
public class SliderStop extends InstantCommand {

	private Slider slider;

	public SliderStop(Slider slider) {
		this.slider = slider;
		addRequirements(slider);
	}

	// Called once when this command runs
	@Override
	public void initialize() {
		System.out.println("SliderStop: initialize");
		slider.stop();
	}

}
