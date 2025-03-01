
package frc.robot.commands.slider;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import frc.robot.util.*;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Slider;

/**
 *
 */
public class SliderSafeExtendWithStallDetection extends ConditionalCommand {
	
	public SliderSafeExtendWithStallDetection(Neck neck, Slider slider) {
		super(new SliderExtendWithStallDetection(slider), new DoNothing(), new NeckSafetyCheck(neck));
	}

}
