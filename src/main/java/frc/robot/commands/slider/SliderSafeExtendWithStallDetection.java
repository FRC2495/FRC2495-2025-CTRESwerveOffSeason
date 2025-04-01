
package frc.robot.commands.slider;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import frc.robot.util.*;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Neck;
import frc.robot.subsystems.Slider;

/**
 *
 */
public class SliderSafeExtendWithStallDetection extends ConditionalCommand {
	
	public SliderSafeExtendWithStallDetection(Slider slider, Elevator elevator) {
		super(new SliderExtendWithLimitSwitch(slider), new DoNothing(), new ElevatorSafetyCheck(elevator));
	}

}
