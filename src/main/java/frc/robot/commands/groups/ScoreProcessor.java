package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.algae_roller.AlgaeRollerTimedRelease;
import frc.robot.commands.neck.NeckMoveProcessorWithStallDetection;
import frc.robot.commands.slider.SliderExtendWithLimitSwitch;
import frc.robot.subsystems.*;

public class ScoreProcessor extends SequentialCommandGroup {

	public ScoreProcessor(/*Neck neck,*/ AlgaeRoller algae_roller, Neck neck, Slider slider) {

		addCommands(

			new NeckMoveProcessorWithStallDetection(neck),

			new SliderExtendWithLimitSwitch(slider),

			new AlgaeRollerTimedRelease(algae_roller, .1)
						
		); 
  
	}
   
}