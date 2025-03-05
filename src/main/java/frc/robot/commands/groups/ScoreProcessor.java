package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class ScoreProcessor extends SequentialCommandGroup {

	public ScoreProcessor(Elevator elevator, AlgaeRoller algae_roller, Slider slide) {

		addCommands(

			//new NeckMoveUpWithStallDetection(neck),

			new AlgaeRollerTimedRelease(algae_roller, .1)
						
		); 
  
	}
   
}