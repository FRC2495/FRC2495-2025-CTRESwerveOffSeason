package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class ScoreNet extends SequentialCommandGroup {

	public ScoreNet(Elevator elevator, AlgaeRoller algae_roller, Slider slide) {

		addCommands(

			new ElevatorMoveUpWithStallDetection(elevator),

			new AdjustToNet(/*neck*/, slider)
						
		); 
  
	}
   
}