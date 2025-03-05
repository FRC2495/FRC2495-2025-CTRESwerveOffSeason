package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class ScoreThirdLevelCoral extends SequentialCommandGroup {

	public ScoreThirdLevelCoral(Elevator elevator, CoralRoller coral_roller) {

		addCommands(

			new ElevatorMoveToThirdLevelWithStallDetection(elevator),

			new CoralRollerTimedRelease(roller, .2),
						
		); 
  
	}
   
}