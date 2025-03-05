package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class ScoreFirstLevelCoral extends SequentialCommandGroup {

	public ScoreFirstLevelCoral(Elevator elevator, CoralRoller coral_roller) {

		addCommands(

			new ElevatorMoveToFirstLevelWithStallDetection(elevator),

			new CoralRollerTimedRelease(roller, .2),
						
		); 
  
	}
   
}