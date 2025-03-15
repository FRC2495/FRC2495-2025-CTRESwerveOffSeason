package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coral_roller.CoralRollerTimedRelease;
import frc.robot.commands.elevator.ElevatorMoveToFourthLevelWithStallDetection;
import frc.robot.commands.neck.NeckMoveToCoralReefWithStallDetection;
import frc.robot.subsystems.*;

public class ScoreFourthLevelCoral extends SequentialCommandGroup {

	public ScoreFourthLevelCoral(Elevator elevator, CoralRoller coral_roller, Neck neck) {

		addCommands(
			
			new ElevatorMoveToFourthLevelWithStallDetection(elevator),
			
			new NeckMoveToCoralReefWithStallDetection(neck),

			new ElevatorMoveToFourthLevelWithStallDetection(elevator),

			new CoralRollerTimedRelease(coral_roller, .4)
						
		); 
  
	}
   
}