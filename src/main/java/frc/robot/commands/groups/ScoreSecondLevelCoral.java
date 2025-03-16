package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coral_roller.CoralRollerTimedRollOut;
import frc.robot.commands.elevator.ElevatorMoveToSecondLevelWithStallDetection;
import frc.robot.commands.neck.NeckMoveHomeWithStallDetection;
import frc.robot.commands.neck.NeckMoveToCoralReefWithStallDetection;
import frc.robot.subsystems.*;

public class ScoreSecondLevelCoral extends SequentialCommandGroup {

	public ScoreSecondLevelCoral(Elevator elevator, CoralRoller coral_roller, Neck neck) {

		addCommands(

			new ElevatorMoveToSecondLevelWithStallDetection(elevator),

			new NeckMoveHomeWithStallDetection(neck),

			new NeckMoveToCoralReefWithStallDetection(neck),

			new ElevatorMoveToSecondLevelWithStallDetection(elevator),

			new CoralRollerTimedRollOut(coral_roller, .4)
						
		); 
  
	}
   
}