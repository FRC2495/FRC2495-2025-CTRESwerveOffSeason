package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coral_roller.CoralRollerForAutoRollOut;
import frc.robot.commands.coral_roller.CoralRollerRollOut;
import frc.robot.commands.coral_roller.CoralRollerTimedRollOut;
import frc.robot.commands.elevator.ElevatorMoveToFourthLevelForAutoWithStallDetection;
import frc.robot.commands.elevator.ElevatorMoveToFourthLevelWithStallDetection;
import frc.robot.commands.neck.NeckMoveHomeWithStallDetection;
import frc.robot.commands.neck.NeckMoveToCoralReefWithStallDetection;
import frc.robot.subsystems.*;

public class ScoreFourthLevelCoral extends SequentialCommandGroup {

	public ScoreFourthLevelCoral(Elevator elevator, CoralRoller coral_roller, Neck neck) {

		addCommands(
			
			//new ElevatorMoveToFourthLevelWithStallDetection(elevator),

			new ElevatorMoveToFourthLevelForAutoWithStallDetection(elevator),

			new NeckMoveHomeWithStallDetection(neck),
			
			new NeckMoveToCoralReefWithStallDetection(neck),

			new ElevatorMoveToFourthLevelForAutoWithStallDetection(elevator),

			new CoralRollerForAutoRollOut(coral_roller)
						
		); 
  
	}
   
}