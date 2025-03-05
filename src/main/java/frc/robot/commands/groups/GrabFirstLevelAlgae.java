package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class GrabFirstLevelAlgae extends SequentialCommandGroup {

	public GrabFirstLevelAlgae(Elevator elevator, CoralRoller coral_roller) {

		addCommands(

			new ElevatorMoveToSecondLevelWithStallDetection(elevator),

			new CoralRollerTimedRelease(roller, .2),
						
		); 
  
	}
   
}