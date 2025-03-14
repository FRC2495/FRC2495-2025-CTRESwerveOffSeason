package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coral_roller.CoralRollerTimedRelease;
import frc.robot.commands.elevator.ElevatorMoveToFirstLevelWithStallDetection;
import frc.robot.subsystems.*;

public class ScoreFirstLevelCoral extends SequentialCommandGroup {

	public ScoreFirstLevelCoral(Elevator elevator, CoralRoller coral_roller, Neck neck) {

		addCommands(

			new ElevatorMoveToFirstLevelWithStallDetection(elevator),

			new CoralRollerTimedRelease(coral_roller, .2)
		); 
  
	}
   
}