package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.algae_roller.AlgaeRollerTimedRelease;
import frc.robot.commands.elevator.ElevatorMoveUpWithStallDetection;
import frc.robot.commands.neck.NeckMoveUpWithStallDetection;
import frc.robot.subsystems.*;

public class ScoreNet extends SequentialCommandGroup {

	public ScoreNet(Elevator elevator, AlgaeRoller algae_roller, Slider slider, Neck neck) {

		addCommands(

			new ElevatorMoveUpWithStallDetection(elevator),

			new NeckMoveUpWithStallDetection(neck),

			new AdjustToNet(/*neck,*/ slider),

			new AlgaeRollerTimedRelease(algae_roller, 1.0)
						
		); 
  
	}
   
}