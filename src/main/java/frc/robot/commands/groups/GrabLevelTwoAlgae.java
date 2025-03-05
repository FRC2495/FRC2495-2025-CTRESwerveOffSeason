package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorMoveToAlgaeLevelTwoWithStallDetection;
import frc.robot.subsystems.*;

public class GrabLevelTwoAlgae extends SequentialCommandGroup {

	public GrabLevelTwoAlgae(Elevator elevator, AlgaeRoller algae_roller, Slider slider) {

		addCommands(

			new ElevatorMoveToAlgaeLevelTwoWithStallDetection(elevator),

			//new NeckMoveUpWithStallDetection(neck),

			new PickupAlgae(algae_roller, slider)
						
		); 
  
	}
   
}