package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorMoveToAlgaeLevelThreeWithStallDetection;
import frc.robot.subsystems.*;

public class GrabLevelThreeAlgae extends SequentialCommandGroup {

	public GrabLevelThreeAlgae(Elevator elevator, AlgaeRoller algae_roller, Slider slider) {

		addCommands(

			new ElevatorMoveToAlgaeLevelThreeWithStallDetection(elevator),

			//new NeckMoveUpWithStallDetection(neck),

			new PickupAlgae(algae_roller, slider)
						
		); 
  
	}
   
}