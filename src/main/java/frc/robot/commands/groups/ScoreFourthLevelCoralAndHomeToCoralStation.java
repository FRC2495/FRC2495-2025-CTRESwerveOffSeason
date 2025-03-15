package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coral_roller.CoralRollerTimedRelease;
import frc.robot.commands.elevator.ElevatorMoveDownWithStallDetection;
import frc.robot.commands.elevator.ElevatorMoveToFourthLevelWithStallDetection;
import frc.robot.commands.neck.NeckMoveToCoralReefWithStallDetection;
import frc.robot.commands.neck.NeckMoveToCoralStationWithStallDetection;
import frc.robot.commands.slider.SliderRetractWithLimitSwitch;
import frc.robot.subsystems.*;

public class ScoreFourthLevelCoralAndHomeToCoralStation extends SequentialCommandGroup {

	public ScoreFourthLevelCoralAndHomeToCoralStation(Elevator elevator, CoralRoller coral_roller, Neck neck, Slider slider) {

		addCommands(
			
			new ScoreFourthLevelCoral(elevator, coral_roller, neck),

			new HomeToCoralStation(elevator, /*slider,*/ neck)	
						
		); 
  
	}
   
}