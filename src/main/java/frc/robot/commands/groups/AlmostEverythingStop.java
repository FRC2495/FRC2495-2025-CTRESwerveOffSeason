
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Neck;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.commands.coral_roller.*;
import frc.robot.commands.algae_roller.*;
import frc.robot.commands.elevator.*;
//import frc.robot.commands.neck.NeckStop;


/**
 *
 */
public class AlmostEverythingStop extends SequentialCommandGroup {

	public AlmostEverythingStop(Elevator elevator, /*Neck neck,*/ CoralRoller coral_roller, AlgaeRoller algae_roller) {

		addCommands(
			new ElevatorStop(elevator),
			//new DrawerStop(drawer),
			//new NeckStop(neck),
			new CoralRollerStop(coral_roller),
			new AlgaeRollerStop(algae_roller));
			//new ShooterStop(shooter);
	} 
}