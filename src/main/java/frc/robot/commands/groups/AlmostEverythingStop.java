
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Drawer;
import frc.robot.subsystems.OldNeck;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.commands.coral_roller.*;
import frc.robot.commands.algae_roller.*;
//import frc.robot.subsystems.Shooter;
import frc.robot.commands.elevator.*;
import frc.robot.commands.old_neck.*;


/**
 *
 */
public class AlmostEverythingStop extends SequentialCommandGroup {

	public AlmostEverythingStop(Elevator elevator, OldNeck old_neck, CoralRoller coral_roller, AlgaeRoller algae_roller) {

		addCommands(
			new ElevatorStop(elevator),
			//new DrawerStop(drawer),
			new OldNeckStop(old_neck),
			new CoralRollerStop(coral_roller),
			new AlgaeRollerStop(algae_roller));
			//new ShooterStop(shooter);
	} 
}