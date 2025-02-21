
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Drawer;
import frc.robot.subsystems.OldNeck;
import frc.robot.subsystems.Roller;
//import frc.robot.subsystems.Shooter;
import frc.robot.commands.elevator.*;
//import frc.robot.commands.drawer.*;
import frc.robot.commands.neck.*;
import frc.robot.commands.roller.*;
//import frc.robot.commands.shooter.ShooterStop;


/**
 *
 */
public class AlmostEverythingStop extends SequentialCommandGroup {

	public AlmostEverythingStop(Elevator elevator, OldNeck old_neck, Roller roller) {

		addCommands(
			new ElevatorStop(elevator),
			//new DrawerStop(drawer),
			new OldNeckStop(old_neck),
			new RollerStop(roller));
			//new ShooterStop(shooter);
	} 
}
