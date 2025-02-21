
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.OldNeck;

import frc.robot.commands.elevator.*;
import frc.robot.commands.neck.*;

/**
 *
 */
public class ElevatorAndDrawerAndNeckResetEncoders extends SequentialCommandGroup {

	public ElevatorAndDrawerAndNeckResetEncoders(Elevator elevator, OldNeck old_neck) {

		addCommands(
			new ElevatorResetEncoder(elevator),
			//new DrawerResetEncoder(drawer),
			new OldNeckResetEncoder(old_neck));
	} 

	// This instant command can run disabled
	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

}
