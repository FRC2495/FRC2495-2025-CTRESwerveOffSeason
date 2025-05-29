
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.commands.drivetrain.*;

/**
 *
 */
public class DrivetrainAndGyroReset extends SequentialCommandGroup {

	public DrivetrainAndGyroReset(CommandSwerveDrivetrain drivetrain) {

		addCommands(
			new DrivetrainResetEncoders(drivetrain),
			new DrivetrainZeroHeading(drivetrain));
	}

	// This instant command can run disabled
	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
