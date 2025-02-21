package frc.robot.commands.neck;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.OldNeck;

/**
 *
 */
public class OldNeckResetEncoder extends InstantCommand {

    private OldNeck old_neck;

	public OldNeckResetEncoder(OldNeck old_neck) {
        this.old_neck = old_neck;
		addRequirements(old_neck);
	}

	// This instant command can run disabled
	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("OldNeckResetEncoder: initialize");
		old_neck.resetEncoder();
	}

}