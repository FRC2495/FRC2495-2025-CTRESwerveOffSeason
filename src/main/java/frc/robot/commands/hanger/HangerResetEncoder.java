package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Hanger;

/**
 *
 */
public class HangerResetEncoder extends InstantCommand {

    private Hanger hanger;

	public HangerResetEncoder(Hanger hanger) {
        this.hanger = hanger;
		addRequirements(hanger);
	}

	// This instant command can run disabled
	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("HangerResetEncoder: initialize");
		hanger.resetEncoder();
	}

}