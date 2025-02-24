/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.algae_roller;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.sensors.NoteSensor;
import frc.robot.subsystems.AlgaeRoller;

/**
 * Add your docs here.
 */
public class AlgaeRollerTimedRollUntilNoteSensed extends WaitCommand {

	private AlgaeRoller algae_roller;
	private NoteSensor notesensor;
	private NoteSensor noteSensorTwo;

	/**
	 * Add your docs here.
	 */
	public AlgaeRollerTimedRollUntilNoteSensed(AlgaeRoller algae_roller, double timeout, NoteSensor notesensor, NoteSensor noteSensorTwo) {
		super(timeout);
		this.algae_roller = algae_roller;
		this.notesensor = notesensor;
		this.noteSensorTwo = noteSensorTwo;
		addRequirements(algae_roller);
		
		
		// ControllerBase is not a real subsystem, so no need to reserve it
	}

	// This instant command can run disabled
	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("AlgaeRollerTimedRollUntilNoteSensed: initialize");
		super.initialize();
		algae_roller.roll();

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// nothing
	}

	@Override
	public boolean isFinished() {
		return !notesensor.isEnergized() || !noteSensorTwo.isEnergized();
	}

	// Called once after timeout
	@Override
	public void end(boolean interrupted) {
		System.out.println("AlgaeRollerTimedRollUntilNoteSensed: end");
		
		super.end(interrupted);
	}
}
