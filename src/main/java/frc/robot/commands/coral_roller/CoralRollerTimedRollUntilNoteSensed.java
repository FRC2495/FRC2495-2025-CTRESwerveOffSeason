/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.coral_roller;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.sensors.NoteSensor;
import frc.robot.subsystems.CoralRoller;

/**
 * Add your docs here.
 */
public class CoralRollerTimedRollUntilNoteSensed extends WaitCommand {

	private CoralRoller coral_roller;
	private NoteSensor notesensor;
	private NoteSensor noteSensorTwo;

	/**
	 * Add your docs here.
	 */
	public CoralRollerTimedRollUntilNoteSensed(CoralRoller coral_roller, double timeout, NoteSensor notesensor, NoteSensor noteSensorTwo) {
		super(timeout);
		this.coral_roller = coral_roller;
		this.notesensor = notesensor;
		this.noteSensorTwo = noteSensorTwo;
		addRequirements(coral_roller);
		
		
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
		System.out.println("CoralRollerTimedRollUntilNoteSensed: initialize");
		super.initialize();
		coral_roller.roll();

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
		System.out.println("CoralRollerTimedRollUntilNoteSensed: end");
		
		super.end(interrupted);
	}
}
