
package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Elevator;


/**
 *
 */
public class ElevatorMoveToFourthLevelWithStallDetection extends Command {

	private Elevator elevator;

	public ElevatorMoveToFourthLevelWithStallDetection(Elevator elevator) {	
		this.elevator = elevator;
		addRequirements(elevator);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("ElevatorMoveToFourthLevelWithStallDetection: initialize");
		elevator.moveToFourthLevel();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// nothing
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return elevator.getForwardLimitSwitchState() || !elevator.tripleCheckMove() || elevator.tripleCheckIfStalled();
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("ElevatorMoveToFourthLevelWithStallDetection: end");
		elevator.stay(); // adjust if needed
	}
}
