
package frc.robot.commands.slider;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Robot;
import frc.robot.subsystems.Slider;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 *
 */
public class SliderJoystickControl extends Command {

	private Slider slider;
	private Joystick joystick;

	public SliderJoystickControl(Slider slider, CommandSwerveDrivetrain drivetrain, Joystick joystick) {
		this.slider = slider;
		this.joystick = joystick;
		addRequirements(slider, drivetrain); // this is needed so that the default drivetrain command does not run at the same time
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		System.out.println("SliderJoystickControl: initialize");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		slider.joystickControl(joystick);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		System.out.println("SliderJoystickControl: end");
		slider.stop();
	}
}
