
// package frc.robot.commands.old_neck;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.Joystick;

// import frc.robot.subsystems.OldNeck;
// import frc.robot.subsystems.SwerveDrivetrain;

// /**
//  *
//  */
// public class OldNeckJoystickControl extends Command {

// 	private OldNeck old_neck;
// 	//private SwerveDrivetrain drivetrain;
// 	private Joystick joystick;

// 	public OldNeckJoystickControl(OldNeck old_neck, SwerveDrivetrain drivetrain, Joystick joystick) {
// 		this.old_neck = old_neck;
// 		//this.drivetrain = drivetrain;
// 		this.joystick = joystick;
		
// 		addRequirements(
// 			old_neck,
// 			drivetrain); // this is needed so that the default drivetrain command does not run at the same time
// 	}

// 	// Called just before this Command runs the first time
// 	@Override
// 	public void initialize() {
// 		System.out.println("OldNeckJoystickControl: initialize");
// 	}

// 	// Called repeatedly when this Command is scheduled to run
// 	@Override
// 	public void execute() {
// 		old_neck.joystickControl(joystick);
// 	}

// 	// Make this return true when this Command no longer needs to run execute()
// 	@Override
// 	public boolean isFinished() {
// 		return false;
// 	}

// 	// Called once after isFinished returns true
// 	@Override
// 	public void end(boolean interrupted) {
// 		System.out.println("OldNeckJoystickControl: end");
// 		old_neck.stop();
// 	}
// }
