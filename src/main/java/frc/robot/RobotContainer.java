// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import java.util.List;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;

import frc.robot.sensors.*;

/*import frc.robot.interfaces.IElevator;
import frc.robot.interfaces.IDrawer;
import frc.robot.interfaces.INeck;
import frc.robot.interfaces.IRoller;*/

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Slider;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.Indicator;
//import frc.robot.subsystems.Neck;
import frc.robot.commands.algae_roller.*;
//import frc.robot.subsystems.SimpleShooter;
import frc.robot.commands.coral_roller.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.slider.*;
//import frc.robot.commands.neck.*;
import frc.robot.commands.elevator.*;
import frc.robot.commands.hanger.*;
//import frc.robot.commands.simpleshooter.*;
//import frc.robot.commands.shooter.*;
import frc.robot.interfaces.ICamera;
//import frc.robot.commands.mouth.*;
import frc.robot.commands.indicator.*;
import frc.robot.commands.groups.*;
//import frc.robot.commands.gamepad.*;
import frc.robot.auton.*;
import frc.robot.auton.trajectories.*;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	public static final double GAMEPAD_AXIS_THRESHOLD = 0.15;
	public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;

	public static final int LX = 0;
	public static final int LY = 1;
	public static final int LT = 2;
	public static final int RT = 3;
	public static final int RX = 4;
	public static final int RY = 5;

	Command indicatorTimedScrollRainbow; // command to run while stating up and when disabled

	// choosers (for auton)

	private final SendableChooser<Command> autoChooser;

	// sensors

	private final HMAccelerometer accelerometer = new HMAccelerometer();

	private final ICamera object_detection_camera = new ObjectDetectionCamera();

	private final ICamera apriltag_camera = new AprilTagCamera();

	//private final CoralSensor firstCoralSensor = new CoralSensor(Ports.Digital.CORAL_SENSOR, 5.0);

	//private final CoralSensor secondCoralSensor = new CoralSensor(Ports.Digital.CORAL_SENSOR_TWO, 5.0);

	//private final NoteSensor noteSensor = new NoteSensor(Ports.Digital.NOTE_SENSOR);

	// motorized devices

	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(apriltag_camera);

	/*private final WPI_TalonSRX drawer_master = new WPI_TalonSRX(Ports.CAN.DRAWER);

	private final Drawer drawer = new Drawer(drawer_master);*/

	private final TalonFX elevator_master = new TalonFX(Ports.CAN.ELEVATOR_MASTER);
	private final TalonFX elevator_follower = new TalonFX(Ports.CAN.ELEVATOR_FOLLOWER);

	private final /*I*/Elevator elevator = new Elevator(elevator_master, elevator_follower);
	
	//private final /*I*/Neck neck = new Neck();

	//private final TalonFX old_neck_master = new TalonFX(Ports.CAN.OLD_NECK_MASTER);
	//private final TalonFX old_neck_follower = new TalonFX(Ports.CAN.OLD_NECK_FOLLOWER);
	
	//private final /*I*/OldNeck old_neck = new OldNeck(old_neck_master, old_neck_follower);

	//private final CANSparkMax roller_master = new CANSparkMax(Ports.CAN.ROLLER, MotorType.kBrushless);
	private final WPI_TalonSRX coral_roller_master = new WPI_TalonSRX(Ports.CAN.ROLLER_MASTER);
	private final WPI_TalonSRX coral_roller_follower = new WPI_TalonSRX(Ports.CAN.ROLLER_FOLLOWER);
	private final WPI_TalonSRX algae_roller_master = new WPI_TalonSRX(Ports.CAN.ROLLER_MASTER);
	private final WPI_TalonSRX algae_roller_follower = new WPI_TalonSRX(Ports.CAN.ROLLER_FOLLOWER);

	private final /*I*/CoralRoller coral_roller = new CoralRoller(coral_roller_master, coral_roller_follower);
	private final /*I*/AlgaeRoller algae_roller = new AlgaeRoller(algae_roller_master, algae_roller_follower);

	private final WPI_TalonSRX slider_master = new WPI_TalonSRX(Ports.CAN.SLIDER_MASTER);
	private final Slider slider = new Slider(slider_master);

	private final TalonFX hanger_master = new TalonFX(Ports.CAN.HANGER_MASTER);
	private final Hanger hanger = new Hanger(hanger_master);

	// pneumatic devices

	//private final Compressor compressor = new Compressor();

	//private final Mouth mouth = new Mouth();

	// misc

	private final Field2d field = new Field2d(); //  a representation of the field

	private final Indicator indicator = new Indicator(apriltag_camera, object_detection_camera);

	public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

	// The driver's and copilot's joystick(s) and controller(s)
	CommandJoystick joyMain = new CommandJoystick(Ports.USB.MAIN_JOYSTICK);
	//CommandXboxController driverGamepad = new CommandXboxController(Ports.USB.DRIVER_GAMEPAD);
	CommandXboxController copilotGamepad = new CommandXboxController(Ports.USB.COPILOT_GAMEPAD);
	CommandJoystick buttonBox = new CommandJoystick(Ports.USB.BUTTON_BOX);
	
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		NamedCommands.registerCommand("elevatorMoveToFourthLevelWithStallDetection", new ElevatorMoveToFourthLevelWithStallDetection(elevator));
        NamedCommands.registerCommand("elevatorMoveToThirdLevelWithStallDetection", new ElevatorMoveToThirdLevelWithStallDetection(elevator));
        NamedCommands.registerCommand("elevatorMoveToSecondLevelWithStallDetection", new ElevatorMoveToSecondLevelWithStallDetection(elevator));
        NamedCommands.registerCommand("elevatorMoveToFirstLevelWithStallDetection", new ElevatorMoveToFirstLevelWithStallDetection(elevator));
        NamedCommands.registerCommand("elevatorMoveUpWithStallDetection", new ElevatorMoveUpWithStallDetection(elevator));
        NamedCommands.registerCommand("elevatorMoveDownWithStallDetection", new ElevatorMoveDownWithStallDetection(elevator));
		NamedCommands.registerCommand("elevatorMoveToAlgaeLevelTwoWithStallDetection", new ElevatorMoveToAlgaeLevelTwoWithStallDetection(elevator));
		NamedCommands.registerCommand("elevatorMoveToAlgaeLevelThreeWithStallDetection", new ElevatorMoveToAlgaeLevelThreeWithStallDetection(elevator));
        NamedCommands.registerCommand("algaeRollerRoll", new AlgaeRollerRoll(algae_roller));
        NamedCommands.registerCommand("algaeRollerRelease", new AlgaeRollerRelease(algae_roller));
        NamedCommands.registerCommand("algaeRollerTimedRoll", new AlgaeRollerTimedRoll(algae_roller, 5));
        NamedCommands.registerCommand("algaeRollerTimedRelease", new AlgaeRollerTimedRelease(algae_roller, 5));
        NamedCommands.registerCommand("coralRollerRoll", new CoralRollerRollOut(coral_roller));
        NamedCommands.registerCommand("coralRollerRelease", new CoralRollerRollIn(coral_roller));
        NamedCommands.registerCommand("coralRollerTimedRoll", new CoralRollerTimedRoll(coral_roller, 5));
        NamedCommands.registerCommand("coralRollerTimedRelease", new CoralRollerTimedRelease(coral_roller, 5));
		//NamedCommands.registerCommand("sliderExtendWithStallDetection", new SliderSafeExtendWithStallDetection(neck, slider));
		NamedCommands.registerCommand("sliderRetractWithStallDetection", new SliderRetractWithStallDetection(slider));
		//NamedCommands.registerCommand("neckMoveUpWithStallDetection", new NeckMoveUpWithStallDetection(neck));
		//NamedCommands.registerCommand("neckMoveDownWithStallDetection", new NeckMoveDownWithStallDetection(neck));
		//NamedCommands.registerCommand("neckMoveToCoralReefWithStallDetection", new NeckMoveToCoralReefWithStallDetection(neck));
		//NamedCommands.registerCommand("neckMoveToAlgaeReefWithStallDetection", new NeckMoveToAlgaeReefWithStallDetection(neck));

		// choosers (for auton)

		autoChooser = AutoBuilder.buildAutoChooser("SPB1 - One Coral and Leave SZ");
		SmartDashboard.putData("Auto Chooser", autoChooser);

		// Configure the button bindings

		configureButtonBindings();


		// Configure default commands

		drivetrain.setDefaultCommand(
			// The left stick controls translation of the robot.
			// Turning is controlled by the X axis of the right stick.
			// We are inverting LeftY because Xbox controllers return negative values when we push forward.
			// We are inverting LeftX because we want a positive value when we pull to the left. Xbox controllers return positive values when you pull to the right by default.
			// We are also inverting RightX because we want a positive value when we pull to the left (CCW is positive in mathematics).
			new RunCommand(
				() -> drivetrain.drive(
					-MathUtil.applyDeadband(joyMain.getY(), JOYSTICK_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(joyMain.getX(), JOYSTICK_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(joyMain.getZ(), JOYSTICK_AXIS_THRESHOLD),
					true, true),
				drivetrain));
		
		coral_roller.setDefaultCommand(new CoralRollerStopForever(coral_roller)); // we stop by default
		algae_roller.setDefaultCommand(new AlgaeRollerStopForever(algae_roller)); // we stop by default

		//shooter.setDefaultCommand(new ShooterStopForever(shooter)); // we stop by default

		//compressor.checkCompressor(); //we compress in the background

		indicator.setDefaultCommand(new IndicatorIndicateUsingCamera(indicator)); // default command, only runs when robot is enabled

		indicatorTimedScrollRainbow = new IndicatorTimedScrollRainbow(indicator,1);
		indicatorTimedScrollRainbow.schedule(); // we schedule the command as we are starting up
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {

		// driver (joystick)

		joyMain.povUp()
			.onTrue(new DrivetrainZeroHeading(drivetrain));	

		joyMain.povDown()
			.onTrue(new DrivetrainOppositeHeading(drivetrain));	

		joyMain.povLeft()
			.onTrue(new DrivetrainLeftSubHeading(drivetrain));	

		joyMain.povRight()
			.onTrue(new DrivetrainRightSubHeading(drivetrain));

		joyMain.button(1)
			.whileTrue(new DrivetrainDriveUsingAprilTagCamera(drivetrain, apriltag_camera, getMainJoystick()));

		joyMain.button(2)
			//.whileTrue(new DrivetrainSetXFormation(drivetrain));	
			.whileTrue(new DrivetrainDriveUsingObjectDetectionCamera(drivetrain, object_detection_camera, getMainJoystick()));
			
		joyMain.button(3)
			.onTrue(new MoveInLShapeInReverse(drivetrain, this, 3));
			
		joyMain.button(4)
			//.onTrue(new MoveInGammaShape(drivetrain, this, 3));
			.whileTrue(new DrivetrainSetXFormation(drivetrain));

		joyMain.button(5)
			.onTrue(new MoveLeftOfCoralReef(drivetrain, this, GAMEPAD_AXIS_THRESHOLD));

		joyMain.button(6)
			//.onTrue(new DrivetrainTurnUsingCamera(drivetrain, object_detection_camera));
			//.whileTrue(new DrivetrainSetXFormation(drivetrain));
			.onTrue(new MoveRightOfCoralReef(drivetrain, this, GAMEPAD_AXIS_THRESHOLD));

		joyMain.button(7)
			.whileTrue(new CoralRollerJoystickControl(coral_roller, drivetrain, getMainJoystick()));
		
		joyMain.button(8)
			.whileTrue(new AlgaeRollerJoystickControl(algae_roller, drivetrain, getMainJoystick()));
		
		joyMain.button(9);
			//.whileTrue(new NeckJoystickControl(neck, drivetrain, getMainJoystick()));
		
		joyMain.button(10)
			.whileTrue(new ElevatorJoystickControl(elevator, drivetrain, getMainJoystick()));

		joyMain.button(11)
			//.onTrue(new DrivetrainZeroHeading(drivetrain));
			//.onTrue(new DrivetrainTurnUsingCamera(drivetrain, apriltag_camera));
			.whileTrue(new HangerJoystickControl(hanger, drivetrain, getMainJoystick()));
		
		joyMain.button(12)
			//.whileTrue(new DrivetrainSetXFormation(drivetrain));
			//.onTrue(new DrivetrainTurnUsingCamera(drivetrain, object_detection_camera));
			.onTrue(new DrivetrainDriveTowardsAprilTag(drivetrain, apriltag_camera));
			
				
		// copilot (gamepad)
		
		copilotGamepad.a()
			.whileTrue(new CoralRollerRollIn(coral_roller));
		
		copilotGamepad.b()
			.whileTrue(new CoralRollerRollOut(coral_roller));

		copilotGamepad.x();
			//.onTrue(new CoralRollerReleaseShortDistance(coral_roller));

		copilotGamepad.y();
			//.whileTrue(new RollerRollLowRpm(roller));
			//.onTrue(new RollerRollLowRpmUntilNoteSensed(roller, getNoteSensor()));
			//.onTrue(new RollerReleaseShortDistance(roller));
			//.onTrue(new RollerSuperSmartRoll(roller, noteSensor, noteSensorTwo));
			//.onTrue(new CoralRollerRollLowRpmUntilNoteSensed(coral_roller, noteSensor, noteSensorTwo));
			
		copilotGamepad.back();
			//.onTrue(new DrivetrainAndGyroReset(drivetrain));
			//.onTrue(new AlmostEverythingStop(elevator, old_neck, coral_roller, algae_roller));

		copilotGamepad.start();
			//.onTrue(new AlmostEverythingStop(elevator, neck, roller));
			//.onTrue(new OldNeckHome(old_neck));


		copilotGamepad.leftTrigger();
			//.onTrue(new DrawerRetractWithStallDetection(drawer));
			//.whileTrue(new ShooterTake(shooter));
			//.whileTrue(new ShooterShootHigh(shooter));

		copilotGamepad.rightTrigger()
			//.onTrue(new DrawerExtendWithStallDetection(drawer));
			.whileTrue(new CoralRollerRollOut(coral_roller));


		copilotGamepad.povDown();
			//.onTrue(new ElevatorMoveDownWithStallDetection(elevator));
			//.onTrue(new OldNeckMoveDownWithStallDetection(old_neck));

		copilotGamepad.povLeft();
			//.onTrue(new ElevatorMoveMidwayWithStallDetection(elevator));
			//.onTrue(new OldNeckMoveSubWithStallDetection(old_neck));

		copilotGamepad.povRight();
			//.onTrue(new ElevatorMoveMidwayWithStallDetection(elevator));
			//.onTrue(new NeckMovePodiumWithStallDetection(neck));
			//.onTrue(new OldNeckMoveFeedNoteWithStallDetection(old_neck));

		copilotGamepad.povUp();
			//.onTrue(new ElevatorMoveUpWithStallDetection(elevator));
			//.onTrue(new OldNeckMoveUpWithStallDetection(old_neck));


		copilotGamepad.leftBumper();
			//.onTrue(new NeckMoveUpWithStallDetection(neck));
			//.onTrue(new NeckMoveUpWithStallDetection(neck));
			//.whileTrue(new OldNeckMoveUsingCamera(old_neck, apriltag_camera));

		copilotGamepad.rightBumper();
			//.onTrue(new NeckMoveDownWithStallDetection(neck));
			//.onTrue(new OldNeckMoveAcrossFieldWithStallDetection(old_neck));


		copilotGamepad.leftStick()
			.onTrue(new CoralRollerTimedRoll(coral_roller, 3));
			//.onTrue(new GamepadRumble(getCopilotGamepad(),false));			

		copilotGamepad.rightStick()
			.onTrue(new CoralRollerTimedRelease(coral_roller, 3));
			//.onTrue(new GamepadRumble(getCopilotGamepad(),false));


		copilotGamepad.axisGreaterThan(LY,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));

		copilotGamepad.axisLessThan(LY,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));

		/*copilotGamepad.axisGreaterThan(LX,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue();

		copilotGamepad.axisLessThan(LX,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue();*/

		copilotGamepad.axisGreaterThan(RY,GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new OldNeckGamepadControl(old_neck, getCopilotGamepad()));

		copilotGamepad.axisLessThan(RY,-GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new OldNeckGamepadControl(old_neck, getCopilotGamepad()));

		copilotGamepad.axisGreaterThan(RX,GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new DrawerGamepadControl(drawer, getCopilotGamepad()));
			//.onTrue(new NeckMovePodiumWithStallDetection(neck));

		copilotGamepad.axisLessThan(RX,-GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new DrawerGamepadControl(drawer, getCopilotGamepad()));
			//.onTrue(new NeckMoveSubWithStallDetection(neck));
			
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
		/*autonSelected = autonChooser.getSelected();
		System.out.println("Auton selected: " + autonSelected);	

		gamePieceSelected = gamePieceChooser.getSelected();
		System.out.println("Game piece selected: " + gamePieceSelected);		

		startPosition = startPositionChooser.getSelected();
		System.out.println("Start position: " + startPosition);

		mainTarget = mainTargetChooser.getSelected();
		System.out.println("Main target: " + mainTarget);
		
		cameraOption = cameraOptionChooser.getSelected();
		System.out.println("Camera option: " + cameraOption);
		
		sonarOption = sonarOptionChooser.getSelected();
		System.out.println("Sonar option: " + sonarOption);
		
		releaseSelected = releaseChooser.getSelected();
		System.out.println("Release chosen: " + releaseSelected);

		autonOption = autonOptionChooser.getSelected();
		System.out.println("Auton option: " + autonOption);
		

		switch (autonSelected) {
			case AUTON_SAMPLE_SWERVE:
				//return createSwerveControllerCommand(createExampleTrajectory());
				//return new DrivetrainSwerveRelative(drivetrain, this, createExampleTrajectory());
				return new MoveInSShape(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_FORWARD:
				return new MoveForward(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_IN_REVERSE:
				return new MoveInReverse(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE:
				return new MoveInGammaShape(drivetrain, this, 3);
				//break;

			case AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE:
				return new MoveInLShapeInReverse(drivetrain, this, 3);
				//break;

			case AUTON_TEST_HARDCODED_MOVE_1:
				return new CompletelyLeaveCommunity(drivetrain, this);
				//break;

			/*case AUTON_TEST_HARDCODED_MOVE_2:
				return new MoveInNonBumpKTurn(drivetrain, this);
				//break;

			case AUTON_TEST_TRAJECTORY_GENERATION:
				return new TrajectoryGenerationTest(drivetrain, this, object_detection_camera, apriltag_camera);
				//break;

			case AUTON_CUSTOM:
				return new CustomAuton(gamePieceSelected, startPosition, mainTarget, cameraOption, sonarOption, autonOption, drivetrain, this, elevator, roller, neck, shooter, object_detection_camera, apriltag_camera, noteSensor, noteSensorTwo);
				//break;

			case AUTON_DO_NOTHING:
				return null;
				//break;
				
			default:
				// nothing
				return null;
				//break;
		}*/ // end switch
	}

	public TrajectoryConfig createFastTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.HIGH_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createSlowTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.REDUCED_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}


	public TrajectoryConfig createReverseTrajectoryConfig() {

		TrajectoryConfig config = createTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}

	public TrajectoryConfig createFastReverseTrajectoryConfig() {

		TrajectoryConfig config = createFastTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}

	/*public Trajectory createExampleTrajectory() {
		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
			createTrajectoryConfig());

		return exampleTrajectory;
	}*/
	
	/*public Command createSwerveControllerCommand(Trajectory trajectory) {

		ProfiledPIDController thetaController = new ProfiledPIDController(
			AutoConstants.THETA_CONTROLLER_P, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
			
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
			trajectory, // trajectory to follow
			drivetrain::getPose, // Functional interface to feed supplier
			DrivetrainConstants.DRIVE_KINEMATICS, // kinematics of the drivetrain
			new PIDController(AutoConstants.X_CONTROLLER_P, 0, 0), // trajectory tracker PID controller for x position
			new PIDController(AutoConstants.Y_CONTROLLER_P, 0, 0), // trajectory tracker PID controller for y position
			thetaController, // trajectory tracker PID controller for rotation
			drivetrain::setModuleStates, // raw output module states from the position controllers
			drivetrain); // subsystems to require

		// Reset odometry to the starting pose of the trajectory.
		drivetrain.resetOdometry(trajectory.getInitialPose()); // WARNING: https://github.com/REVrobotics/MAXSwerve-Java-Template/issues/13

		field.getObject("trajectory").setTrajectory(trajectory);

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false, false));
	}*/

	/*public AprilTagFieldLayout getAprilTagFieldLayout()
	{
		return FIELD_LAYOUT;
	}*/

	public Field2d getField()
	{
		return field;
	}

	public HMAccelerometer getAccelerometer()
	{
		return accelerometer;
	}

	public ICamera getObjectDetectionCamera()
	{
		return object_detection_camera;
	}

	public ICamera getAprilTagCamera()
	{
		return apriltag_camera;
	}

	/*(public NoteSensor getNoteSensor()
	{
		return noteSensor;
	}*/

	/*public CoralSensor getCoralSensor() 
	{
		return coralSensor;
	}*/

	public SwerveDrivetrain getDrivetrain()
	{
		return drivetrain;
	}

	public Elevator getElevator()
	{
		return elevator;
	}

	/*public Drawer getDrawer()
	{
		return drawer;
	}*/

	/*public Neck getNeck()
	{
		return neck;
	}*/

	public Slider getSlider()
	{
		return slider;
	}

	public Hanger getHanger()
	{
		return hanger;
	}


	public CoralRoller getCoralRoller()
	{
		return coral_roller;
	}

	public AlgaeRoller getAlgaeRoller()
	{
		return algae_roller;
	}

	/*public Mouth getMouth()
	{
		return mouth;
	}*/

	/*public Shooter getShooter()
	{
		return shooter;
	}*/

	public Joystick getMainJoystick()
	{
		return joyMain.getHID();
	}

	public XboxController getCopilotGamepad()
	{
		return copilotGamepad.getHID();
	}

	public Joystick getButtonBox() 
	{
		return buttonBox.getHID();
	}

	/*public SendableChooser<String> getAutonChooser()
	{
		return autonChooser;
	}
	
	public SendableChooser<String> getGamePieceChooser()
	{
		return gamePieceChooser;
	}

	public SendableChooser<String> getStartPositionChooser()
	{
		return startPositionChooser;
	}

	public SendableChooser<String> getMainTargetChooser()
	{
		return mainTargetChooser;
	}

	public SendableChooser<String> getCameraOptionChooser()
	{
		return cameraOptionChooser;
	}

	public SendableChooser<String> getSonarOptionChooser()
	{
		return sonarOptionChooser;
	}

	public SendableChooser<String> getReleaseChooser()
	{
		return releaseChooser;
	}

	public SendableChooser<String> getAutonOptionChooser()
	{
		return autonOptionChooser;
	}*/
}
