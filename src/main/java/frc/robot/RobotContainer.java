// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import java.util.List;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.auton.AutoAlignLeftAndElevatorUp;
import frc.robot.auton.AutoAlignRightAndElevatorUp;
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
//import frc.robot.subsystems.CoralRoller;
//import frc.robot.subsystems.AlgaeRoller;
//import frc.robot.subsystems.Indicator;
import frc.robot.subsystems.Neck;
//import frc.robot.commands.DoNothing;
import frc.robot.commands.algae_roller.*;
//import frc.robot.subsystems.SimpleShooter;
import frc.robot.commands.coral_roller.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.slider.*;
import frc.robot.commands.neck.*;
import frc.robot.commands.elevator.*;
import frc.robot.commands.hanger.*;
//import frc.robot.commands.simpleshooter.*;
//import frc.robot.commands.shooter.*;
import frc.robot.interfaces.ICamera;
//import frc.robot.commands.mouth.*;
//import frc.robot.commands.indicator.*;
import frc.robot.commands.groups.*;
//import frc.robot.commands.gamepad.*;
//import frc.robot.auton.*;
//import frc.robot.auton.trajectories.*;


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

	/*public static final String AUTON_DO_NOTHING = "Do Nothing";
	public static final String AUTON_CUSTOM = "My Auto";
	private String autonSelected;
	private SendableChooser<String> autonChooser = new SendableChooser<>();
*/
	// sensors

	private final HMAccelerometer accelerometer = new HMAccelerometer();

	//private final ICamera object_detection_camera = new ObjectDetectionCamera();

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
	
	private final /*I*/Neck neck = new Neck();

	//private final TalonFX old_neck_master = new TalonFX(Ports.CAN.OLD_NECK_MASTER);
	//private final TalonFX old_neck_follower = new TalonFX(Ports.CAN.OLD_NECK_FOLLOWER);
	
	//private final /*I*/OldNeck old_neck = new OldNeck(old_neck_master, old_neck_follower);

	//private final CANSparkMax roller_master = new CANSparkMax(Ports.CAN.ROLLER, MotorType.kBrushless);
	private final WPI_TalonSRX coral_roller_master = new WPI_TalonSRX(Ports.CAN.CORAL_ROLLER_MASTER);
	//private final WPI_TalonSRX coral_roller_follower = new WPI_TalonSRX(Ports.CAN.ROLLER_FOLLOWER);
	private final WPI_TalonSRX algae_roller_master = new WPI_TalonSRX(Ports.CAN.ALGAE_ROLLER_MASTER);
	//private final WPI_TalonSRX algae_roller_follower = new WPI_TalonSRX(Ports.CAN.ROLLER_FOLLOWER);

	private final /*I*/CoralRoller coral_roller = new CoralRoller(coral_roller_master);
	private final /*I*/AlgaeRoller algae_roller = new AlgaeRoller(algae_roller_master);

	private final WPI_TalonSRX slider_master = new WPI_TalonSRX(Ports.CAN.SLIDER_MASTER);
	private final Slider slider = new Slider(slider_master);

	private final TalonFX hanger_master = new TalonFX(Ports.CAN.HANGER_MASTER);
	private final Hanger hanger = new Hanger(hanger_master);

	// pneumatic devices

	//private final Compressor compressor = new Compressor();

	//private final Mouth mouth = new Mouth();

	// misc

	private final Field2d field = new Field2d(); //  a representation of the field

	//private final Indicator indicator = new Indicator(apriltag_camera, object_detection_camera);

	//public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

	// The driver's and copilot's joystick(s) and controller(s)
	CommandJoystick joyMain = new CommandJoystick(Ports.USB.MAIN_JOYSTICK);
	//CommandXboxController driverGamepad = new CommandXboxController(Ports.USB.DRIVER_GAMEPAD);
	CommandXboxController copilotGamepad = new CommandXboxController(Ports.USB.COPILOT_GAMEPAD);
	CommandJoystick buttonBox = new CommandJoystick(Ports.USB.BUTTON_BOX);
	
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		//autonChooser.setDefaultOption("SP2 One Coral Test", AUTON_CUSTOM);
		//SmartDashboard.putData("Auto choices", autonChooser); 

		NamedCommands.registerCommand("elevatorMoveToFourthLevelWithStallDetection", new ElevatorMoveToFourthLevelWithStallDetection(elevator));
        NamedCommands.registerCommand("elevatorMoveToThirdLevelWithStallDetection", new ElevatorMoveToThirdLevelWithStallDetection(elevator));
        NamedCommands.registerCommand("elevatorMoveToSecondLevelWithStallDetection", new ElevatorMoveToSecondLevelWithStallDetection(elevator));
        NamedCommands.registerCommand("elevatorMoveToFirstLevelWithStallDetection", new ElevatorMoveToFirstLevelWithStallDetection(elevator));
        NamedCommands.registerCommand("elevatorMoveUpWithStallDetection", new ElevatorMoveUpWithStallDetection(elevator));
        NamedCommands.registerCommand("elevatorMoveDownWithStallDetection", new ElevatorMoveDownWithStallDetection(elevator));
		NamedCommands.registerCommand("elevatorMoveToAlgaeLevelTwoWithStallDetection", new ElevatorMoveToAlgaeLevelTwoWithStallDetection(elevator));
		NamedCommands.registerCommand("elevatorMoveToAlgaeLevelThreeWithStallDetection", new ElevatorMoveToAlgaeLevelThreeWithStallDetection(elevator));
		NamedCommands.registerCommand("elevatorMoveToAlgaeLevelTwoNeckDownWithStallDetection", new ElevatorMoveToAlgaeLevelTwoNeckDownWithStallDetection(elevator));
		NamedCommands.registerCommand("elevatorMoveToAlgaeLevelThreeNeckDownWithStallDetection", new ElevatorMoveToAlgaeLevelThreeNeckDownWithStallDetection(elevator));
        NamedCommands.registerCommand("algaeRollerTimedRoll", new AlgaeRollerTimedRoll(algae_roller, 1));
		NamedCommands.registerCommand("algaeRollerTimedRollForAlgaeRemoval", new AlgaeRollerTimedRoll(algae_roller, 1));
        NamedCommands.registerCommand("algaeRollerTimedReleaseForAlgaeRemoval", new AlgaeRollerTimedRelease(algae_roller, 2));
		NamedCommands.registerCommand("algaeRollerStop", new AlgaeRollerStop(algae_roller));
        NamedCommands.registerCommand("CoralRollerTimedRollIn", new CoralRollerTimedRollIn(coral_roller, .4));
        NamedCommands.registerCommand("CoralRollerTimedRollOut", new CoralRollerTimedRollOut(coral_roller, .4));
		NamedCommands.registerCommand("coralRollerStop", new CoralRollerStop(coral_roller));
		NamedCommands.registerCommand("sliderExtendWithStallDetection", new SliderExtendWithLimitSwitch(slider));
		NamedCommands.registerCommand("sliderRetractWithStallDetection", new SliderRetractWithLimitSwitch(slider));
		NamedCommands.registerCommand("neckMoveUpWithStallDetection", new NeckMoveUpWithStallDetection(neck));
		NamedCommands.registerCommand("neckMoveDownWithStallDetection", new NeckMoveDownWithStallDetection(neck));
		NamedCommands.registerCommand("neckMoveToCoralReefWithStallDetection", new NeckMoveToCoralReefWithStallDetection(neck));
		NamedCommands.registerCommand("neckMoveToCoralStationWithStallDetection", new NeckMoveToCoralStationWithStallDetection(neck));
		NamedCommands.registerCommand("neckMoveToAlgaeReefWithStallDetection", new NeckMoveToAlgaeReefWithStallDetection(neck));
		/*NamedCommands.registerCommand("scoreFirstLevelCoral", new ScoreFirstLevelCoral(elevator, coral_roller, neck));
		NamedCommands.registerCommand("scoreSecondLevelCoral", new ScoreSecondLevelCoral(elevator, coral_roller, neck));
		NamedCommands.registerCommand("scoreThirdLevelCoral", new ScoreThirdLevelCoral(elevator, coral_roller, neck));
		NamedCommands.registerCommand("scoreFourthLevelCoral", new ScoreFourthLevelCoral(elevator, coral_roller, neck));
		NamedCommands.registerCommand("grabLevelTwoAlgae", new GrabLevelTwoAlgae(elevator, algae_roller, slider));
		NamedCommands.registerCommand("grabLevelThirdAlgae", new GrabLevelThreeAlgae(elevator, algae_roller, slider));
		NamedCommands.registerCommand("scoreNet", new ScoreNet(elevator, algae_roller, slider, neck));
		NamedCommands.registerCommand("scoreProcessor", new ScoreProcessor(algae_roller, neck, slider));*/
		NamedCommands.registerCommand("doNothingUntilCoralSensed", new DoNothingUntilCoralSensed(coral_roller));
		//NamedCommands.registerCommand("home", new Home(elevator, slider, neck));
		//NamedCommands.registerCommand("homeToCoralStation", new HomeToCoralStation(elevator, /*slider,*/ neck));
		NamedCommands.registerCommand("drivetrainStop", new DrivetrainStop(drivetrain));
		//NamedCommands.registerCommand("scoreFourthLevelCoralAndHomeToCoralStation", new ScoreFourthLevelCoralAndHomeToCoralStation(elevator, coral_roller, neck, slider));
		//NamedCommands.registerCommand("scoreFourthLevelCoralAndHome", new ScoreFourthLevelCoralAndHome(elevator, coral_roller, neck, slider));
		NamedCommands.registerCommand("neckHome", new NeckMoveHomeWithStallDetection(neck));
		NamedCommands.registerCommand("coralRollerForAutoRollOut", new CoralRollerForAutoRollOut(coral_roller));
		NamedCommands.registerCommand("waitCommand2s", new WaitCommand(2));
		NamedCommands.registerCommand("waitCommand1.5s", new WaitCommand(1.5));
		NamedCommands.registerCommand("waitCommand1s", new WaitCommand(1));
		NamedCommands.registerCommand("autoAlignToLeftReef", new AutoAlignToReef(false, drivetrain, apriltag_camera, getMainJoystick()));
		NamedCommands.registerCommand("autoAlignToRightReef", new AutoAlignToReef(true, drivetrain, apriltag_camera, getMainJoystick()));
		//NamedCommands.registerCommand("autoAlignToRightReefTimed", new AutoAlignToReefTimed(true, drivetrain, apriltag_camera, 2));
		NamedCommands.registerCommand("autoAlignToRightReefAuton", new AutoAlignToReefForAuton(true, drivetrain, apriltag_camera, getMainJoystick()));
		NamedCommands.registerCommand("autoAlignToLeftReefForAuton", new AutoAlignToReefForAuton(false, drivetrain, apriltag_camera, getMainJoystick()));

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
		hanger.setDefaultCommand(new HangerStop(hanger));

		//shooter.setDefaultCommand(new ShooterStopForever(shooter)); // we stop by default

		//compressor.checkCompressor(); //we compress in the background

		//indicator.setDefaultCommand(new IndicatorIndicateUsingCamera(indicator)); // default command, only runs when robot is enabled

		//indicatorTimedScrollRainbow = new IndicatorTimedScrollRainbow(indicator,1);
		//indicatorTimedScrollRainbow.schedule(); // we schedule the command as we are starting up

		Trigger hasCoral = new Trigger(() -> coral_roller.hasCoral());
		Trigger noCoralPresent = new Trigger(() -> coral_roller.noCoralPresent() && !coral_roller.isReleasing());
		Trigger isCoralEntering = new Trigger(() -> coral_roller.isCoralEntering() && !coral_roller.isReleasing());
		Trigger isCoralExiting = new Trigger(() -> coral_roller.isCoralExiting() && !coral_roller.isReleasing());

		isCoralEntering.whileTrue(
			new CoralRollerRollOutLowRpm(coral_roller)
		);

		isCoralExiting.whileTrue(
			new CoralRollerRollInLowRpm(coral_roller)
		);

		(hasCoral).or(noCoralPresent).whileTrue(
			new CoralRollerStop(coral_roller)
		);
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
			//.whileTrue(new DrivetrainDriveUsingObjectDetectionCamera(drivetrain, object_detection_camera, getMainJoystick()));
			.whileTrue(new DrivetrainDriveTowardsAprilTag(drivetrain, apriltag_camera));
			
		joyMain.button(3)
			//.onTrue(new MoveInLShapeInReverse(drivetrain, this, 3));
			//.onTrue(new AutoAlignToReefTimed(false, drivetrain, apriltag_camera, 2));
			.onTrue(new AutoAlignToReefForAuton(false, drivetrain, apriltag_camera, getMainJoystick()));
			//.onTrue(new AutoAlignLeftAndElevatorUp(drivetrain, elevator, apriltag_camera, getMainJoystick()));
			
			
		joyMain.button(4)
			//.onTrue(new MoveInGammaShape(drivetrain, this, 3));
			//.whileTrue(new DrivetrainSetXFormation(drivetrain));
			//.onTrue(new AutoAlignToReefTimed(true, drivetrain, apriltag_camera, 2));
			.onTrue(new AutoAlignToReefForAuton(true, drivetrain, apriltag_camera, getMainJoystick()));
			//.onTrue(new AutoAlignRightAndElevatorUp(drivetrain, elevator, apriltag_camera, getMainJoystick()));

		joyMain.button(5)
			//.onTrue(new MoveLeftOfCoralReef(drivetrain, this));
			//.whileTrue(new AutoAlignToReef(null, null, true, drivetrain));
			.onTrue(new AutoAlignToReef(false, drivetrain, apriltag_camera, getMainJoystick()));

		joyMain.button(6)
			//.onTrue(new DrivetrainTurnUsingCamera(drivetrain, object_detection_camera));
			//.whileTrue(new DrivetrainSetXFormation(drivetrain));
			//.onTrue(new MoveRightOfCoralReef(drivetrain, this));
			//.whileTrue(new AutoAlignToReef(null, null, false, drivetrain));
			.onTrue(new AutoAlignToReef(true, drivetrain, apriltag_camera, getMainJoystick()));

		joyMain.button(7)
			//.whileTrue(new CoralRollerJoystickControl(coral_roller, drivetrain, getMainJoystick()));
			.whileTrue(new SliderJoystickControl(slider, drivetrain, getMainJoystick()));
		
		joyMain.button(8)
			.whileTrue(new AlgaeRollerJoystickControl(algae_roller, drivetrain, getMainJoystick()));
		
		joyMain.button(9)
			.whileTrue(new NeckJoystickControl(neck, drivetrain, getMainJoystick()));
		
		joyMain.button(10)
			.whileTrue(new ElevatorJoystickControl(elevator, drivetrain, getMainJoystick()));

		joyMain.button(11)
			.whileTrue(new HangerJoystickControl(hanger, drivetrain, getMainJoystick()));
		
		joyMain.button(12)
			//.onTrue(new DrivetrainDriveTowardsAprilTag(drivetrain, apriltag_camera));
			.whileTrue(new DrivetrainSetXFormation(drivetrain));
			
				
		// copilot (gamepad)
		
		copilotGamepad.a()
			//.onTrue(new NeckMoveDownWithStallDetection(neck));
			//.onTrue(new NeckMoveToAlgaeReefWithStallDetection(neck));
			.onTrue(new ElevatorMoveToAlgaeLevelTwoWithStallDetection(elevator));
		
		copilotGamepad.b()
			//.onTrue(new NeckMoveProcessorWithStallDetection(neck));
			//.onTrue(new NeckMoveUpWithStallDetection(neck));
			//.onTrue(new NeckMoveHomeWithStallDetection(neck));
			//.onTrue(new NeckMoveDownWithStallDetection(neck));
			.onTrue(new ElevatorMoveDownWithStallDetection(elevator));

		copilotGamepad.x()
			//.whileTrue(new CoralRollerRollIn(coral_roller));
			.onTrue(new ElevatorMoveToAlgaeLevelThreeWithStallDetection(elevator));

		copilotGamepad.y()
			//.whileTrue(new CoralRollerRollOut(coral_roller));
			.whileTrue(new AlgaeRollerRelease(algae_roller));
			
		copilotGamepad.back()
			.onTrue(new DrivetrainAndGyroReset(drivetrain));
			//.onTrue(new AlmostEverythingStop(elevator, old_neck, coral_roller, algae_roller));

		copilotGamepad.start()
			.onTrue(new AlmostEverythingStop(elevator, coral_roller, algae_roller));
			//.onTrue(new OldNeckHome(old_neck));


		copilotGamepad.leftTrigger()
			.whileTrue(new CoralRollerRollOut(coral_roller));

		copilotGamepad.rightTrigger()
			//.whileTrue(new AlgaeRollerRelease(algae_roller));
			.whileTrue(new AlgaeRollerRoll(algae_roller));


		copilotGamepad.povDown()
			//.onTrue(new NeckMoveDownWithStallDetection(neck));
			.onTrue(new NeckMoveHomeWithStallDetection(neck));

		copilotGamepad.povLeft()
			.onTrue(new NeckMoveToCoralStationWithStallDetection(neck));

		copilotGamepad.povRight()
			//.onTrue(new NeckMoveToCoralReefWithStallDetection(neck));
			//.onTrue(new NeckMoveToAlgaeReefWithStallDetection(neck));
			.onTrue(new NeckMoveUpWithStallDetection(neck));

		copilotGamepad.povUp();
			//.onTrue(new NeckMoveUpWithStallDetection(neck));
			//.onTrue(new ElevatorMoveUpWithStallDetection(elevator));
			//.onTrue(new NeckMoveUpWithStallDetection(neck));
			//.onTrue(new NeckMoveToAlgaeReefWithStallDetection(neck));
			//.onTrue(new NeckMoveUpWithStallDetection(neck));


		copilotGamepad.leftBumper()
			.onTrue(new SliderExtendWithLimitSwitch(slider)); // TODO see if safe mode is needed?
			//.onTrue(new ElevatorMoveToFirstLevelWithStallDetection(elevator));

		copilotGamepad.rightBumper()
			.onTrue(new SliderRetractWithLimitSwitch(slider));


		copilotGamepad.leftStick();
			//.onTrue(new CoralRollerTimedRoll(coral_roller, 3));
			//.onTrue(new GamepadRumble(getCopilotGamepad(),false));
			//.onTrue(new NeckMoveToCoralReefWithStallDetection(neck));

		copilotGamepad.rightStick();
			//.onTrue(new CoralRollerTimedRelease(coral_roller, 3));
			//.onTrue(new GamepadRumble(getCopilotGamepad(),false));
			//.onTrue(new ElevatorMoveUpWithStallDetection(elevator));


		copilotGamepad.axisGreaterThan(LY,GAMEPAD_AXIS_THRESHOLD)
			//.whileTrue(new HangerGamepadControl(hanger, getCopilotGamepad()));
			.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		copilotGamepad.axisLessThan(LY,-GAMEPAD_AXIS_THRESHOLD)
			//.whileTrue(new HangerGamepadControl(hanger, getCopilotGamepad()));
			.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		copilotGamepad.axisGreaterThan(LX,GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));
			//.onTrue(new ElevatorMoveToThirdLevelWithStallDetection(elevator));

		copilotGamepad.axisLessThan(LX,-GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));
			//.onTrue(new ElevatorMoveToSecondLevelWithStallDetection(elevator));

		copilotGamepad.axisGreaterThan(RY,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));

		copilotGamepad.axisLessThan(RY,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));
			//.onTrue(new ElevatorMoveToFourthLevelWithStallDetection(elevator));

		copilotGamepad.axisGreaterThan(RX,GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new SliderGamepadControl(slider, getCopilotGamepad()));
			//.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		copilotGamepad.axisLessThan(RX,-GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new SliderGamepadControl(slider, getCopilotGamepad()));
			//.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));
		
		// button box 

		buttonBox.button(1)
			.onTrue(new ElevatorMoveToFirstLevelWithStallDetection(elevator)); 

		buttonBox.button(2)
			.onTrue(new ElevatorMoveToSecondLevelWithStallDetection(elevator));
		
		buttonBox.button(3)
			.onTrue(new ElevatorMoveToThirdLevelWithStallDetection(elevator));

		buttonBox.button(4)
			.onTrue(new ElevatorMoveToFourthLevelWithStallDetection(elevator));
		
		buttonBox.button(5)
			.onTrue(new ElevatorMoveUpWithStallDetection(elevator));
		
		buttonBox.button(6)
			.onTrue(new ElevatorMoveDownWithStallDetection(elevator));
		
		buttonBox.button(9)
			.onTrue(new ElevatorMoveToAlgaeLevelTwoWithStallDetection(elevator));

		buttonBox.button(10)
			.onTrue(new ElevatorMoveToAlgaeLevelThreeWithStallDetection(elevator));
		
		buttonBox.button(11)
			.whileTrue(new HangerButtonBoxUpControl(hanger));
		
		buttonBox.button(12)
			.whileTrue(new HangerButtonBoxDownControl(hanger));
		
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

		switch (autonSelected) {
			case AUTON_CUSTOM:
				return new StartingPositionTwoOneCoral(this, drivetrain, coral_roller, neck, elevator, slider);
				//break;

			case AUTON_DO_NOTHING:
				return null;
				//break;
					
			default:
				// nothing
				return null;
				//break;
			} // end switch*/
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

	public TrajectoryConfig createAlmostMaxTrajectoryConfig() {
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

	/*public ICamera getObjectDetectionCamera()
	{
		return object_detection_camera;
	}*/

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

	public Neck getNeck()
	{
		return neck;
	}

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
	}*/

}
