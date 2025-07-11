// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.subsystems.Slider;
import frc.robot.interfaces.ICamera;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.net.PortForwarder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		// Port forwarders for LimeLight
		// Do not place these function calls in any periodic functions
		PortForwarder.add(5800, "limelight.local", 5800);
		PortForwarder.add(5801, "limelight.local", 5801);
		PortForwarder.add(5802, "limelight.local", 5802);
		PortForwarder.add(5803, "limelight.local", 5803);
		PortForwarder.add(5804, "limelight.local", 5804);
		PortForwarder.add(5805, "limelight.local", 5805);

		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();

		SmartDashboard.putData("Swerve Odometry", m_robotContainer.getField());		

		FollowPathCommand.warmupCommand().schedule(); 

		CanandEventLoop.getInstance();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		
		if (m_robotContainer.getAprilTagCamera() != null) 
		{
			m_robotContainer.getAprilTagCamera().updateCacheResults();
			Optional<EstimatedRobotPose> result = m_robotContainer.getAprilTagCamera().getGlobalPose();

			if (result.isPresent() && m_robotContainer.getVisionCorrectionEnablement())
			{
				// calls the overriden version of addVisionMeasurement() that internally calls Utils.fpgaToCurrentTime() to use the correct time base
				m_robotContainer.getDrivetrain().addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
			}
		}

	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {

		// m_robotContainer.getCamera().acquireTargets(false);

		updateToSmartDash();
	}

	/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		
		updateToSmartDash();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {

	//	m_robotContainer.getCamera().acquireTargets(false);

		updateToSmartDash();
	}

	public void updateToSmartDash()
	{
		// TODO
		SmartDashboard.putNumber("FrontLeftTurningAbsoluteEncoderPosition", m_robotContainer.getDrivetrain().getFrontLeftAbsoluteEncoderPosition());
		SmartDashboard.putNumber("FrontRightTurningAbsoluteEncoderPosition", m_robotContainer.getDrivetrain().getFrontRightAbsoluteEncoderPosition());
		SmartDashboard.putNumber("RearLeftTurningAbsoluteEncoderPosition", m_robotContainer.getDrivetrain().getRearLeftAbsoluteEncoderPosition());
		SmartDashboard.putNumber("RearRightTurningAbsoluteEncoderPosition", m_robotContainer.getDrivetrain().getRearRightAbsoluteEncoderPosition());
		
		SmartDashboard.putNumber("FrontLeftRelativeEncoderPosition", m_robotContainer.getDrivetrain().getFrontLeftRelativeEncoderPosition());
		SmartDashboard.putNumber("FrontRightRelativeEncoderPosition", m_robotContainer.getDrivetrain().getFrontRightRelativeEncoderPosition());
		SmartDashboard.putNumber("RearLeftRelativeEncoderPosition", m_robotContainer.getDrivetrain().getRearLeftRelativeEncoderPosition());
		SmartDashboard.putNumber("RearRightRelativeEncoderPosition", m_robotContainer.getDrivetrain().getRearRightRelativeEncoderPosition());
		
		// SmartDashboard.putNumber("x", m_robotContainer.getDrivetrain().getPose().getX()); // lets us see the absolute position of the robot on the field
    	// SmartDashboard.putNumber("y", m_robotContainer.getDrivetrain().getPose().getY()); 
    	// SmartDashboard.putNumber("rot", m_robotContainer.getDrivetrain().getPose().getRotation().getDegrees()); 
	  
		// SmartDashboard.putNumber("FrontLeftDrivingEncoderPosition", m_robotContainer.getDrivetrain().getFrontLeftModule().getDrivingEncoder().getPosition());
		// SmartDashboard.putNumber("FrontLeftTurningEncoderPosition", m_robotContainer.getDrivetrain().getFrontLeftModule().getTurningEncoder().getPosition());
		
		// SmartDashboard.putNumber("RearLeftDrivingEncoderPosition", m_robotContainer.getDrivetrain().getRearLeftModule().getDrivingEncoder().getPosition());
		// SmartDashboard.putNumber("RearLeftTurningEncoderPosition", m_robotContainer.getDrivetrain().getRearLeftModule().getTurningEncoder().getPosition());
		
		// SmartDashboard.putNumber("FrontRightDrivingEncoderPosition", m_robotContainer.getDrivetrain().getFrontRightModule().getDrivingEncoder().getPosition());
		// SmartDashboard.putNumber("FrontRightTurningEncoderPosition", m_robotContainer.getDrivetrain().getFrontRightModule().getTurningEncoder().getPosition());
		
		// SmartDashboard.putNumber("RearRightDrivingEncoderPosition", m_robotContainer.getDrivetrain().getRearRightModule().getDrivingEncoder().getPosition());
		// SmartDashboard.putNumber("RearRightTurningEncoderPosition", m_robotContainer.getDrivetrain().getRearRightModule().getTurningEncoder().getPosition());
	
		// SmartDashboard.putNumber("FrontLeftTurningAbsoluteEncoderPosition", m_robotContainer.getDrivetrain().getFrontLeftModule().getTurningAbsoluteEncoder().getPosition());
		// SmartDashboard.putNumber("RearLeftTurningAbsoluteEncoderPosition", m_robotContainer.getDrivetrain().getRearLeftModule().getTurningAbsoluteEncoder().getPosition());
		// SmartDashboard.putNumber("FrontRightTurningAbsoluteEncoderPosition", m_robotContainer.getDrivetrain().getFrontRightModule().getTurningAbsoluteEncoder().getPosition());
		// SmartDashboard.putNumber("RearRightTurningAbsoluteEncoderPosition", m_robotContainer.getDrivetrain().getRearRightModule().getTurningAbsoluteEncoder().getPosition());

		// SmartDashboard.putNumber("FrontLeftTurningAbsoluteEncoderVirtualPosition", m_robotContainer.getDrivetrain().getFrontLeftModule().getTurningAbsoluteEncoder().getVirtualPosition());
		// SmartDashboard.putNumber("RearLeftTurningAbsoluteEncoderVirtualPosition", m_robotContainer.getDrivetrain().getRearLeftModule().getTurningAbsoluteEncoder().getVirtualPosition());
		// SmartDashboard.putNumber("FrontRightTurningAbsoluteEncoderVirtualPosition", m_robotContainer.getDrivetrain().getFrontRightModule().getTurningAbsoluteEncoder().getVirtualPosition());
		// SmartDashboard.putNumber("RearRightTurningAbsoluteEncoderVirtualPosition", m_robotContainer.getDrivetrain().getRearRightModule().getTurningAbsoluteEncoder().getVirtualPosition());
	
		// SmartDashboard.putNumber("FrontLeftTurningDesiredState", m_robotContainer.getDrivetrain().getFrontLeftModule().getDesiredState().angle.getRadians());
		// SmartDashboard.putNumber("RearLeftTurningDesiredState", m_robotContainer.getDrivetrain().getRearLeftModule().getDesiredState().angle.getRadians());
		// SmartDashboard.putNumber("FrontRightTurningDesiredState", m_robotContainer.getDrivetrain().getFrontRightModule().getDesiredState().angle.getRadians());
		// SmartDashboard.putNumber("RearRightTurningDesiredState", m_robotContainer.getDrivetrain().getRearRightModule().getDesiredState().angle.getRadians());

		// /* Display 6-axis Processed Angle Data                                      */
		// SmartDashboard.putBoolean("IMU_Connected", m_robotContainer.getDrivetrain().getImu().isConnected());
		// //SmartDashboard.putBoolean("IMU_IsCalibrating", m_robotContainer.getDrivetrain().getImu().isCalibrating());
		// //SmartDashboard.putNumber("IMU_Yaw", m_robotContainer.getDrivetrain().getImu().getYaw()); /* TODO FIX */
		// //SmartDashboard.putNumber("IMU_Pitch", m_robotContainer.getDrivetrain().getImu().getPitch());
		// //SmartDashboard.putNumber("IMU_Roll", m_robotContainer.getDrivetrain().getImu().getRoll());

		m_robotContainer.getField().setRobotPose(m_robotContainer.getDrivetrain().getPose());
		SmartDashboard.putNumber("Heading", m_robotContainer.getDrivetrain().getHeading());


		SmartDashboard.putNumber("AccelZ", m_robotContainer.getAccelerometer().getAccelZ());
		//SmartDashboard.putNumber("FilteredAccelZ", m_robotContainer.getAccelerometer().getFilteredAccelZ());
		SmartDashboard.putNumber("Tilt", m_robotContainer.getAccelerometer().getTilt());
		//SmartDashboard.putNumber("FilteredTilt", m_robotContainer.getAccelerometer().getFilteredTilt());
		SmartDashboard.putNumber("AccurateTilt", m_robotContainer.getAccelerometer().getAccurateTilt());
		SmartDashboard.putBoolean("isFlat", m_robotContainer.getAccelerometer().isFlat());
		SmartDashboard.putBoolean("isSuperFlat", m_robotContainer.getAccelerometer().isSuperFlat());
		SmartDashboard.putNumber("AccuratePitch", m_robotContainer.getAccelerometer().getAccuratePitch());
		SmartDashboard.putNumber("AccurateRoll", m_robotContainer.getAccelerometer().getAccurateRoll());
		//SmartDashboard.putNumber("FilteredAccurateRoll", m_robotContainer.getAccelerometer().getFilteredAccurateRoll());

		/*SmartDashboard.putNumber("Distance to Target", m_robotContainer.getCamera().getDistanceToCompositeTargetUsingVerticalFov());
		SmartDashboard.putNumber("Angle to Target", m_robotContainer.getCamera().getAngleToTurnToCompositeTarget());
		SmartDashboard.putNumber("Distance to Target Using Horizontal FOV", m_robotContainer.getCamera().getDistanceToCompositeTargetUsingHorizontalFov());
		SmartDashboard.putNumber("Filtered Distance to Target", m_robotContainer.getCamera().getFilteredDistanceToCompositeTarget());
		SmartDashboard.putNumber("Vertical Offset to Target", m_robotContainer.getCamera().getVerticalOffsetToCompositeTarget());
		SmartDashboard.putNumber("Filtered Vertical Offset to Target", m_robotContainer.getCamera().getFilteredVerticalOffsetToCompositeTarget());*/

		SmartDashboard.putNumber("Distance to AprilTag", m_robotContainer.getAprilTagCamera().getDistanceToTarget());
		SmartDashboard.putNumber("Angle to AprilTag", m_robotContainer.getAprilTagCamera().getAngleToTurnToTarget());
		SmartDashboard.putBoolean("At Left Scoring Position?", m_robotContainer.getAprilTagCamera().isAtLeftScoringPosition());
		SmartDashboard.putBoolean("At Right Scoring Position?", m_robotContainer.getAprilTagCamera().isAtRightScoringPosition());
		//SmartDashboard.putNumber("Latest AprilTag ID", m_robotContainer.getAprilTagCamera().getLatestID());

		SmartDashboard.putBoolean("Elevator Forward Limit Switch", m_robotContainer.getElevator().getForwardLimitSwitchState());
		SmartDashboard.putBoolean("Elevator Reverse Limit Switch", m_robotContainer.getElevator().getReverseLimitSwitchState());
		SmartDashboard.putNumber("Elevator Enc Position", m_robotContainer.getElevator().getEncoderPosition());
		SmartDashboard.putBoolean("Elevator IsMoving?", m_robotContainer.getElevator().isMoving());
		SmartDashboard.putNumber("Elevator Target", m_robotContainer.getElevator().getTarget());
		SmartDashboard.putBoolean("Elevator isStalled?", m_robotContainer.getElevator().isStalled());
		SmartDashboard.putBoolean("Elevator isDown", m_robotContainer.getElevator().isDown());
		SmartDashboard.putBoolean("Elevator isMidway", m_robotContainer.getElevator().isMidway());
		SmartDashboard.putBoolean("Elevator isUp", m_robotContainer.getElevator().isUp());
		SmartDashboard.putBoolean("Elevator isDangerous", m_robotContainer.getElevator().isDangerous());

		SmartDashboard.putBoolean("Neck Reverse Limit Switch", m_robotContainer.getNeck().getReverseLimitSwitchState());
		SmartDashboard.putBoolean("Neck Forward Limit Switch", m_robotContainer.getNeck().getForwardLimitSwitchState());
		SmartDashboard.putNumber("Neck Position", m_robotContainer.getNeck().getPosition());
		SmartDashboard.putNumber("Neck Enc Position", m_robotContainer.getNeck().getEncoderPosition());
		SmartDashboard.putBoolean("Neck IsMoving?", m_robotContainer.getNeck().isMoving());
		SmartDashboard.putBoolean("Neck IsHoming?", m_robotContainer.getNeck().isHoming());
		SmartDashboard.putNumber("Neck Target", m_robotContainer.getNeck().getTarget());
		SmartDashboard.putBoolean("Neck isStalled?", m_robotContainer.getNeck().isStalled());
		SmartDashboard.putBoolean("Neck isDown", m_robotContainer.getNeck().isDown());
		SmartDashboard.putBoolean("Neck isMidway", m_robotContainer.getNeck().isMidway());
		SmartDashboard.putBoolean("Neck isUp", m_robotContainer.getNeck().isUp());

		SmartDashboard.putBoolean("CoralRoller IsRolling?", m_robotContainer.getCoralRoller().isRolling());
		SmartDashboard.putBoolean("CoralRoller IsReleasing?", m_robotContainer.getCoralRoller().isReleasing());
		SmartDashboard.putBoolean("CoralRoller IsShooting?", m_robotContainer.getCoralRoller().isShooting());
		SmartDashboard.putBoolean("CoralRoller IsMoving?", m_robotContainer.getCoralRoller().isMoving());
		SmartDashboard.putNumber("CoralRoller Enc Position", m_robotContainer.getCoralRoller().getEncoderPosition());
		SmartDashboard.putNumber("CoralRoller Enc Velocity", m_robotContainer.getCoralRoller().getEncoderVelocity());
		SmartDashboard.putNumber("CoralRoller Rpm", m_robotContainer.getCoralRoller().getRpm());
		SmartDashboard.putNumber("CoralRoller Preset Rpm", m_robotContainer.getCoralRoller().getPresetRpm());
		SmartDashboard.putNumber("CoralRoller Target", m_robotContainer.getCoralRoller().getTarget());
		SmartDashboard.putBoolean("CoralRoller HasCoral", m_robotContainer.getCoralRoller().hasCoralEntered());

		SmartDashboard.putBoolean("AlgaeRoller IsRolling?", m_robotContainer.getAlgaeRoller().isRolling());
		SmartDashboard.putBoolean("AlgaeRoller IsReleasing?", m_robotContainer.getAlgaeRoller().isReleasing());
		SmartDashboard.putBoolean("AlgaeRoller IsShooting?", m_robotContainer.getAlgaeRoller().isShooting());
		SmartDashboard.putBoolean("AlgaeRoller IsMoving?", m_robotContainer.getAlgaeRoller().isMoving());
		SmartDashboard.putNumber("AlgaeRoller Enc Position", m_robotContainer.getAlgaeRoller().getEncoderPosition());
		SmartDashboard.putNumber("AlgaeRoller Enc Velocity", m_robotContainer.getAlgaeRoller().getEncoderVelocity());
		SmartDashboard.putNumber("AlgaeRoller Rpm", m_robotContainer.getAlgaeRoller().getRpm());
		SmartDashboard.putNumber("AlgaeRoller Preset Rpm", m_robotContainer.getAlgaeRoller().getPresetRpm());
		SmartDashboard.putNumber("AlgaeRoller Target", m_robotContainer.getAlgaeRoller().getTarget());

		SmartDashboard.putBoolean("Slider Limit Switch", m_robotContainer.getSlider().getForwardLimitSwitchState());
		SmartDashboard.putBoolean("Slider Reverse Limit Switch", m_robotContainer.getSlider().getReverseLimitSwitchState());
		SmartDashboard.putBoolean("Slider IsMoving?", m_robotContainer.getSlider().isMoving());
		SmartDashboard.putBoolean("Slider isStalled?", m_robotContainer.getSlider().isStalled());
		SmartDashboard.putBoolean("Slider isRetracted", m_robotContainer.getSlider().isRetracted());
		SmartDashboard.putBoolean("Slider isMidway", m_robotContainer.getSlider().isMidway());
		SmartDashboard.putBoolean("Slider isExtended", m_robotContainer.getSlider().isExtended());

		SmartDashboard.putBoolean("Hanger Forward Limit Switch", m_robotContainer.getHanger().getForwardLimitSwitchState());
		SmartDashboard.putBoolean("Hanger Reverse Limit Switch", m_robotContainer.getHanger().getReverseLimitSwitchState());
		SmartDashboard.putBoolean("Hanger IsMoving?", m_robotContainer.getHanger().isMoving());
		SmartDashboard.putNumber("Hanger Target", m_robotContainer.getHanger().getTarget());
		SmartDashboard.putBoolean("Hanger isStalled?", m_robotContainer.getHanger().isStalled());
		SmartDashboard.putBoolean("Hanger isDown", m_robotContainer.getHanger().isDown());
		SmartDashboard.putBoolean("Hanger isMidway", m_robotContainer.getHanger().isMidway());
		SmartDashboard.putBoolean("Hanger isUp", m_robotContainer.getHanger().isUp());
		
		SmartDashboard.putBoolean("Vision Correction", m_robotContainer.getVisionCorrectionEnablement());

		/*SmartDashboard.putString("Auton selected", m_robotContainer.getAutonChooser().getSelected());	
		SmartDashboard.putString("Game piece", m_robotContainer.getGamePieceChooser().getSelected());
		SmartDashboard.putString("Start position", m_robotContainer.getStartPositionChooser().getSelected());
		SmartDashboard.putString("Main target", m_robotContainer.getMainTargetChooser().getSelected());
		SmartDashboard.putString("Camera option", m_robotContainer.getCameraOptionChooser().getSelected());
		SmartDashboard.putString("Sonar option", m_robotContainer.getSonarOptionChooser().getSelected());
		SmartDashboard.putString("Release chosen", m_robotContainer.getReleaseChooser().getSelected());
		SmartDashboard.putString("Auton option", m_robotContainer.getAutonOptionChooser().getSelected());*/

		Optional<Alliance> alliance = DriverStation.getAlliance();
		Alliance allianceColor = alliance.isPresent() ? alliance.get() : Alliance.Blue;
		SmartDashboard.putString("Alliance color", allianceColor.toString());

		SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}
}
