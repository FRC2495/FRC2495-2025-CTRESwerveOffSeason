// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
/*import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;*/
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Ports;
import frc.robot.RobotContainer;
import frc.robot.interfaces.INeck;
import frc.robot.sensors.ThriftyEncoder;

/**
 * The {@code SwerveModule} class contains fields and methods pertaining to the function of a swerve module.
 */
public class Neck extends SubsystemBase implements INeck {
	private final SparkMax neck;
	private final SparkMaxConfig neckConfig;

	private final RelativeEncoder neckEncoder;

	private final SparkClosedLoopController neckClosedLoopController;

	private static final double NECK_P = 0.3;//0.1;
	private static final double NECK_I = 0;
	private static final double NECK_D = 0;

	private static final double MIN_OUTPUT = -1.0;
	private static final double MAX_OUTPUT = 1.0;

	private double setPosition;

	// general settings
	static final int TIMEOUT_MS = 15000;
	
	public static final double GEAR_RATIO = 3.0; // todo change if needed

	public static final int ANGLE_TO_CORAL_STATION_TICKS = 0; //TODO set proper value
	public static final int ANGLE_TO_ALGAE_REEF_TICKS = 0; //TODO set proper value
	public static final int ANGLE_TO_CORAL_REEF_TICKS = 10; //TODO set proper value
	public static final int ANGLE_TO_MIDWAY_TICKS = 90000;
	public static final int ANGLE_TO_TRAVEL_TICKS = 180000; // todo set proper value
	
	/*
	!!! VIRTUAL_HOME_OFFSET_TICKS is important for moving up,     !!!
	!!! if this is changed make sure to check to see if moveUp() works !!!
	(it's used as an error margin for moving up, since we can't reliably check when it's up)
	*/
	static final double VIRTUAL_HOME_OFFSET_TICKS = -4000; // position of virtual home compared to physical home
	
	static final double MAX_PCT_OUTPUT = 1.0; // ~full speed
	
	static final int TALON_TIMEOUT_MS = 20;
	public static final int TICKS_PER_REVOLUTION = 2048;
	

	// move settings
	static final int PRIMARY_PID_LOOP = 0;
	
	static final int SLOT_0 = 0;
	
	static final double REDUCED_PCT_OUTPUT = 0.7;
	static final double SUPER_REDUCED_PCT_OUTPUT = 0.5;
	static final double HOMING_PCT_OUTPUT = 0.9;//0.7;//0.5;//0.3; // ~homing speed
	
	static final double MOVE_PROPORTIONAL_GAIN = 0.06;
	static final double MOVE_INTEGRAL_GAIN = 0.0;
	static final double MOVE_DERIVATIVE_GAIN = 0.0;
	
	static final int TALON_TICK_THRESH = 256;
	static final double TICK_THRESH = 2048;	
	public static final double TICK_PER_100MS_THRESH = 256;
	
	private final static int MOVE_ON_TARGET_MINIMUM_COUNT= 20; // number of times/iterations we need to be on target to really be on target

	private final static int MOVE_STALLED_MINIMUM_COUNT = MOVE_ON_TARGET_MINIMUM_COUNT * 2 + 30; // number of times/iterations we need to be stalled to really be stalled
	
	// variables
	boolean isMoving;
	boolean isMovingUp;
	boolean isReallyStalled;
	boolean isHoming;

	double tac;

	private int onTargetCount; // counter indicating how many times/iterations we were on target
	private int stalledCount; // counter indicating how many times/iterations we were stalled	

	/**
	 * Constructs a SwerveModule and configures neck motor
	 * encoder, and PID controller.
	 */
	public Neck() {
		neckConfig = new SparkMaxConfig();

		neck = new SparkMax(Ports.CAN.NECK_MASTER, MotorType.kBrushed);

		neckConfig
			.inverted(true)
			.idleMode(IdleMode.kBrake)
			.closedLoopRampRate(0.6);
		neckConfig.encoder
			.positionConversionFactor(10);
		neckConfig.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.pid(NECK_P, NECK_I, NECK_D)
			.positionWrappingEnabled(false)
            .outputRange(MIN_OUTPUT, MAX_OUTPUT);
		neck.configure(neckConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		
		// Setup encoders and PID controllers for the neck SPARKS MAX.
		neckClosedLoopController = neck.getClosedLoopController();
		neckEncoder = neck.getEncoder();
		setPosition = getPosition();
	}

	@Override
	public void periodic() {
		// Put code here to be run every loop
		if (neck.getForwardLimitSwitch().isPressed() && getPosition() != 0.0) {
			resetEncoder();
		}
	}

	// homes the hinge
	// we go down slowly until we hit the limit switch.
	public void home() {
		//neck.set(ControlMode.PercentOutput,+HOMING_PCT_OUTPUT); // we start moving down
		neckClosedLoopController.setReference(+HOMING_PCT_OUTPUT, ControlType.kDutyCycle);
		
		isHoming = true;
	}

	// this method need to be called to assess the homing progress
	// (and it takes care of going to step 2 if needed)
	public boolean checkHome() {
		if (isHoming) {
			isHoming = !getForwardLimitSwitchState(); // we are not done until we reach the switch

			if (!isHoming) {
				System.out.println("You have reached the home.");
				neckClosedLoopController.setReference(0, ControlType.kDutyCycle); // turn power off
			}
		}

		return isHoming();
	}
	
	// This method should be called to assess the progress of a move
	/*public boolean tripleCheckMove() {
		if (isMoving) {
			
			double error = neckClosedLoopController.getClosedLoopError(PRIMARY_PID_LOOP);
			System.out.println("Neck moving error: " + Math.abs(error));
			
			boolean isOnTarget = (Math.abs(error) < TICK_THRESH);
			
			if (isOnTarget) { // if we are on target in this iteration 
				onTargetCount++; // we increase the counter
			} else { // if we are not on target in this iteration
				if (onTargetCount > 0) { // even though we were on target at least once during a previous iteration
					onTargetCount = 0; // we reset the counter as we are not on target anymore
					System.out.println("Triple-check failed (neck moving).");
				} else {
					// we are definitely moving
				}
			}
			
			if (onTargetCount > MOVE_ON_TARGET_MINIMUM_COUNT) { // if we have met the minimum
				isMoving = false;
			}
			
			if (!isMoving) {
				System.out.println("You have reached the target (neck moving).");
				//neck.set(ControlMode.PercentOutput,0);
				if (isMovingUp) {
					stay();
				} else {
					stop();
					//stay();
				}
			}
		}
		return isMoving; 
	}*/

	// return if drivetrain might be stalled
	public boolean tripleCheckIfStalled() {
		if (isMoving) {
			
			double velocity = getEncoderVelocity();
			
			boolean isStalled = (Math.abs(velocity) < TICK_PER_100MS_THRESH);
			
			if (isStalled) { // if we are stalled in this iteration 
				stalledCount++; // we increase the counter
			} else { // if we are not stalled in this iteration
				if (stalledCount > 0) { // even though we were stalled at least once during a previous iteration
					stalledCount = 0; // we reset the counter as we are not stalled anymore
					System.out.println("Triple-check failed (detecting stall).");
				} else {
					// we are definitely not stalled
					
					System.out.println("moving velocity : " + velocity);
				}
			}
			
			if (isMoving && stalledCount > MOVE_STALLED_MINIMUM_COUNT) { // if we have met the minimum
				isReallyStalled = true;
			}
					
			if (isReallyStalled) {
				System.out.println("WARNING: Stall detected!");
				stop(); // WE STOP IF A STALL IS DETECTED				 
			}
		}
		
		return isReallyStalled;
	}

	public int getEncoderVelocity() {
		return (int) (neckEncoder.getVelocity());
	}
	
	public void moveUp() {	

		System.out.println("Moving Up");

		tac = -ANGLE_TO_TRAVEL_TICKS;
		neckClosedLoopController.setReference(tac, ControlType.kPosition);
		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveCustom(double encoder_ticks) {	

		System.out.println("Moving Custom");

		tac = encoder_ticks;
		neckClosedLoopController.setReference(tac, ControlType.kPosition);
		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveToCoralStation() {	

		System.out.println("Moving to Coral Station");

		tac = -ANGLE_TO_CORAL_STATION_TICKS;
		neckClosedLoopController.setReference(tac, ControlType.kPosition);
		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveToCoralReef() {	

		System.out.println("Moving to Coral Station");

		tac = -ANGLE_TO_CORAL_REEF_TICKS;
		neckClosedLoopController.setReference(tac, ControlType.kPosition);
		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveToAlgaeReef() {	

		System.out.println("Moving to Coral Station");

		tac = -ANGLE_TO_ALGAE_REEF_TICKS;
		neckClosedLoopController.setReference(tac, ControlType.kPosition);
		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveMidway() {	

		System.out.println("Moving to Midway");
	
		tac = -ANGLE_TO_MIDWAY_TICKS;
		neckClosedLoopController.setReference(tac, ControlType.kPosition);
		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveDown() {

		System.out.println("Moving Down");

		tac = -VIRTUAL_HOME_OFFSET_TICKS;
		neckClosedLoopController.setReference(tac, ControlType.kPosition);
		
		isMoving = true;
		isMovingUp = false;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	//TODO check to see if this translates from talon to rev?
	public double getPosition() {
		return neckEncoder.getPosition() * GEAR_RATIO / TICKS_PER_REVOLUTION;
	}

	public double getEncoderPosition() {
		return neckEncoder.getPosition();
	}

	public void stay() {	 		
		isMoving = false;		
		isMovingUp = false;
		isHoming = false;
	}
	
	public void stop() {	 

		neckClosedLoopController.setReference(0, ControlType.kDutyCycle);
		
		isMoving = false;
		isMovingUp = false;	
		isHoming = false;	
	}	

	public boolean isHoming() {
		return isHoming;
	}

	public boolean isMoving() {
		return isMoving;
	}

	public boolean isMovingUp() {
		return isMovingUp;
	}
	
	public boolean isUp() {
		return Math.abs(getEncoderPosition()) > ANGLE_TO_TRAVEL_TICKS * 2/3;
	}
	
	public boolean isDown() {
		return Math.abs(getEncoderPosition()) < ANGLE_TO_TRAVEL_TICKS * 1/3;
	}
	
	public boolean isMidway() {
		return !isUp() && !isDown();
	}

	public boolean isDangerous() {
		return isUp();
	}

	// return if stalled
	public boolean isStalled() {
		return isReallyStalled;
	}	
	
	// for debug purpose only
	public void joystickControl(Joystick joystick)
	{
		if (!isMoving) // if we are already doing a move we don't take over
		{
			neckClosedLoopController.setReference(-joystick.getY(), ControlType.kDutyCycle);
			//neck.set(ControlMode.PercentOutput, -joystick.getY());
		}
	}	

	public void gamepadControl(XboxController gamepad)
	{
		if (!isMoving) // if we are already doing a move we don't take over
		{
			neckClosedLoopController.setReference(+MathUtil.applyDeadband(gamepad.getRightY(),RobotContainer.GAMEPAD_AXIS_THRESHOLD)*0.6, ControlType.kDutyCycle);
			//neck.set(ControlMode.PercentOutput, +MathUtil.applyDeadband(gamepad.getRightY(),RobotContainer.GAMEPAD_AXIS_THRESHOLD)*0.6); // adjust sign if desired
		}
	}

	public double getTarget() {
		return tac;
	}

	// returns the state of the limit switch
	public boolean getForwardLimitSwitchState() {
		return neck.getForwardLimitSwitch().isPressed();
	}

	public boolean getReverseLimitSwitchState() {
		return neck.getReverseLimitSwitch().isPressed();
	}

	// MAKE SURE THAT YOU ARE NOT IN A CLOSED LOOP CONTROL MODE BEFORE CALLING THIS METHOD.
	// OTHERWISE THIS IS EQUIVALENT TO MOVING TO THE DISTANCE TO THE CURRENT ZERO IN REVERSE! 
	public void resetEncoder() {
		neckEncoder.setPosition(0.0);
	}


}
