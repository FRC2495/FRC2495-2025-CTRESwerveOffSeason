/**
 * 
 */
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;

//import java.util.Timer;
//import java.util.TimerTask;

//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
/*import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;*/
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.interfaces.*;
//import frc.robot.Ports;
import frc.robot.RobotContainer;

/**
 * The {@code Elevator} class contains fields and methods pertaining to the function of the elevator.
 */
public class Elevator extends SubsystemBase implements IElevator {

	
	// general settings
	public static final int TICKS_PER_REVOLUTION = 2048;
	public static final int LENGTH_OF_TRAVEL_REVS = 100;
	public static final int LENGTH_OF_MIDWAY_REVS = 50; // TODO adjust as needed (halve for Talon FX)
	public static final int LENGTH_OF_LEVEL_ONE_REVS = 20;//TODO FIX
	public static final int LENGTH_OF_LEVEL_TWO_REVS = 40;//25; 
	public static final int LENGTH_OF_LEVEL_THREE_REVS = 75;
	public static final int LENGTH_OF_LEVEL_FOUR_REVS = 100; 
	public static final int LENGTH_OF_ALGAE_LEVEL_TWO_REVS = 54; //TODO FIX
	public static final int LENGTH_OF_ALGAE_LEVEL_THREE_REVS = 77;//30; //TODO FIX
	


	static final double MAX_PCT_OUTPUT = 1.0;
	static final int WAIT_MS = 1000;
	static final int TIMEOUT_MS = 5000;

	static final int TALON_TIMEOUT_MS = 20;

	// move settings
	static final int PRIMARY_PID_LOOP = 0;
	
	static final int SLOT_0 = 0;
	
	static final double REDUCED_PCT_OUTPUT = 0.8; // 0.9;
	static final double HALF_PCT_OUTPUT = 0.5; // 0.9;
	static final double SUPER_REDUCED_PCT_OUTPUT = 0.3; // 0.9
	
	static final double MOVE_PROPORTIONAL_GAIN =  0.06;//0.6; //1.2 for SRX // TODO switch to 0.6 if required if switching to Talon FX (as encoder resolution is halved)
	static final double MOVE_INTEGRAL_GAIN = 0.0;
	static final double MOVE_DERIVATIVE_GAIN = 0.0;
	static final double MOVE_FEED_FORWARD_GAIN = 0.05;

	static final double MOVE_DOWN_PROPORTIONAL_GAIN =  0.02;//0.6; //1.2 for SRX // TODO switch to 0.6 if required if switching to Talon FX (as encoder resolution is halved)
	static final double MOVE_DOWN_INTEGRAL_GAIN = 0.0;
	static final double MOVE_DOWN_DERIVATIVE_GAIN = 0.0;
	static final double MOVE_DOWN_FEED_FORWARD_GAIN = 0.05;
	
	//static final int TALON_TICK_THRESH = 512; // 128; //256
	//static final double TICK_THRESH = 2048; // 512;
	//public static final double TICK_PER_100MS_THRESH = 64; // about a tenth of a rotation per second 
	static final double REV_THRESH = 1;
	public static final double RPS_THRESH = 0.3;
	
	private final static int MOVE_ON_TARGET_MINIMUM_COUNT= 20; // number of times/iterations we need to be on target to really be on target

	private final static int MOVE_STALLED_MINIMUM_COUNT = MOVE_ON_TARGET_MINIMUM_COUNT * 2 + 30; // number of times/iterations we need to be stalled to really be stalled

	TalonFX elevator; 
	TalonFX elevator_follower;

	TalonFXConfiguration elevatorConfig;
	TalonFXConfiguration elevator_followerConfig;

	DutyCycleOut elevatorStopOut = new DutyCycleOut(0);
	DutyCycleOut elevatorReducedOut = new DutyCycleOut(REDUCED_PCT_OUTPUT);

	PositionDutyCycle elevatorUpPosition = new PositionDutyCycle(LENGTH_OF_TRAVEL_REVS);
	PositionDutyCycle elevatorMidwayPosition = new PositionDutyCycle(LENGTH_OF_MIDWAY_REVS);
	PositionDutyCycle elevatorHomePosition = new PositionDutyCycle(0);
	PositionDutyCycle elevatorLevelOnePosition = new PositionDutyCycle(LENGTH_OF_LEVEL_ONE_REVS);
	PositionDutyCycle elevatorLevelTwoPosition = new PositionDutyCycle(LENGTH_OF_LEVEL_TWO_REVS);
	PositionDutyCycle elevatorLevelThreePosition = new PositionDutyCycle(LENGTH_OF_LEVEL_THREE_REVS);
	PositionDutyCycle elevatorLevelFourPosition = new PositionDutyCycle(LENGTH_OF_LEVEL_FOUR_REVS);
	PositionDutyCycle elevatorAlgaeLevelTwoPosition = new PositionDutyCycle(LENGTH_OF_ALGAE_LEVEL_TWO_REVS);
	PositionDutyCycle elevatorAlgaeLevelThreePosition = new PositionDutyCycle(LENGTH_OF_ALGAE_LEVEL_THREE_REVS);
	
	boolean isMoving;
	boolean isMovingUp;
	boolean isReallyStalled;

	double targetEncoder;

	private int onTargetCount; // counter indicating how many times/iterations we were on target 
	private int stalledCount; // counter indicating how many times/iterations we were stalled
	
	
	public Elevator(TalonFX elevator_in, TalonFX elevator_follower_in) {
		
		elevator = elevator_in;
		elevator_follower = elevator_follower_in;

		//elevator.getConfigurator().apply(new TalonFXConfiguration());
		//elevator_follower.getConfigurator().apply(new TalonFXConfiguration());

		// Both the Talon SRX and Victor SPX have a follower feature that allows the motor controllers to mimic another motor controller's output.
		// Users will still need to set the motor controller's direction, and neutral mode.
		// The method follow() allows users to create a motor controller follower of not only the same model, but also other models
		// , talon to talon, victor to victor, talon to victor, and victor to talon.
	
		
		// Mode of operation during Neutral output may be set by using the setNeutralMode() function.
		// As of right now, there are two options when setting the neutral mode of a motor controller,
		// brake and coast.
		elevatorConfig = new TalonFXConfiguration();
		elevator_followerConfig = new TalonFXConfiguration();

		elevator_follower.setControl(new Follower(elevator.getDeviceID(), true));

		//elevator.getConfigurator().apply(elevatorConfig);
		//elevator_follower.getConfigurator().apply(elevator_followerConfig);

		elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		elevator_followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		//elevator.setNeutralMode(NeutralMode.Brake);
		//elevator_follower.setNeutralMode(NeutralMode.Brake);
				
		// Sensor phase is the term used to explain sensor direction.
		// In order for limit switches and closed-loop features to function properly the sensor and motor has to be in-phase.
		// This means that the sensor position must move in a positive direction as the motor controller drives positive output.
		
		//elevator.setSensorPhase(true); // false for SRX // TODO switch to true if required if switching to Talon FX
		
		//Enable forward limit switches
		elevatorConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        elevatorConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        elevatorConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
		//drawer.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, TALON_TIMEOUT_MS);
		
		//Enable reverse limit switches
		elevatorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        elevatorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        elevatorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
		/*elevator_followerConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        elevator_followerConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        elevator_followerConfig.HardwareLimitSwitch.ReverseLimitEnable = true;*/
		//elevator.overrideLimitSwitchesEnable(true);
	
		// Motor controller output direction can be set by calling the setInverted() function as seen below.
		// Note: Regardless of invert value, the LEDs will blink green when positive output is requested (by robot code or firmware closed loop).
		// Only the motor leads are inverted. This feature ensures that sensor phase and limit switches will properly match the LED pattern
		// (when LEDs are green => forward limit switch and soft limits are being checked).
		elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // change value or comment out if needed
		elevator_followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		

		// Motor controllers that are followers can set Status 1 and Status 2 to 255ms(max) using setStatusFramePeriod.
		// The Follower relies on the master status frame allowing its status frame to be slowed without affecting performance.
		// This is a useful optimization to manage CAN bus utilization.

		//elevator_follower.setStatusFramePeriod(StatusFrame.Status_1_General, 255, TALON_TIMEOUT_MS);
		elevator_follower.optimizeBusUtilization();
		//elevator_follower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255, TALON_TIMEOUT_MS);

		//setPIDParameters();
		var slot0Configs = elevatorConfig.Slot0;
		slot0Configs.kV = 0 * 2048 / 1023 / 10;
		slot0Configs.kP = MOVE_PROPORTIONAL_GAIN; // * 2048 / 1023 / 10;
		slot0Configs.kI = MOVE_INTEGRAL_GAIN; // * 2048 / 1023 * 1000 / 10;
		slot0Configs.kD = MOVE_DERIVATIVE_GAIN; // * 2048 / 1023 / 1000 / 10;
		slot0Configs.kG = MOVE_FEED_FORWARD_GAIN;

		var slot1Configs = elevatorConfig.Slot1;
		slot1Configs.kV = 0 * 2048 / 1023 / 10;
		slot1Configs.kP = MOVE_DOWN_PROPORTIONAL_GAIN; // * 2048 / 1023 / 10;
		slot1Configs.kI = MOVE_DOWN_INTEGRAL_GAIN; // * 2048 / 1023 * 1000 / 10;
		slot1Configs.kD = MOVE_DOWN_DERIVATIVE_GAIN; // * 2048 / 1023 / 1000 / 10;
		slot1Configs.kG = MOVE_DOWN_FEED_FORWARD_GAIN;
		//slot0Configs.kS = SHOOT_DERIVATIVE_GAIN; //TODO change value
		elevator.getConfigurator().apply(slot0Configs, 0.050); // comment out if needed
		elevator.getConfigurator().apply(slot1Configs, 0.050); // comment out if needed
		elevator_follower.getConfigurator().apply(slot0Configs, 0.050); // comment out if needed
		elevator_follower.getConfigurator().apply(slot1Configs, 0.050); // comment out if needed
		// use slot 0 for closed-looping
 		//elevator.selectProfileSlot(SLOT_0, PRIMARY_PID_LOOP);
		
		// set peak output to max in case if had been reduced previously
		setPeakOutputs(MAX_PCT_OUTPUT);

	
		// Sensors for motor controllers provide feedback about the position, velocity, and acceleration
		// of the system using that motor controller.
		// Note: With Phoenix framework, position units are in the natural units of the sensor.
		// This ensures the best resolution possible when performing closed-loops in firmware.
		// CTRE Magnetic Encoder (relative/quadrature) =  4096 units per rotation		
		// FX Integrated Sensor = 2048 units per rotation
		
		//elevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PRIMARY_PID_LOOP, TALON_TIMEOUT_MS); // .CTRE_MagEncoder_Relative for SRX // TODO switch to FeedbackDevice.IntegratedSensor if switching to Talon FX
		elevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 

		// this will reset the encoder automatically when at or past the forward limit sensor
		/*elevator.configSetParameter(ParamEnum.eClearPositionOnLimitF, 1, 0, 0, TALON_TIMEOUT_MS);
		elevator.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, TALON_TIMEOUT_MS);*/
		elevatorConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
		elevatorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;

		StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = elevator.getConfigurator().apply(elevatorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

		for (int i = 0; i < 5; ++i) {
            status = elevator_follower.getConfigurator().apply(elevator_followerConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
		
		isMoving = false;
		isMovingUp = false;
		isReallyStalled = false;
		stalledCount = 0;
	}
	
	@Override
	public void periodic() {
		// Put code here to be run every loop
	}

	// This method should be called to assess the progress of a move
	public boolean tripleCheckMove() {
		if (isMoving) {
			
			//double error = elevator.getClosedLoopError(PRIMARY_PID_LOOP);
			double error = elevator.getClosedLoopError().getValueAsDouble();
			
			boolean isOnTarget = (Math.abs(error) < REV_THRESH);
			
			if (isOnTarget) { // if we are on target in this iteration 
				onTargetCount++; // we increase the counter
			} else { // if we are not on target in this iteration
				if (onTargetCount > 0) { // even though we were on target at least once during a previous iteration
					onTargetCount = 0; // we reset the counter as we are not on target anymore
					System.out.println("Triple-check failed (elevator moving).");
				} else {
					// we are definitely moving
				}
			}
			
			if (onTargetCount > MOVE_ON_TARGET_MINIMUM_COUNT) { // if we have met the minimum
				isMoving = false;
			}
			
			if (!isMoving) {
				System.out.println("You have reached the target (elevator moving).");
				//elevator.set(ControlMode.PercentOutput,0);
				if (isMovingUp)	{
					stop(); // adjust if needed
				} else {
					stop(); // adjust if needed
				}
			}
		}
		return isMoving; 
	}

	// return if drivetrain might be stalled
	public boolean tripleCheckIfStalled() {
		if (isMoving) {
			
			double velocity = getEncoderVelocity();
			
			boolean isStalled = (Math.abs(velocity) < RPS_THRESH);
			
			if (isStalled) { // if we are stalled in this iteration 
				stalledCount++; // we increase the counter
			} else { // if we are not stalled in this iteration
				if (stalledCount > 0) { // even though we were stalled at least once during a previous iteration
					stalledCount = 0; // we reset the counter as we are not stalled anymore
					System.out.println("Triple-check failed (detecting stall).");
				} else {
					// we are definitely not stalled
					
					//System.out.println("moving velocity : " + velocity);
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
		//return (int) (elevator.getSelectedSensorVelocity(PRIMARY_PID_LOOP));
		return (int) elevator.getVelocity().getValueAsDouble();
	}
	
	public void moveUp() {
		
		//setPIDParameters();
		System.out.println("Moving Up");
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		targetEncoder = elevatorUpPosition.Position;
		elevator.setControl(elevatorUpPosition.withSlot(0)); //fix

		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveToFirstLevel() {

		System.out.println("Moving to First Level");
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		targetEncoder = elevatorLevelOnePosition.Position;
		if (isGoingUp(targetEncoder)) {
			elevator.setControl(elevatorLevelOnePosition.withSlot(0)); //fix
		}
		else {
			elevator.setControl(elevatorLevelOnePosition.withSlot(1)); //fix
		}
		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}
	public void moveToSecondLevel() {
		
		//setPIDParameters();
		System.out.println("Moving to Second Level");
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		targetEncoder = elevatorLevelTwoPosition.Position;
		if (isGoingUp(targetEncoder)) {
			elevator.setControl(elevatorLevelTwoPosition.withSlot(0)); //fix
		}
		else {
			elevator.setControl(elevatorLevelTwoPosition.withSlot(1)); //fix
		}

		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;

	}
	public void moveToThirdLevel() {
		
		//setPIDParameters();
		System.out.println("Moving to Third Level");
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		targetEncoder = elevatorLevelThreePosition.Position;
		if (isGoingUp(targetEncoder)) {
			elevator.setControl(elevatorLevelThreePosition.withSlot(0)); //fix
		}
		else {
			elevator.setControl(elevatorLevelThreePosition.withSlot(1)); //fix
		}

		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveToFourthLevel() {
		
		//setPIDParameters();
		System.out.println("Moving to Fourth Level");
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		targetEncoder = elevatorLevelFourPosition.Position;
		//if (isGoingUp(targetEncoder)) {
		elevator.setControl(elevatorLevelFourPosition.withSlot(0)); //fix
		/* }
		else {
			elevator.setControl(elevatorLevelFourPosition.withSlot(1)); //fix
		}*/

		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveToAlgaeLevelTwo() {
		
		//setPIDParameters();
		System.out.println("Moving to Algae Level Two");
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		targetEncoder = elevatorAlgaeLevelTwoPosition.Position;
		if (isGoingUp(targetEncoder)) {
			elevator.setControl(elevatorAlgaeLevelTwoPosition.withSlot(0)); //fix
		}
		else {
			elevator.setControl(elevatorAlgaeLevelTwoPosition.withSlot(1)); //fix
		}
		
		isMoving = true;
		isMovingUp = false;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveToAlgaeLevelThree() {
		
		//setPIDParameters();
		System.out.println("Moving to Algae Level Three");
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		targetEncoder = elevatorAlgaeLevelThreePosition.Position;
		if (isGoingUp(targetEncoder)) {
			elevator.setControl(elevatorAlgaeLevelThreePosition.withSlot(0)); //fix
		}
		else {
			elevator.setControl(elevatorAlgaeLevelThreePosition.withSlot(1)); //fix
		}
		
		isMoving = true;
		isMovingUp = false;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public void moveMidway() {
		
		//setPIDParameters();
		System.out.println("Moving to Midway");
		setPeakOutputs(REDUCED_PCT_OUTPUT);

		//tac = -LENGTH_OF_MIDWAY_TICKS;
		
		targetEncoder = elevatorMidwayPosition.Position;
		if (isGoingUp(targetEncoder)) {
			elevator.setControl(elevatorMidwayPosition.withSlot(0)); //fix
		}
		else {
			elevator.setControl(elevatorMidwayPosition.withSlot(1)); //fix
		}
		
		isMoving = true;
		isMovingUp = true;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}
	
	public void moveDown() {
		
		//setPIDParameters();
		System.out.println("Moving Down");
		setPeakOutputs(SUPER_REDUCED_PCT_OUTPUT);

		targetEncoder = elevatorHomePosition.Position;
		if (getEncoderPosition() > elevatorLevelFourPosition.Position) {
			elevator.setControl(elevatorHomePosition.withSlot(0));
		}
		else {
			elevator.setControl(elevatorHomePosition.withSlot(1)); 
		}
		
		isMoving = true;
		isMovingUp = false;
		onTargetCount = 0;
		isReallyStalled = false;
		stalledCount = 0;
	}

	public double getEncoderPosition() {
		////return elevator.getSelectedSensorPosition(PRIMARY_PID_LOOP);
		return elevator.getPosition().getValueAsDouble();
	}
	
	public void stay() {	 		
		isMoving = false;		
		isMovingUp = false;
	}

	public synchronized void stop() {
		//elevator.set(ControlMode.PercentOutput, 0);
		//dutyCycleOut = 0;
		elevator.setControl(elevatorStopOut);
		
		setPeakOutputs(MAX_PCT_OUTPUT); // we undo what me might have changed
		
		isMoving = false;
		isMovingUp = false;
	}

	public boolean isGoingUp(double target) {
		if (target > getEncoderPosition()) {
			return true;
		}
		else {
			return false;
		}
	}
	
	
	// NOTE THAT THIS METHOD WILL IMPACT BOTH OPEN AND CLOSED LOOP MODES
	public void setPeakOutputs(double peakOutput)
	{
		elevatorConfig.MotorOutput.PeakForwardDutyCycle = peakOutput;
		elevatorConfig.MotorOutput.PeakReverseDutyCycle = -peakOutput;

	}
	
	public synchronized boolean isMoving() {
		return isMoving;
	}

	public synchronized boolean isMovingUp() {
		return isMovingUp;
	}

	public boolean isUp() {
		return Math.abs(getEncoderPosition()) > LENGTH_OF_TRAVEL_REVS * 9/10;
	}
	
	public boolean isDown() {
		return Math.abs(getEncoderPosition()) < LENGTH_OF_TRAVEL_REVS * 1/10;
	}
	
	public boolean isMidway() {
		return !isUp() && !isDown();
	}

	public boolean isDangerous() {
		return false;
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
			//elevator.set(ControlMode.PercentOutput, -joystick.getY()); // adjust sign if desired
			elevator.setControl(elevatorReducedOut.withOutput(-joystick.getY()));
		}
	}

	public void gamepadControl(XboxController gamepad)
	{
		if (!isMoving) // if we are already doing a move we don't take over
		{
			elevator.setControl(elevatorReducedOut.withOutput(+MathUtil.applyDeadband(-gamepad.getRightY(),RobotContainer.GAMEPAD_AXIS_THRESHOLD)*1.0/*0.7*/)); // adjust sign if desired
		}
	}

	public double getTarget() {
		return targetEncoder;
	}	

	public boolean getForwardLimitSwitchState() {
		//return elevator.getSensorCollection().isFwdLimitSwitchClosed()>0?true:false;
		return elevator.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
	}

	public boolean getReverseLimitSwitchState() {
		//return elevator.getSensorCollection().isRevLimitSwitchClosed()>0?true:false;
		return elevator.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
	}

	// MAKE SURE THAT YOU ARE NOT IN A CLOSED LOOP CONTROL MODE BEFORE CALLING THIS METHOD.
	// OTHERWISE THIS IS EQUIVALENT TO MOVING TO THE DISTANCE TO THE CURRENT ZERO IN REVERSE! 
	public void resetEncoder() {
		//elevator.set(ControlMode.PercentOutput, 0); // we stop AND MAKE SURE WE DO NOT MOVE WHEN SETTING POSITION
		elevator.setControl(elevatorStopOut);
		elevator.setPosition(0, TALON_TIMEOUT_MS);
		//elevator.setSelectedSensorPosition(0, PRIMARY_PID_LOOP, TALON_TIMEOUT_MS); // we mark the virtual zero
	}

}
