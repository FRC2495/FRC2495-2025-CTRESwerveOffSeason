// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;

import frc.robot.Ports;

/**
 * The {@code NoteSensor} class contains fields and methods pertaining to the function of the note sensor.
 */
public class CoralSensor
{
	Canandcolor coralSensor = new Canandcolor(Ports.Digital.CORAL_SENSOR);
	CanandcolorSettings coralSensorSettings = new CanandcolorSettings();

	public CoralSensor() {
		//this.digitalInput = new DigitalInput(port);
		coralSensor.setSettings(coralSensorSettings);
	}

	/**
	 * Returns the state of the note sensor.
	 *
	 * @return the current state of the note sensor.
	 */
	//public boolean isEnergized() {
		//return digitalInput.get();
	//}
}