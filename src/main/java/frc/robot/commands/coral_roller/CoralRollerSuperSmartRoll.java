package frc.robot.commands.coral_roller;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.sensors.*;
import frc.robot.subsystems.*;


public class CoralRollerSuperSmartRoll extends SequentialCommandGroup {

    public CoralRollerSuperSmartRoll(CoralRoller coral_roller, NoteSensor notesensor, NoteSensor noteSensorTwo){

        addCommands(

			new CoralRollerRollLowRpmUntilNoteSensed(coral_roller, notesensor, noteSensorTwo),

            new WaitCommand(0.5), // we wait for things to settle down

            new CoralRollerReleaseShortDistance(coral_roller)
            
        ); 
  
    }


}