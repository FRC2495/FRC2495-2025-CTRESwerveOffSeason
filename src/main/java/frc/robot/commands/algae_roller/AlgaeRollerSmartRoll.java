package frc.robot.commands.algae_roller;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.sensors.*;
import frc.robot.subsystems.*;

public class AlgaeRollerSmartRoll extends SequentialCommandGroup {

    public AlgaeRollerSmartRoll(AlgaeRoller algae_roller, NoteSensor notesensor, NoteSensor noteSensorTwo){

        addCommands(

			new AlgaeRollerRollUntilNoteSensed(algae_roller, notesensor, noteSensorTwo),

            new AlgaeRollerTimedRelease(algae_roller, 0.1)
            
        ); 
  
    }


}