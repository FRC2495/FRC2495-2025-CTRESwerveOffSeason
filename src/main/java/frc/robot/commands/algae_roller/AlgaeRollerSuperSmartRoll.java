package frc.robot.commands.algae_roller;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.sensors.*;
import frc.robot.subsystems.*;


public class AlgaeRollerSuperSmartRoll extends SequentialCommandGroup {

    public AlgaeRollerSuperSmartRoll(AlgaeRoller algae_roller, NoteSensor notesensor, NoteSensor noteSensorTwo){

        addCommands(

			new AlgaeRollerRollLowRpmUntilNoteSensed(algae_roller, notesensor, noteSensorTwo),

            new WaitCommand(0.5), // we wait for things to settle down

            new AlgaeRollerReleaseShortDistance(algae_roller)
            
        ); 
  
    }


}