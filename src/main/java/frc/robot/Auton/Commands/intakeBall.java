package frc.robot.Auton.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class intakeBall extends Command {  
    
    @Override
    public void initialize(){
        Intake.getInstance().runAllIntake(true, true);
    }

    @Override
    public boolean isFinished(){
        //boolean finished = Intake.getInstance().isBallReady();
        return true;
    }
}
