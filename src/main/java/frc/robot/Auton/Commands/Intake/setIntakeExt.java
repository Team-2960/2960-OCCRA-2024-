package frc.robot.Auton.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class setIntakeExt extends Command{
    boolean intakeExt;
    Intake intake = Intake.getInstance();
    
    public setIntakeExt(boolean intakeExt){
        this.intakeExt = intakeExt;
    }

    @Override
    public void initialize(){
        intake.setIntakeExt(intakeExt);
    }

    @Override
    public boolean isFinished(){
        return true;
    }


}
