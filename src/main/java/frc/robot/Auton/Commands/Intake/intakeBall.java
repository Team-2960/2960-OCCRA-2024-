package frc.robot.Auton.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class intakeBall extends Command {  
    Intake intake = Intake.getInstance();

    @Override
    public void initialize(){
        intake.setIntake(true);
        intake.setHandoff(true, false);
    }

    @Override
    public void end(boolean interrupt){
        intake.setIntake(false);
        intake.setHandoff(false, false);
    }

    @Override
    public boolean isFinished(){
        //boolean finished = Intake.getInstance().isBallReady();
        return intake.isBallPresent();
    }
}
