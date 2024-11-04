package frc.robot.Auton.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class setIntakeState extends Command{
    double handoffPower;
    double intakePower;

    Intake intake = Intake.getInstance();
    

    public setIntakeState(double handoffPower, double intakePower){
        this.handoffPower = handoffPower;
        this.intakePower = intakePower;
    }
    
    @Override
    public void initialize(){
        intake.setHandoffSpeed(handoffPower);
        intake.setIntakeSpeed(intakePower);
    }

    @Override
    public void end(boolean interrupt){
        intake.setHandoffSpeed(0);
        intake.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
