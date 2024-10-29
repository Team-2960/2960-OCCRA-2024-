package frc.robot.Auton.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class shootToTime extends Command {
    double shootPower;
    double time;
    boolean setHandoff;
    
    Timer timer = new Timer();

    Shooter shooter = Shooter.getInstance();
    Intake intake = Intake.getInstance();

    public shootToTime(double shootPower, boolean setHandoff, double time){
        this.shootPower = shootPower;
        this.setHandoff = setHandoff;
        this.time = time;
    }

    @Override
    public void initialize(){
        timer.restart();
    }

    @Override
    public void execute(){
        shooter.setShooter(shootPower);
        intake.setHandoff(setHandoff);

    }

    @Override
    public void end(boolean interrupt){
        shooter.setShooter(0);
        intake.setHandoff(false);
    }

    @Override
    public boolean isFinished(){
        return timer.get() >= time;
    }

}
