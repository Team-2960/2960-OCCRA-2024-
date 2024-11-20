package frc.robot.Auton.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class shootToTime extends Command {
    double shootSpeed;
    double time;
    boolean setHandoff;
    
    Timer timer = new Timer();

    Shooter shooter = Shooter.getInstance();
    Intake intake = Intake.getInstance();

    public shootToTime(double shootSpeed, boolean setHandoff, double time){
        this.shootSpeed = shootSpeed;
        this.setHandoff = setHandoff;
        this.time = time;
    }

    @Override
    public void initialize(){
        timer.restart();
    }

    @Override
    public void execute(){
        shooter.setShooterSpeed(shootSpeed);
        intake.setHandoff(setHandoff, false);

    }

    @Override
    public void end(boolean interrupt){
        shooter.setShooterSpeed(0);
        intake.setHandoff(false, false);
    }

    @Override
    public boolean isFinished(){
        return timer.get() >= time;
    }

}
