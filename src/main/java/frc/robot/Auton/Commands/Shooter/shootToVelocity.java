package frc.robot.Auton.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class shootToVelocity extends Command {
    double shootSpeed;
    boolean setHandoff;
    double tolerance;
    double time;

    Timer timer = new Timer();

    Shooter shooter = Shooter.getInstance();
    Intake intake = Intake.getInstance();

    public shootToVelocity(double shootSpeed, boolean setHandoff, double tolerance, double time){
        this.shootSpeed = shootSpeed;
        this.setHandoff = setHandoff;
        this.tolerance = tolerance;
        this.time = time;
    }

    @Override
    public void initialize(){
        timer.restart();
    }

    @Override
    public void execute(){
        shooter.setShooterSpeed(shootSpeed);
        if (Math.abs(shootSpeed - shooter.getShooterVelocity()) <= tolerance){
            intake.setHandoff(setHandoff, true);
        }
    }

    @Override
    public void end(boolean interrupt){
        shooter.setShooterSpeed(0);
        intake.setHandoff(false, false);
    }

    @Override
    public boolean isFinished(){
        if (setHandoff){
            return timer.advanceIfElapsed(time + 0.5);
        }else{
            return timer.advanceIfElapsed(time);
        }
    }

}
