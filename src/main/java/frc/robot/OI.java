package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class OI extends SubsystemBase {
    private Drive drive = Drive.getInstance();
    private Intake intake = Intake.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private static OI oi = null;
    private Joystick gamepad1;
    private Joystick gamepad2;

    
    private OI() {
        gamepad1 = new Joystick(0);
        gamepad2 = new Joystick(1);
    }

    private void updateDrive(){
        drive.setPower(gamepad1.getRawAxis(2), gamepad1.getRawAxis(3));
    }

    private void updateIntake(){
        boolean isIntakeExt = false;
        if (gamepad2.getPOV() == 0){
            isIntakeExt = true;
        }else if(gamepad2.getPOV() == 180){
            isIntakeExt = false;
        }
        intake.setIntake(gamepad2.getRawAxis(3));
        intake.setHandoff(gamepad2.getRawButton(6));
        
        intake.setIntakeExt(isIntakeExt);
    }

    private void updateShooter(){
        shooter.setShooter(gamepad2.getRawAxis(2));
    }
    
    public void periodic(){
        updateDrive();
        updateIntake();
    }

    public static OI getInstance(){
        if(oi == null){
            oi = new OI();
        }
        return oi;
    }
}
