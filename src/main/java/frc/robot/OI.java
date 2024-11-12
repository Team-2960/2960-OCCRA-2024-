package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Auton.Commands.Drive.driveToDistance;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class OI extends SubsystemBase {
    
    private Intake intake = Intake.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private static OI oi = null;
    private Joystick gamepad1;
    private Joystick gamepad2;

    private double leftInitial;
    private double rightInitial;


    
    private OI() {
        gamepad1 = new Joystick(0);
        gamepad2 = new Joystick(1);

        leftInitial = 0;
        rightInitial = 0;
    }

//Driver
    private void updateDrive(){
        Drive drive = Drive.getInstance();
        drive.setPower(MathUtil.applyDeadband(gamepad1.getRawAxis(1), 0.1), 
            MathUtil.applyDeadband(gamepad1.getRawAxis(5), 0.1), 
            gamepad1.getRawButton(6));
        if (gamepad1.getRawButton(1)){
            drive.presetPosition(new Pose2d());
        }
        if (gamepad1.getRawButton(2)){
            drive.setSpeeds(new DifferentialDriveWheelSpeeds(1, 1));
        }
        else if (gamepad1.getRawButton(3)){
            drive.driveDistance(1, 1, leftInitial, rightInitial);
        }
        else if (gamepad1.getRawButton(4)){
            rightInitial = drive.getRightPosition();
            leftInitial = drive.getLeftPosition();
        }
        else if (gamepad1.getRawButton(5)){
            drive.driveToRotation(0);
        }

        if (gamepad1.getPOV() == 0){
            drive.setDriveIdleMode(IdleMode
            .kCoast);
        }else if(gamepad1.getPOV() == 180){
            drive.setDriveIdleMode(IdleMode.kBrake);
        }
    }


//Operator
    private void updateIntake(){
        if (gamepad2.getPOV() == 0){
            intake.setIntakeExt(true);
        }else if(gamepad2.getPOV() == 180){
            intake.setIntakeExt(false);
        }

        if(gamepad2.getRawButton(6)){
            intake.setReverseHandoff(true);
        }else if(gamepad2.getRawButton(5)){
            intake.setIntake(true);
            intake.setHandoff(true);

        }else{
            intake.setIntake(false);
            intake.setHandoff(false);
        }
        //intake.setReverseHandoff(gamepad2.getRawButton(6));

        //intake.setIntake(gamepad2.getRawButton(5));
        //intake.setHandoff(gamepad2.getRawButton(5));


    }

    private void updateShooter(){
        if (gamepad2.getRawAxis(3) > 0.1){
            shooter.enableShooter(true);
        } else if(gamepad2.getRawAxis(2) > 0.1){
            shooter.revShootSpeed(true);
        } else{
            shooter.enableShooter(false);
            shooter.revShootSpeed(false);
        }
    }

    @Override
    public void periodic(){
        if(DriverStation.isTeleop()){
            updateDrive();
            updateIntake();
            updateShooter();
        }
        SmartDashboard.putNumber("Left Position", Drive.getInstance().getLeftPosition() - leftInitial);
        SmartDashboard.putNumber("Right Position", Drive.getInstance().getRightPosition() - rightInitial);
    }


    public static OI getInstance(){
        if(oi == null){
            oi = new OI();
        }
        return oi;
    }
}
