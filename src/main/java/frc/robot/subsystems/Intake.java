package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    public static Intake intake = null;

    private CANSparkMax intakeMotor;
    private CANSparkMax handoffMotor;
    private DoubleSolenoid intakeExtender;
    //private DigitalInput intakePhotoeye;
    

    private Intake(){
        intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
        handoffMotor = new CANSparkMax(Constants.handoffMotorID, MotorType.kBrushless);
        intakeExtender = new DoubleSolenoid(Constants.intakeExtModuleID ,PneumaticsModuleType.CTREPCM , Constants.intakeExtFor, Constants.intakeExtRev);
        //intakePhotoeye = new DigitalInput(Constants.intakePhotoeyeID);

        intakeMotor.setIdleMode(IdleMode.kBrake);
        handoffMotor.setIdleMode(IdleMode.kBrake);
        handoffMotor.setInverted(true);
        intakeMotor.setInverted(true);
    }
    /**
     * Sets intake power
     * @param setIntake
     */
    public void setIntake(boolean setIntake){
        if (setIntake){
            intakeMotor.set(Constants.intakePower);
        }else{
            intakeMotor.set(0);
        }
    }

    
    
    /**
     * Sets handoff power
     * @param setHandoff
     */
    public void setHandoff(boolean setHandoff){
        if(setHandoff){
            handoffMotor.set(Constants.handoffPower);
        } else{
            handoffMotor.set(0);
        }
        
    }

    /**
     * Reverses handoff direction
     * @param setReverseIntake
     */
    
    public void setReverseHandoff(boolean setReverseHandoff){
        if (setReverseHandoff){
            handoffMotor.set(Constants.revHandoffPower);
        }else{
            handoffMotor.set(0);
        }
    }
    
    /**
     * Sets intake extend position
     * @param setIntake
     */
    public void setIntakeExt(boolean setIntake){
        if(setIntake == true){
            intakeExtender.set(Value.kForward);
        }else{
            intakeExtender.set(Value.kReverse);
        }
    }

    /**
     * Runs both intake and handoff at the same time
     * @param setIntake
     * @param setHandoff
     */
    public void runAllIntake(boolean setIntake, boolean setHandoff){
        if (setIntake){
            intakeMotor.set(Constants.intakePower);
        }else{
            intakeMotor.set(0);
        }

        if (setHandoff){
            handoffMotor.set(Constants.handoffPower);
        }else{
            handoffMotor.set(0);
        }
    }

    /* 
    public boolean isBallReady(){
        return intakePhotoeye.get();
    }
    */
    
    
    public static Intake getInstance(){
        if (intake == null){
            intake = new Intake();
        }
        return intake;
    }
}
