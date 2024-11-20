package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    public static Intake intake = null;

    private CANSparkMax intakeMotor;
    private CANSparkMax handoffMotor;
    private DoubleSolenoid intakeExtender;
    private DigitalInput intakePhotoeye;

    private boolean intakeExtState = false;
    private boolean intakePhotoeyeState = false;
    
    private boolean enablePhotoeye = true;

    private Intake(){
        intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
        handoffMotor = new CANSparkMax(Constants.handoffMotorID, MotorType.kBrushless);
        intakeExtender = new DoubleSolenoid(20, PneumaticsModuleType.CTREPCM, Constants.intakeExtFor, Constants.intakeExtRev);
        intakePhotoeye = new DigitalInput(Constants.intakePhotoeyeID);
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
        if (intakePhotoeye.get() && enablePhotoeye){
            intakeMotor.set(0);
        }else if (setIntake){
            intakeMotor.set(Constants.intakePower);
        }else{
            intakeMotor.set(0);
        }
    }

    public void setIntakeSpeed(double speed){
        intakeMotor.set(speed);
    }

    /**
     * Sets handoff power
     * @param setHandoff
     */
    public void setHandoff(boolean setHandoff, boolean override){
        if(override){
            handoffMotor.set(Constants.handoffPower);
        }
        else if(intakePhotoeye.get() && enablePhotoeye){
            handoffMotor.set(0);
        }
        else if(setHandoff){
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
    
    public void setHandoffSpeed(double speed){
        handoffMotor.set(speed);
    }

    /**
     * Sets intake extend position
     * @param setIntake
     */
    public void setIntakeExt(boolean state){
        if(state){
            intakePhotoeyeState = true;
        }else{
            intakePhotoeyeState = false;
        }
    }

    public void updateIntake(){
        if (intakePhotoeye.get() && enablePhotoeye){
            intakeExtender.set(Value.kReverse);
            intakePhotoeyeState = false;
        }else if(intakePhotoeyeState){
            intakeExtender.set(Value.kForward);
        }else{
            intakeExtender.set(Value.kReverse);
        }
    }

    public boolean isBallPresent(){
        return intakePhotoeye.get();
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
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Intake State", intakeExtState);
        SmartDashboard.putBoolean("FwsSolenoidDisabled?", intakeExtender.isFwdSolenoidDisabled());
        SmartDashboard.putBoolean("RevSolenoidDisabled?", intakeExtender.isRevSolenoidDisabled());
        SmartDashboard.putBoolean("intakePhotoeye", intakePhotoeye.get());
        updateIntake();
    }
    
    
    public static Intake getInstance(){
        if (intake == null){
            intake = new Intake();
        }
        return intake;
    }
}
