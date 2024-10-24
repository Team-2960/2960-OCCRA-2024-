package frc.robot.subsystems;



import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.gyrosetup.BNO055;


public class Drive extends SubsystemBase { 
    private static Drive drive = null;
    private CANSparkMax leftMotor1;
    private CANSparkMax leftMotor2;
    private CANSparkMax rightMotor1;
    private CANSparkMax rightMotor2;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private Encoder encoder;

    private static BNO055 imu;
    private BNO055.CalData calibration;

    private PIDController leftPID;
    private PIDController rightPID;

    private DifferentialDriveKinematics kinematics;

    private DifferentialDrivePoseEstimator poseEstimator;

    private SimpleMotorFeedforward feedforward;

    private PIDController leftPidController;
    private PIDController rightPidController;

    public AutoBuilder autoBuilder;

    public int number = 4;

    

    private Drive(){
        //Initialize Drive Motors
        leftMotor1 = new CANSparkMax(Constants.leftMotor1ID, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(Constants.leftMotor2ID, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(Constants.rightMotor1ID, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(Constants.rightMotor2ID, MotorType.kBrushless);

        //leftMotor2.follow(leftMotor1);
        //rightMotor2.follow(rightMotor1);

        //Invert One side of the chassis
        leftMotor1.setInverted(true);
        leftMotor2.setInverted(true);

        leftEncoder = leftMotor1.getEncoder(Type.kHallSensor, 42);
        rightEncoder = rightMotor1.getEncoder(Type.kHallSensor, 42);
        imu = BNO055.getInstance(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
            BNO055.vector_type_t.VECTOR_EULER);

        kinematics = new DifferentialDriveKinematics(Constants.trackWidth);
    
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(imu.getHeading()), 
            leftEncoder.getPosition() * Constants.driveGearRatio * Constants.wheelCirc, 
            rightEncoder.getPosition() * Constants.driveGearRatio * Constants.wheelCirc, 
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        feedforward = new SimpleMotorFeedforward(1, 3);
        leftPidController = new PIDController(1, 0, 0);
        rightPidController = new PIDController(1, 0, 0);

        autoBuilder = new AutoBuilder();
        AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::presetPosition, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public void setPower(double leftPower, double rightPower, boolean turbo){
        if (turbo){
            rightMotor1.set(rightPower * Constants.turboMode);
            rightMotor2.set(rightPower * Constants.turboMode);
            leftMotor1.set(leftPower * Constants.turboMode);
            leftMotor2.set(leftPower * Constants.turboMode);
        }else{
            rightMotor1.set(rightPower * Constants.normalMode);
            rightMotor2.set(rightPower * Constants.normalMode);
            leftMotor1.set(leftPower * Constants.normalMode);
            leftMotor2.set(leftPower * Constants.normalMode);
        }
    }

    /**
     * Sets the voltage of the drive motors
     * @param leftVoltage
     * @param rightVoltage
     */
    public void setVoltage(double leftVoltage, double rightVoltage){
        rightMotor1.setVoltage(rightVoltage);
        rightMotor2.setVoltage(rightVoltage);
        leftMotor1.setVoltage(leftVoltage);
        leftMotor2.setVoltage(leftVoltage);
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedforward =
            feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward =
            feedforward.calculate(speeds.rightMetersPerSecond);
    
        final double leftOutput =
            leftPidController.calculate(leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
        final double rightOutput =
            rightPidController.calculate(rightEncoder.getVelocity(), speeds.rightMetersPerSecond);
        leftMotor1.setVoltage(leftOutput + leftFeedforward);
        rightMotor1.setVoltage(rightOutput + rightFeedforward);
    }

    public void drive(ChassisSpeeds chassisSpeeds){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        setSpeeds(wheelSpeeds);
    }

    public void presetPosition(Pose2d pose2d){
        poseEstimator.resetPosition(Rotation2d.fromDegrees(imu.getHeading()), 
            new DifferentialDriveWheelPositions(leftEncoder.getPosition() * Constants.wheelCirc, 
            rightEncoder.getPosition() * Constants.wheelCirc),
            pose2d);
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getCurrentSpeeds(){
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds((leftEncoder.getVelocity() * Constants.wheelCirc * Constants.driveGearRatio)/60, 
            (rightEncoder.getVelocity() * Constants.wheelCirc * Constants.driveGearRatio)/60));
    }


    @Override
    public void periodic(){
        //Motor Values
        SmartDashboard.putNumber("LMotor1 Pos", leftEncoder.getPosition());
        SmartDashboard.putNumber("RMotor1 Pos", rightEncoder.getPosition());

        SmartDashboard.putNumber("LMotor1 Velocity", leftEncoder.getVelocity());
        SmartDashboard.putNumber("RMotor1 Velocity", rightEncoder.getPosition());

        SmartDashboard.putNumber("LeftMotor1 V", leftMotor1.getBusVoltage());
        SmartDashboard.putNumber("LeftMotor2 V", leftMotor2.getBusVoltage());
        //Gyroscope
        SmartDashboard.putBoolean("Gyro Initialized", imu.isInitialized());
        SmartDashboard.putBoolean("Gyro Calibrated", imu.isCalibrated());
        SmartDashboard.putBoolean("Gyro Present", imu.isSensorPresent());
        SmartDashboard.putNumberArray("Gyro Vector", imu.getVector());
        SmartDashboard.putNumber("Gyro Heading", imu.getHeading());
    }

    public static Drive getInstance(){
        if(drive == null){
            drive = new Drive();
        }
        return drive;
    }
    
}