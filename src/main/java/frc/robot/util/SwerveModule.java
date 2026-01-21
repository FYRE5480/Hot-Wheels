package frc.robot.util;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/** A wrapper class to encapsulate a swerve module. */
public class SwerveModule extends SubsystemBase {

    // the index of this swerve module
    private final int index;
    
    private final SparkMax swerveMotor;
    private final SparkMaxConfig swerveConfig;
    private final RelativeEncoder swerveEncoder;

    private final SparkMax driveMotor;
    private final SparkMaxConfig driveConfig;
    private final RelativeEncoder driveEncoder;

    private final DutyCycleEncoder swerveEncoderAbsolute;

    private final SparkClosedLoopController swervePID;

    public SwerveModuleState currentState;

    double lastMotorSpeed;
    double lastMotorSetTime;

    boolean absoluteEnoderConnected;

    boolean errorSent = false;

    /** Represents a speed and angle for a swerve module. */
    public class SwerveAngleSpeed {
        double targetAngle;
        int multiplier;
    }

    /**
     * Initializes a new swerve module with the given index.

     * @param index - the index of the module in order of the defined placement
     */
    public SwerveModule(int index) {

        this.index = index;

        swerveMotor = new SparkMax(
            DriveConstants.swerveMotorPorts[index], 
            SparkLowLevel.MotorType.kBrushless
        );
        swerveEncoder = swerveMotor.getEncoder();
        swerveEncoderAbsolute = new DutyCycleEncoder(DriveConstants.encoders[index]);
        swerveConfig = new SparkMaxConfig();

        driveMotor = new SparkMax(
            DriveConstants.driveMotorPorts[index],
            SparkLowLevel.MotorType.kBrushless
        );
        driveEncoder = driveMotor.getEncoder();
        driveConfig = new SparkMaxConfig();

        swervePID = swerveMotor.getClosedLoopController();


        // configure the swerve motor
        swerveConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(15);

        swerveConfig.encoder
            .positionConversionFactor(DriveConstants.swerveRotationToDegrees)
            .velocityConversionFactor(DriveConstants.swerveRotationToDegrees);

        swerveConfig.signals
            .primaryEncoderPositionPeriodMs(20);

        swerveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                DriveConstants.swerveP,
                DriveConstants.swerveI,
                DriveConstants.swerveD
            ).iZone(0)
            .outputRange(-1, 1);

        // configure the drive motor
        driveConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);

        driveConfig.encoder
            .positionConversionFactor(DriveConstants.driveMotorToWheel)
            .velocityConversionFactor(DriveConstants.driveMotorToWheel);

        driveConfig.signals
            .primaryEncoderPositionPeriodMs(50);


        swerveMotor.configure(
            swerveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(
            driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        double relativeZero = getAbsolutePosition();

        REVLibError error = swerveEncoder.setPosition(relativeZero - DriveConstants.absoluteOffsets[index]);    
        
        if (error.equals(REVLibError.kOk)) System.out.println("Swerve Module " + index + " is initialized!");

        currentState = new SwerveModuleState();
    }

    @Override
    public void periodic() {
        absoluteEnoderConnected = getAbsolutePosition() != 0 && getAbsolutePosition() != 360;
    }

    public void setSwerveReference(double value) {
        swervePID.setSetpoint(value, ControlType.kPosition);
    }


    public void setSwerveEncoder(double position) {
        swerveEncoder.setPosition(position);
    }

    public double getSwervePosition() {
        return swerveEncoder.getPosition();
    }

    public void setDriveEncoder(double position) {
        driveEncoder.setPosition(position);
    }

    public double getDrivePostion() {
        return driveEncoder.getPosition();
    }

    public boolean getAbsoluteEncoderConnected() {
        return absoluteEnoderConnected;
    }

    public void printModuleStatus() {
        System.out.printf("%d: %f\n", index, getAbsolutePosition());
    }

    public double getAbsolutePosition() {
        return (swerveEncoderAbsolute.get() * 360);
    }

    /**
     * Returns a SwerveModuleState object from this module.

     * @return swerveModuleState - the object from this module
     */
    public SwerveModuleState getSwerveModuleState() {
        currentState = new SwerveModuleState(
            driveEncoder.getVelocity() * DriveConstants.metersPerRotation,
            Rotation2d.fromDegrees(swerveEncoder.getPosition())
        );
        return currentState;
    }

    /**
     * Returns a SwerveModulePosition object from this module.

     * @return swerveModulePosition - the object from this module
     */
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition() * DriveConstants.metersPerRotation,
            Rotation2d.fromDegrees(swerveEncoder.getPosition())
        );
    }

    /**
     * Returns the absolute angle this module needs to approach.

     * @param targetAngle - the angle the module should be at
     * @param currentAngle - the current position of the module
     * @return absoluteTarget - the absolute angle the target should approach
     */
    public SwerveAngleSpeed getAbsoluteTarget(double targetAngle, double currentAngle) {

        int multiplier = 1;

        double angleDiff = targetAngle - doubleMod(doubleMod(currentAngle, 360) + 360, 360);

        if (angleDiff > 180) {
            angleDiff -= 360;
        } else if (angleDiff < -180) {
            angleDiff += 360;
        }

        if (angleDiff < -90) {
            angleDiff += 180;
            multiplier = -1;
        } else if (angleDiff > 90) {
            angleDiff -= 180;
            multiplier = -1;
        }

        SwerveAngleSpeed absoluteTarget = new SwerveAngleSpeed();
        absoluteTarget.multiplier = multiplier;
        absoluteTarget.targetAngle = currentAngle + angleDiff;
        
        return absoluteTarget;
    }

    /**
     * Drives this module with the given module state.

     * @param moduleState - the module state for this module to use
     * @param rotate - whether or not we need to try to rotate
     * @param nos - if NOS (high drive mode) is enabled
     * @param throttle - the speed at which the modules should drive
     */
    public void driveModule(SwerveModuleState moduleState, boolean rotate, boolean nos, double throttle) {
        double currentAngle = swerveEncoder.getPosition();
        double targetAngle = moduleState.angle.getDegrees();

        SwerveAngleSpeed absoluteTarget = getAbsoluteTarget(targetAngle, currentAngle);

        if (rotate) {
            swervePID.setSetpoint(absoluteTarget.targetAngle, SparkMax.ControlType.kPosition);
        }

        setMotorSpeed(
            absoluteTarget.multiplier
            * moduleState.speedMetersPerSecond
            * DriveConstants.speedModifier
            * throttle
            * (nos ? DriveConstants.nosBooster : 1)
        );
    }
    
    
    /**
     * Sets the velocity of the drive motor using feedforward.

     * @param velocity - the target speed to set the module of
     */
    private void setMotorSpeed(double velocity) {
        double time = Timer.getFPGATimestamp();
        double acceleration = time - lastMotorSetTime > 0.1
            ? 0
            : (velocity - lastMotorSpeed) / (time - lastMotorSetTime);

        double ffv = DriveConstants.driveFeedForward[index].calculate(velocity);
        driveMotor.setVoltage(ffv);
        lastMotorSpeed = velocity;
        lastMotorSetTime = time;
    }


    private double doubleMod(double x, double y) {
        // x mod y behaving the same way as Math.floorMod but with doubles
        return (x - Math.floor(x / y) * y);
    }
}
