package frc.robot.subsystems;

import choreo.trajectory.SwerveSample;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.ControllerInput;
import frc.robot.util.ControllerInput.VisionStatus;
import frc.robot.util.SwerveModule;

/**
 * The physical subsystem that controls the drivetrain.
 */
public class Swerve extends SubsystemBase {
    private final ControllerInput controllerInput;

    public final AHRS gyroAhrs;

    private final SwerveModule[] swerveModules = new SwerveModule[4];

    private final SwerveDrivePoseEstimator poseEstimator;
    private Pose2d currentPose;
    public Field2d field;

    private final PIDController xController = new PIDController(
        DriveConstants.xyP, DriveConstants.xyI, DriveConstants.xyD);
    private final PIDController yController = new PIDController(
        DriveConstants.xyP, DriveConstants.xyI, DriveConstants.xyD);
    private final PIDController turnPID = new PIDController(
        DriveConstants.turnP, DriveConstants.turnI, DriveConstants.turnD, DriveConstants.turnR);

    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        DriveConstants.frontLeft, DriveConstants.frontRight,
        DriveConstants.backLeft, DriveConstants.backRight
    );

    private double startTime = Timer.getTimestamp();

    /**
     * Constructs a swerve subsystem with the given controller and vision systems.

     * @param controller - the controller object that will be used to control the drive system
     * @param visionSystem - the vision system that will be used to control the drivetrain
     */
    public Swerve(ControllerInput controller) {

        // assign constructor variables
        this.controllerInput = controller;

        // pose of the swerve is initialized to real values in Auto when auto routine is run
        this.currentPose = new Pose2d();
        this.field = new Field2d();

        // define the gyro
        gyroAhrs = new AHRS(NavXComType.kMXP_SPI);
        // reset the gyro
        gyroAhrs.reset();
        gyroAhrs.configureVelocity(
            false,
            false,
            false,
            true
        );

        // sets up the motors
        setupModules();
        
        // define pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            swerveDriveKinematics,
            gyroAhrs.getRotation2d(),
            getSwerveModulePositions(),
            currentPose 
        );

        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        currentPose = poseEstimator.updateWithTime(
            startTime - Timer.getTimestamp(), gyroAhrs.getRotation2d(), getSwerveModulePositions());

        field.setRobotPose(currentPose);

        if (!DriverStation.isAutonomousEnabled()) swerveDrive(getDriveSpeeds());
    }
     
    /**
     * Depending on the vision status, returns either chassis speeds based on controller inputs, or vision tag drive
     * 
     * @return ChassisSpeeds for the swerve drive to run
     */
    private ChassisSpeeds getDriveSpeeds() {
        VisionStatus status = controllerInput.visionStatus();
        ChassisSpeeds speeds;

        // if we are doing vision, then reset the gyro to prevent "whiplash"
        if (controllerInput.visionStatus() != VisionStatus.NONE) {
            controllerInput.setTurnTarget(gyroAhrs.getRotation2d().getRadians());
        }

        // you can parse through different controller inputs here and build
        // ChassisSpeeds objects to perform different actions (with vision)
        switch (status) {
            default: // if all else fails - revert to drive controls
                speeds = controllerInput.controllerChassisSpeeds(turnPID, gyroAhrs.getRotation2d());
                break;
        }

        return speeds;
    }

    /**
     * Moves the robot using given ChassisSpeeds object.

     * @param chassisSpeeds - the chassis speed that the robot should take
     */
    public void swerveDrive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleState = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        boolean rotate = chassisSpeeds.vxMetersPerSecond != 0 
                        || chassisSpeeds.vyMetersPerSecond != 0 
                        || chassisSpeeds.omegaRadiansPerSecond != 0;

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleState, DriveConstants.highDriveSpeed);

        for (int i = 0; i < 4; i++) {
            SwerveModuleState targetState = moduleState[i];
            swerveModules[i].driveModule(targetState, rotate, controllerInput.nos(), controllerInput.throttle());
        }
    }

    /**
     * Get an array of SwerveModuleState objects for each swerve module in the drive.

     * @return swerveModuleStates - the array of module states
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] swerveModuleState = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            swerveModuleState[i] = swerveModules[i].getSwerveModuleState();
        }
        return swerveModuleState;
    }

    /**
     * Get an array of SwerveModulePosition objects for each swerve module in the drive.

     * @return swerveModulePositions - the array of module postions
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            swerveModulePositions[i] = swerveModules[i].getSwerveModulePosition();
        }
        return swerveModulePositions;
    }


    private void setupModules() {
        System.out.println("Setting up swerve modules");

        // if this needs to loop more than 4 times, something is very wrong
        for (int i = 0; i < 4; i++) {
            swerveModules[i] = new SwerveModule(i);
        }
    }

    private void setSwerveEncoders(double position) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setSwerveEncoder(position);
        }
    }

    public SwerveModule[] getModules() {return swerveModules;}

    public SwerveDriveKinematics getSwerveDriveKinematics() {return swerveDriveKinematics;}

    public ChassisSpeeds getRobotState() {return swerveDriveKinematics.toChassisSpeeds(getSwerveModuleStates());}

    public Pose2d getPose() {return currentPose;} 

    public void setPose(Pose2d pose) {currentPose = pose;}

    public void resetGyro() {gyroAhrs.reset();}

    /** Prints the states of all 4 swerve modules. */
    public void printModuleStatus() {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].printModuleStatus();
        }
    }

    /**
     * Sets the robot's odometry to match that of the given pose.

     * @param pose - the pose that the robot should assume
     */
    public void resetOdometry(Pose2d pose) {

        resetGyro();
        gyroAhrs.setAngleAdjustment(pose.getRotation().getDegrees());

        currentPose = pose;
        poseEstimator.resetPose(pose);

    }

    @Override
    public void initSendable(SendableBuilder builder) {

        builder.setSmartDashboardType("SwerveDrive");     

        builder.addDoubleProperty("Front Left Angle", () -> getSwerveModuleStates()[0].angle.getDegrees(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> swerveModules[0].currentState.speedMetersPerSecond / 10, null);

        builder.addDoubleProperty("Front Right Angle", () -> swerveModules[1].currentState.angle.getDegrees(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> swerveModules[1].currentState.speedMetersPerSecond / 10, null);

        builder.addDoubleProperty("Back Left Angle", () -> swerveModules[2].currentState.angle.getDegrees(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> swerveModules[2].currentState.speedMetersPerSecond / 10, null);

        builder.addDoubleProperty("Back Right Angle", () -> swerveModules[3].currentState.angle.getDegrees(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> swerveModules[3].currentState.speedMetersPerSecond / 10, null);

        builder.addDoubleProperty("Robot Angle", () -> gyroAhrs.getRotation2d().getDegrees(), null);

        builder.addDoubleProperty("Match Time", () -> DriverStation.getMatchTime(), null);

        builder.addBooleanProperty("Module 0 Encoder", () -> 
            swerveModules[0].getAbsoluteEncoderConnected(), null);
        builder.addBooleanProperty("Module 1 Encoder", () -> 
            swerveModules[1].getAbsoluteEncoderConnected(), null);
        builder.addBooleanProperty("Module 2 Encoder", () -> 
            swerveModules[2].getAbsoluteEncoderConnected(), null);
        builder.addBooleanProperty("Module 3 Encoder", () -> 
            swerveModules[3].getAbsoluteEncoderConnected(), null);
        
    }


    // =============== AUTO STUFF ==================== //

    /**
     * Compiles and drives a ChassisSpeeds object from a given SwerveSample along the trajectory.

     * @param sample - the SwerveSample object that the robot should follow
     */
    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + turnPID.calculate(pose.getRotation().getRadians(), sample.heading),
            gyroAhrs.getRotation2d()
        );

        swerveDrive(speeds);
    }

}
