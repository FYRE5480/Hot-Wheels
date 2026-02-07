package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;

/**
 * A controller object to handle everything involving user input.
 */
public class ControllerInput extends SubsystemBase {

    /** Enumeration to represent what the robot should be doing with vision. */
    public enum VisionStatus {
        NONE,
        LOCKON,
        // add your stuff here
    }

    private double x, y, theta, slider;

    private double throttle = 0.6;

    // enables / disables "full throttle" on the drive wheels
    private boolean nos;

    private boolean fieldRelative = true;
    private boolean leftBumper;
    private boolean rightBumper;
    private boolean coral = false;

    private VisionStatus visionStatus;

    private CommandXboxController controller;
    private CommandJoystick joystick;

    // the angle the robot should try to face
    private double turnTarget = 0;
  
    public ControllerInput(CommandXboxController controller, CommandJoystick joystick) {
        this.controller = controller;
        this.joystick = joystick;
        this.visionStatus = VisionStatus.NONE;
    }

    @Override
    public void periodic() {
        // controls the X and Y directions of the robot respectively
        x = controller.getLeftX();
        y = controller.getLeftY();

        // simple deadzone, we can change this to be a circle instead of a square but it doesn't really matter
        if (Math.abs(x) < 0.05 && Math.abs(y) < 0.05) {
            x = 0;
            y = 0;
        }

        // this controls the robot spinning 
        theta = controller.getRightX();

        if (Math.abs(theta) < 0.05) {
            theta = 0;
        }

        slider = (joystick.getRawAxis(3) + 1) / 2;
        
        if (leftBumper && rightBumper) visionStatus = VisionStatus.LOCKON;
        else visionStatus = VisionStatus.NONE;

        // rightBumper && leftBumper) visionStatus = VisionStatus.STRAIGHT_POSITION;
        // else if (rightBumper) visionStatus = VisionStatus.RIGHT_POSITION;
        // else if (leftBumper) visionStatus = VisionStatus.LEFT_POSITION;
    }

    public void setTurnTarget(double target) {
        turnTarget = target;
    }

    /**
     * Uses controller input to return a ChassisSpeeds object.

     * @return chassisSpeeds - the spped of the robot calclated by the controller
     */
    public ChassisSpeeds controllerChassisSpeeds(PIDController turnPID, Rotation2d currentAngle) {
        double turnSpeed = 0;

        if (Math.abs(theta) > 0.05) {
            turnTarget = currentAngle.getRadians() - (theta * (Math.sqrt(throttle)));
        }

        turnSpeed = turnPID.calculate(currentAngle.getRadians(), turnTarget);

        ChassisSpeeds chassisSpeeds;

        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -DriveConstants.highDriveSpeed * y,
                -DriveConstants.highDriveSpeed * x,
                turnSpeed,
                currentAngle
            );
        } else {
            // If we are not in field relative mode, we are in robot relative mode
            chassisSpeeds = new ChassisSpeeds(
                -DriveConstants.highDriveSpeed * y,
                -DriveConstants.highDriveSpeed * x,
                turnSpeed
            );
        }

        return chassisSpeeds;
    }

    public Command toggleNos = Commands.runOnce(() -> {
        nos = !nos;
    });

    public Command upShift = Commands.runOnce(() -> {
        if (throttle < 1)
            throttle += 0.2;
    });

    public Command downShift = Commands.runOnce(() -> {
        if (throttle > 0)
            throttle -= 0.2;
    });

    public Command toggleFeildRelative = Commands.runOnce(() -> {
        fieldRelative = !fieldRelative;
    });

    public Command toggleRightBumper = Commands.runOnce(() -> {
        rightBumper = !rightBumper;
    });

    public Command toggleLeftBumper = Commands.runOnce(() -> {
        leftBumper = !leftBumper;
    });

    public Command toggleLockOn = Commands.runOnce(() -> {
        // lockOn = !lockOn;
    });

    public Command a = Commands.runOnce(() -> {
        coral = !coral;
    });


    public boolean nos() {return nos;}
    public double throttle() {return throttle;}
    public VisionStatus visionStatus() {return visionStatus;}

    public double slider() {return slider;}
}
