package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.CameraWebsocketClient;
import frc.robot.util.CameraWebsocketClient.Apriltag;
import frc.robot.util.CameraWebsocketClient.Info;

import java.lang.ref.Cleaner;

public class Vision {
    public static enum Side {FRONT, LEFT, BACK, RIGHT}

    private String ip;
    private ArrayList<CameraWebsocketClient> camClientList = new ArrayList<CameraWebsocketClient>();
    private HashMap<String, Integer[]> apriltagPoses; // Hashmap of [angle, x, y] where angle is the angle of the tag in rads, where 
    private Timer timer = new Timer();

    private PIDController turnPID = new PIDController(Constants.VisionConstants.turnP, Constants.VisionConstants.turnI, Constants.VisionConstants.turnD);
    private PIDController movePID = new PIDController(Constants.VisionConstants.moveP, Constants.VisionConstants.moveI, Constants.VisionConstants.moveD);
    private ChassisSpeeds prevChassisSpeeds;
    private double prevTime;

    private final Cleaner cleaner = Cleaner.create();
    private final Cleaner.Cleanable cleanable;

    public static class CameraPair{
        public int cam1;
        public int cam2;

        public double cam1Angle;
        public double cam2Angle;

        public double cam1XOffset;
        public double cam1YOffset;

        public double cam2XOffset;
        public double cam2YOffset;
        public CameraPair(int cam1, double cam1Angle, double cam1XOffset, double cam1YOffset, int cam2, double cam2Angle, double cam2XOffset, double cam2YOffset) {
            this.cam1 = cam1;
            this.cam2 = cam2;
            this.cam1Angle = cam1Angle;
            this.cam2Angle = cam2Angle;
            this.cam1XOffset = cam1XOffset;
            this.cam1YOffset = cam1YOffset;
            this.cam2XOffset = cam2XOffset;
            this.cam2YOffset = cam2YOffset;
        }
    }
    public static class RobotPositionOffset{
        public double xOffset;
        public double yOffset;
        public double angleOffset;

        public RobotPositionOffset(double xOffset, double yOffset, double angleOffset) {
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.angleOffset = angleOffset;
        }
    }
    
    public Vision(String ipAddress, int[] cameraRotation, HashMap<String, Integer[]> apriltagPoses) {
        // This constructor is not ideal but it works for the example. IRL you would want to use the other constructor so you can still have a list of cameras outside of the Interface.
        // Maybe I will make this the only class that you need to use with the cameras then it will be fine.
        // Camera Rotation is the rotation of each camera in degrees. 0 is the default rotation.
        // Apriltag Angles is a hashmap of the apriltag id to the angle of the tag in degrees. 0 is facing the camera.

        this.ip = ipAddress;
        this.apriltagPoses = apriltagPoses;
        boolean failed = false;
        int i = 0;
        while(!failed) {
            System.out.println("Trying to commect to: " + ip + ":" + (i + 50000));
            CameraWebsocketClient newCam = new CameraWebsocketClient(ip + ":" + (i + 50000), 1000);
            newCam.setupConnection();

            if(newCam.isConnected()) {
                newCam.setRotation(cameraRotation[i]);
                camClientList.add(newCam);
                i++;
            } else {
                failed = true;
            }
        }

        this.cleanable = cleaner.register(this, cleanCams(camClientList));

        turnPID.enableContinuousInput(-180, 180);
        turnPID.setSetpoint(0);
        movePID.enableContinuousInput(-180, 180);
        movePID.setSetpoint(0);
    }

    public Vision(CameraWebsocketClient[] camList, HashMap<String, Integer[]> apriltagPoses) {
        this.apriltagPoses = apriltagPoses;
        for(CameraWebsocketClient newCam : camList) {
            if(newCam.isConnected()) {
                camClientList.add(newCam);
            }
        }

        this.cleanable = cleaner.register(this, cleanCams(camClientList));

        // Erm, what the sigma? IS this corect? I think it is but I am not sure
        turnPID.enableContinuousInput(-180, 180);
        turnPID.setSetpoint(0);
        movePID.enableContinuousInput(-180, 180);
        movePID.setSetpoint(0);
    }

    public void clear(){
        for(CameraWebsocketClient cam : camClientList) {
            cam.clear();
        }
    }

    public ChassisSpeeds frontToSide(ChassisSpeeds inSpeeds, Side side){
        switch(side) {
            case FRONT:
                return inSpeeds;
            case LEFT:
                return new ChassisSpeeds(inSpeeds.vyMetersPerSecond, -inSpeeds.vxMetersPerSecond, inSpeeds.omegaRadiansPerSecond);
            case RIGHT:
                return new ChassisSpeeds(-inSpeeds.vyMetersPerSecond, inSpeeds.vxMetersPerSecond, inSpeeds.omegaRadiansPerSecond);
            case BACK:
                return new ChassisSpeeds(-inSpeeds.vxMetersPerSecond, -inSpeeds.vyMetersPerSecond, inSpeeds.omegaRadiansPerSecond);
            default:
                return inSpeeds;
          }
    }

    public double getZAngle(int maxTags) {
        // This function returns the average calculated angle of the robot in degrees on the z axis, aka the only one the robot turns on. Limit the number of tags to use with maxTags if you want.

        double ZAngle = 0;
        double numTags = 0;
        for(CameraWebsocketClient cam : camClientList) {
            List<CameraWebsocketClient.Apriltag> tags = cam.getApriltags();

            System.out.println(tags.size());

            for (CameraWebsocketClient.Apriltag tag : tags) {
                Integer[] tagAngleArray = apriltagPoses.getOrDefault(tag.tagId, new Integer[]{0, 0, 0});
                int tagAngle = tagAngleArray[0];
                ZAngle += tagAngle * (180/Math.PI) + cam.getRotation() + tag.orientation[1];
                numTags++;
                if(numTags == maxTags) {
                    break;
                }
            }
            if(numTags == maxTags) {
                break;
            }
        }
        if(numTags == 0) {
            return 69420.0; // nice
        }
        return ZAngle/numTags % 360;
    }

    public double getZAngle() {
        // W overload
        return getZAngle(4);
    }

    /**
     * Returns an INCOMPLETE ChassisSpeeds object - only the rotation unit - for locking on to a tag 

     * @param camIndex - the index of the desired camera to use
     * @param tagId - the apriltag ID to search for, null if no preference
     * @return speeds - the ChassisSpeeds object for the rotation to take
     */
    public ChassisSpeeds lockonTagSpeeds(int camIndex, String tagId) {
        Apriltag tag;
        if (tagId != null) tag = decideTag(camIndex, tagId);
        else tag = decideTag(camIndex);
        if (tag == null) return null;

        return new ChassisSpeeds(
            0,
            0,
            turnPID.calculate(tag.horizontalAngle)
        );
    }

    /**
     * This function returns the relative position of the tag to the camera in the camera's frame of reference.

     * @param camIndex - the index of the desired camera to use
     * @param tagId - the apriltag ID to search for, null if no preference
     * @return position - the relative position of the tag to the robot
     */

    public Transform2d getTagRelativePosition(int camIndex, String[] tagIds, Side side, double cameraHorizontalAngle, double xOffset, double yOffset) {
        // The position is returned as a 3 element array of doubles in the form [x, y, z]
        // The position is in meters.
        Apriltag tag;
        if (tagIds != null) tag = decideTag(camIndex, tagIds);
        else tag = decideTag(camIndex);    

        if (tag == null) {
            System.out.println("No tags found for camera " + camIndex);
            return null;
        }

        double horizontalAngle = -tag.horizontalAngle + cameraHorizontalAngle/2;
        double tagAngle = -tag.orientation[1] + cameraHorizontalAngle/2;

        double e; 
        if (yOffset == 0) e = -Math.PI/2;
        else e = Math.atan(-xOffset/yOffset) + (Math.PI * (Math.signum(yOffset)-1)/2);
        double f = Math.sqrt(xOffset * xOffset + yOffset * yOffset);

        double xDist = tag.distance * Math.sin(horizontalAngle) + Math.cos(tagAngle + e)*f;
        double yDist = tag.distance * Math.cos(horizontalAngle) + Math.sin(tagAngle + e)*f;

        System.out.println("camIndex: " + camIndex);
        System.out.println("X Distance: " + xDist);
        System.out.println("Y Distance: " + yDist);
        System.out.println("Tag Angle: " + tagAngle);

        Transform2d position = new Transform2d(
            xDist,
            yDist,
            new Rotation2d(tagAngle)
        );

        return position;
    }

    public ChassisSpeeds getTagDrive(CameraPair cams, String[] tagIds, Side side, RobotPositionOffset offsets) {
        double angleOffset = offsets.angleOffset;
        double xOffset = offsets.xOffset;
        double yOffset = offsets.yOffset;

        Transform2d cam1Position = getTagRelativePosition(cams.cam1, tagIds, side, cams.cam1Angle + angleOffset, cams.cam1XOffset + xOffset, cams.cam1YOffset + yOffset);
        Transform2d cam2Position = getTagRelativePosition(cams.cam2, tagIds, side, cams.cam2Angle + angleOffset, cams.cam2XOffset + xOffset, cams.cam2YOffset + yOffset);

        Transform2d averagedPosition = new Transform2d();

        if (cam1Position != null){
            averagedPosition.plus(cam1Position);
        }
        if (cam2Position != null){
            averagedPosition.plus(cam2Position);
        }

        // average out averagedPosition if both positions are not null
        if (cam1Position != null && cam2Position != null) {
            averagedPosition.div(2);
        }

        double xDist = averagedPosition.getX();
        double yDist = averagedPosition.getY();
        double totDist = Math.sqrt(xDist * xDist + yDist * yDist);

        double turnSpeed = turnPID.calculate(averagedPosition.getRotation().getRadians());   
        double moveSpeed = movePID.calculate(totDist);

        double xMove = (xDist / totDist) * moveSpeed;
        double yMove = (-yDist / totDist) * moveSpeed;

        System.out.println("Averaged X Distance: " + xDist);
        System.out.println("Averaged Y Distance: " + yDist);
        System.out.println("Averaged Tag Angle: " + averagedPosition.getRotation().getRadians());

        ChassisSpeeds speeds = new ChassisSpeeds(
            DriveConstants.highDriveSpeed * xMove,
            DriveConstants.highDriveSpeed * yMove,
            -turnSpeed
        );

        return speeds;
    }

    public ChassisSpeeds getPieceDrive(int camIndex, double cameraOffsetAngle, double xOffset, double yOffset) {
        CameraWebsocketClient cam = camClientList.get(camIndex);
        CameraWebsocketClient.Piece piece = cam.getPiece();
        
        if(piece == null) {
            return null;
        }
        
        double driveAngleModifier;
        if (piece.angle > VisionConstants.maxIntakeAngle) {
            driveAngleModifier = VisionConstants.misallignedPieceOffset / piece.distance; 
            // This math works in my head. Make the angle larger if the piece is closer, and less when it is farther. 
            //Asymptotic to 0, so it will always theoretically adjust.
        } else if (piece.angle < -VisionConstants.maxIntakeAngle) {
            driveAngleModifier = -VisionConstants.misallignedPieceOffset / piece.distance;
        } else {
            driveAngleModifier = 0;
        }

        double x = Math.cos(piece.angle + cam.getRotation() + driveAngleModifier) * piece.distance - xOffset;
        double y = Math.sin(piece.angle + cam.getRotation() + driveAngleModifier) * piece.distance - yOffset;
        

        double turnSpeed = turnPID.calculate(piece.angle-cameraOffsetAngle);
        double moveSpeed = movePID.calculate(piece.distance);

        double xMove = (x / Math.sqrt(x*x + y*y)) * moveSpeed;
        double yMove = (y / Math.sqrt(x*x + y*y)) * moveSpeed;

        return new ChassisSpeeds(
            DriveConstants.highDriveSpeed * xMove,
            DriveConstants.highDriveSpeed * yMove,
            turnSpeed
        );
    }

    public ChassisSpeeds getPieceDrive() {
        return getPieceDrive(VisionConstants.pieceDetectionCamIndex, 0, 0, 0);
    }

    private Apriltag decideTag(int camIndex) {
        CameraWebsocketClient cam = camClientList.get(camIndex);
        List<CameraWebsocketClient.Apriltag> tags = cam.getApriltags();
        Apriltag tag = null;

        if (tags.size() > 0) {
            Apriltag bestTag = tags.get(0);
            for (Apriltag t : tags) { // This is a weird way to do this but it works - I need to make this more efficient
                if(t.distance < bestTag.distance) {
                    tag = t;
                    break;
                }
            }
        }

        return tag;
    }

    private Apriltag decideTag(int camIndex, String tagId) {
        CameraWebsocketClient cam = camClientList.get(camIndex);
        List<CameraWebsocketClient.Apriltag> tags = cam.getApriltags();
        Apriltag tag = null;
        for (Apriltag t : tags) { // This is a weird way to do this but it works - I need to make this more efficient
            if(t.tagId.equals(tagId)) {
                tag = t;
                break;
            }
        }

        return tag;
    }

    private Apriltag decideTag(int camIndex, String tagIds[]) {
        CameraWebsocketClient cam = camClientList.get(camIndex);
        List<CameraWebsocketClient.Apriltag> tags = cam.getApriltags();
        List<String> tagIdList = Arrays.asList(tagIds);
        if (tags.size() == 0) {
            return null;
        }
        Apriltag bestTag = tags.get(0);
        for (Apriltag t : tags) { // This is a weird way to do this but it works - I need to make this more efficient
            if(tagIdList.contains(t.tagId) && t.distance <= bestTag.distance) {
                bestTag = t;
            }
        }

        return bestTag;
    }

    public Info getInfo() {
        return camClientList.get(0).getInfo();
    }
    
    private static Runnable cleanCams(final ArrayList<CameraWebsocketClient> cams) {
    return () -> {
        for (CameraWebsocketClient cam : cams) {
            if (cam.isConnected()) {
                cam.disconnect();
            }
        }
    };
  }
}
