
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;



import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * the easy way.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: AprilTag Easy", group = "Concept")


public class AprilTag extends LinearOpMode {
    
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor FrontRight;
    private IMU imu;
    
    double degSquare;
    double tagDistance;
    double gyroAngle;

    
    double targetAngle;
    
    boolean angleCorrecting = false; 
    boolean tagDetected = false; 

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() 
    {
        
        
        
        imu = hardwareMap.get(IMU.class, "imu");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        double xRotation = -90;  // enter the desired X rotation angle here.
        double yRotation = -34;  // enter the desired Y rotation angle here.
        double zRotation = 180;  // enter the desired Z rotation angle here.

    Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

    // Now initialize the IMU with this mounting orientation
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
    imu.initialize(new IMU.Parameters(orientationOnRobot));

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) 
        {
            while (opModeIsActive()) 
            {

                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down)
                {
                    visionPortal.stopStreaming();
                } 
                else if (gamepad1.dpad_up)
                {
                    visionPortal.resumeStreaming();
                }
                
                
                    
                    
                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        }
        else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK, aprilTag);
        }
    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        
        tagDetected = false;
        


        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections)
        {
            if (detection.metadata != null) 
            {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                
                if (detection.id == 9) 
                {
                    
                    YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                    
                    tagDetected = true;
                    degSquare = detection.ftcPose.yaw;
                    tagDistance = detection.ftcPose.y;
                    gyroAngle = orientation.getYaw(AngleUnit.DEGREES);
                    if (angleCorrecting == false)
                    {
                        targetAngle = gyroAngle - degSquare;
                        if (gyroAngle < targetAngle - 1 || gyroAngle > targetAngle + 1)
                        {
                            angleCorrecting = true;
                        }
                    }
                    
                    /*
                    if (tagDistance > 15)
                    {
                        
                        FrontLeft.setPower(0.2);
                        FrontRight.setPower(0.2);
                        BackLeft.setPower(0.2);
                        BackRight.setPower(0.2);
                        
                    }
                    else
                    {
                        FrontLeft.setPower(0);
                        FrontRight.setPower(0);
                        BackLeft.setPower(0);
                        BackRight.setPower(0);
                    }
                    */
                }
            } 
            else
            {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
            }
            
        } 
        if (angleCorrecting)
        {
            if (gyroAngle < targetAngle - 8)
            {
                FrontLeft.setPower(-0.3);
                FrontRight.setPower(0.3);
                BackLeft.setPower(-0.3);
                BackRight.setPower(0.3);
            }
            else if (gyroAngle < targetAngle - 1)
            {
                FrontLeft.setPower(-0.1);
                FrontRight.setPower(0.1);
                BackLeft.setPower(-0.1);
                BackRight.setPower(0.1);
            }
            else if (gyroAngle > targetAngle + 1)
            {
                FrontLeft.setPower(0.1);
                FrontRight.setPower(-0.1);
                BackLeft.setPower(0.1);
                BackRight.setPower(-0.1);
            }
            else if (gyroAngle > targetAngle + 8)
            {
                FrontLeft.setPower(0.8);
                FrontRight.setPower(-0.8);
                BackLeft.setPower(0.8);
                BackRight.setPower(-0.8);
            }
            else
            {
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
                angleCorrecting = false;
            }
        }
        else
        {
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }
        
        // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("target", targetAngle);
        //telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        //telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
    

    }   // end method telemetryAprilTag()

}   // end class
