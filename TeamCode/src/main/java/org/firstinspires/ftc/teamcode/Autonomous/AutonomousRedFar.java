package org.firstinspires.ftc.teamcode.Autonomous;



import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.*;
import org.openftc.easyopencv.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PWMOutputEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous
public class AutonomousRedFar extends OpMode
{
    OpenCvWebcam cam = null;

    private IMU imu;

    double degSquare;
    double tagDistance;
    double gyroAngle;

    double targetAngle;

    double timeStart; // used to delay without using sleep
    int activeClaw = 1; // 1=ruby top  2=ruby bottom
    int clawPositionStatus = 0; // 0=grab 1=rotate 2=finish rotate 3=drive
    boolean angleCorrecting = false;
    boolean tagDetected = false;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor odometryPodLeft;
    private DcMotor odometryPodRight;
    private DcMotor odometryPodBack;

    // linear slide
    private DcMotor motorSlideLeft;
    private DcMotor motorSlideRight;

    private Servo servoClaw1;
    private Servo servoClaw2;
    private Servo servoRotate;
    private Servo servoPivot;
    private Servo servoDroneLauncher;

    private final int camWidth = 1080;
    private final int camHeight = 720;

    //servo position constants
    //bottom claw
    final double servoClaw1Open = 0.71;  // - more closed
    final double servoClaw1Closed = 0.41;

    //top claw
    final double servoClaw2Open = 0.54; // - more closed
    final double servoClaw2Closed = 0.24;

    final double servoPivotPlacePosition = 0.70;
    final double servoPivotRotatePosition = 0.65;
    final double servoPivotDrivePosition = 0.50;
    final double servoPivotGrabPosition = 0.87; // + lowers angle

    //top is the side with ruby
    double servoRotateTop = 0.155; // + rotates clockwise
    double servoRotateBottom = 0.825;

    final double servoDroneHoldPosition = 0.5;
    final double servoDroneLaunchPosition = 0.15;

    private ElapsedTime runtime = new ElapsedTime();

    public final int ticksToInch = 336;

    public String propPosition = "Unknown";

    public int colorMiddleValue = 0;
    public int colorRightValue = 0;



    public void RobotSetup()
    {
        imu = hardwareMap.get(IMU.class, "imu");

        double rotationX = -90;  // control hub X rotation angle
        double rotationY = -34;  // control hub Y rotation angle
        double rotationZ = 180;  // control hub Z rotation angle

        Orientation hubRotation = xyzOrientation(rotationX, rotationY, rotationZ);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");
        motorSlideLeft = hardwareMap.get(DcMotor.class, "SlideLeft");
        motorSlideRight = hardwareMap.get(DcMotor.class, "SlideRight");

        servoClaw1 = hardwareMap.get(Servo.class, "ClawServo1");
        servoClaw2 = hardwareMap.get(Servo.class, "ClawServo2");
        servoPivot = hardwareMap.get(Servo.class, "PivotServo");
        servoRotate = hardwareMap.get(Servo.class, "RotateServo");
        servoDroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");

        // set motor direction
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // set zero power behavior
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odometryPodLeft = hardwareMap.get(DcMotor.class, "PodLeft");
        odometryPodRight = hardwareMap.get(DcMotor.class, "FrontRight");
        odometryPodBack = hardwareMap.get(DcMotor.class, "FrontLeft");
    }
    public void DriveRobot(String driveDirection, double driveDistance, double driveSpeed, String runConcurrent)
    {
        int targetLeft;
        int targetBack;

        double driveSpeedLower = 0.2;
        int driveSpeedLowerPoint = 2;


        if(runConcurrent.equalsIgnoreCase("claw - pivot grab"))
        {
            // Moves claw to grab position
            servoPivot.setPosition(servoPivotGrabPosition);
        }

        if(driveDirection.equalsIgnoreCase("backward") || driveDirection.equalsIgnoreCase("left"))
        {
            driveDistance *= -1;
        }

        targetLeft = odometryPodLeft.getCurrentPosition() + (int)(driveDistance * ticksToInch);
        targetBack = odometryPodBack.getCurrentPosition() + (int)(driveDistance * ticksToInch);

        odometryPodLeft.setTargetPosition(targetLeft);
        odometryPodBack.setTargetPosition(targetBack);

        telemetry.addData("target", odometryPodLeft.getTargetPosition());
        telemetry.addData("current", odometryPodLeft.getCurrentPosition());

        if(driveDirection.equalsIgnoreCase("forward"))
        {


            while(odometryPodLeft.getCurrentPosition() < odometryPodLeft.getTargetPosition())
            {
                
                if(odometryPodLeft.getTargetPosition() - (driveSpeedLowerPoint * ticksToInch) <= odometryPodLeft.getCurrentPosition())
                {
                    motorFrontLeft.setPower(driveSpeedLower);
                    motorFrontRight.setPower(driveSpeedLower);
                    motorBackLeft.setPower(driveSpeedLower);
                    motorBackRight.setPower(driveSpeedLower);
                }
                
                else
                {
    
                    motorFrontLeft.setPower(driveSpeed);
                    motorFrontRight.setPower(driveSpeed);
                    motorBackLeft.setPower(driveSpeed);
                    motorBackRight.setPower(driveSpeed);
                }

            }


            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }
        else if(driveDirection.equalsIgnoreCase("backward"))
        {

            driveSpeed *= -1;
            driveSpeedLower *= -1;
            
            while(odometryPodLeft.getCurrentPosition() > odometryPodLeft.getTargetPosition())
            {
                if(odometryPodLeft.getTargetPosition() + (driveSpeedLowerPoint * ticksToInch) >= odometryPodLeft.getCurrentPosition())
                {
                    motorFrontLeft.setPower(driveSpeedLower);
                    motorFrontRight.setPower(driveSpeedLower);
                    motorBackLeft.setPower(driveSpeedLower);
                    motorBackRight.setPower(driveSpeedLower);
                }
                else
                {
                    motorFrontLeft.setPower(driveSpeed);
                    motorFrontRight.setPower(driveSpeed);
                    motorBackLeft.setPower(driveSpeed);
                    motorBackRight.setPower(driveSpeed);
                }
            }

            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }
        else if(driveDirection.equalsIgnoreCase("left"))
        {
            targetBack = odometryPodBack.getCurrentPosition() + (int)((driveDistance * -1) * ticksToInch);
            odometryPodBack.setTargetPosition(targetBack);

            while(odometryPodBack.getCurrentPosition() > odometryPodBack.getTargetPosition())
            {
                if(odometryPodBack.getTargetPosition() + (driveSpeedLowerPoint * ticksToInch) >= odometryPodBack.getCurrentPosition())
                {
                    motorFrontLeft.setPower(driveSpeedLower * -1);
                    motorFrontRight.setPower(driveSpeedLower);
                    motorBackLeft.setPower(driveSpeedLower);
                    motorBackRight.setPower(driveSpeedLower * -1);
                }

                else
                {

                    motorFrontLeft.setPower(driveSpeed * -1);
                    motorFrontRight.setPower(driveSpeed);
                    motorBackLeft.setPower(driveSpeed);
                    motorBackRight.setPower(driveSpeed * -1);
                }
                
            }
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }
        else if(driveDirection.equalsIgnoreCase("right"))
        {
            targetBack = odometryPodBack.getCurrentPosition() + (int)(driveDistance * ticksToInch);
            odometryPodBack.setTargetPosition(targetBack);

            while(odometryPodBack.getCurrentPosition() < odometryPodBack.getTargetPosition())
            {

                if(odometryPodBack.getTargetPosition() - (driveSpeedLowerPoint * ticksToInch) <= odometryPodBack.getCurrentPosition())
                {
                    motorFrontLeft.setPower(driveSpeedLower);
                    motorFrontRight.setPower(driveSpeedLower * -1);
                    motorBackLeft.setPower(driveSpeedLower * -1);
                    motorBackRight.setPower(driveSpeedLower);
                }
                else
                {

                    motorFrontLeft.setPower(driveSpeed);
                    motorFrontRight.setPower(driveSpeed * -1);
                    motorBackLeft.setPower(driveSpeed* -1);
                    motorBackRight.setPower(driveSpeed);
                }
                motorFrontLeft.setPower(driveSpeed);
                motorFrontRight.setPower(driveSpeed * -1);
                motorBackLeft.setPower(driveSpeed * -1);
                motorBackRight.setPower(driveSpeed);
            }

            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }
    }

    public void TurnRobot(String turnDirection, int turnAngle, double turnSpeed)
    {
        int targetLeft;
        int targetBack;

        int ticksPerDegree = 32;

        if(turnDirection.equalsIgnoreCase("counter")) // Counter Clockwise
        {
            targetBack = odometryPodBack.getCurrentPosition() + (int)(turnAngle * ticksPerDegree);

            motorFrontLeft.setPower(-1 * turnSpeed);
            motorFrontRight.setPower(turnSpeed);
            motorBackLeft.setPower(-1 * turnSpeed);
            motorBackRight.setPower(turnSpeed);

            while(targetBack > odometryPodBack.getCurrentPosition())
            {
                telemetry.addData("target", targetBack);
                telemetry.addData("current", odometryPodBack.getCurrentPosition());

                telemetry.update();
            }

            motorFrontLeft.setPower(0.0);
            motorFrontRight.setPower(0.0);
            motorBackLeft.setPower(0.0);
            motorBackRight.setPower(0.0);
        }
        else if(turnDirection.equalsIgnoreCase("clockwise"))
        {
            targetBack = odometryPodBack.getCurrentPosition() - (int)(turnAngle * ticksPerDegree);

            motorFrontLeft.setPower(turnSpeed);
            motorFrontRight.setPower(-1 * turnSpeed);
            motorBackLeft.setPower(turnSpeed);
            motorBackRight.setPower(-1 * turnSpeed);

            while(targetBack < odometryPodBack.getCurrentPosition())
            {
                telemetry.addData("target", targetBack);
                telemetry.addData("current", odometryPodBack.getCurrentPosition());

                telemetry.update();
            }

            motorFrontLeft.setPower(0.0);
            motorFrontRight.setPower(0.0);
            motorBackLeft.setPower(0.0);
            motorBackRight.setPower(0.0);

        }
    }

    public void MoveSlide(int slideTickPosition, String slideDirection)
    {
        boolean inPosition = false;

        motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!inPosition)
        {
            if(slideDirection.equalsIgnoreCase("up"))
            {
                if ((motorSlideLeft.getCurrentPosition()) < slideTickPosition)
                {
                    motorSlideLeft.setPower(1);
                    motorSlideRight.setPower(1);
                }
                else
                {
                    motorSlideLeft.setPower(0);
                    motorSlideRight.setPower(0);
                    inPosition = true;
                }
            }
            else if(slideDirection.equalsIgnoreCase("down"))
            {
                if ((motorSlideLeft.getCurrentPosition()) > slideTickPosition)
                {
                    motorSlideLeft.setPower(-1);
                    motorSlideRight.setPower(-1);
                }
                else
                {
                    motorSlideLeft.setPower(0);
                    motorSlideRight.setPower(0);
                    inPosition = true;
                }
            }
            telemetry.addData("target", slideTickPosition);
            telemetry.addData("current", motorSlideLeft.getCurrentPosition());
            telemetry.update();
        }
        motorSlideLeft.setPower(0);
        motorSlideRight.setPower(0);
    }


    public double[] ReadAprilTag(int tagId)
    {
        boolean inPosition = false;
        double[] tagPosition = new double[2];

        double aprilTagX = 0.0;
        double aprilTagY = 0.0;


        while(!inPosition)
        {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections)
            {
                if (detection.metadata != null)
                {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));


                    if (detection.id == tagId)
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
                            else
                            {
                                //inPosition = true;
                            }
                        }
                    }

                }
                else
                {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));

                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                }

                aprilTagX = detection.ftcPose.x;
                aprilTagY = detection.ftcPose.y;

            }
            if (angleCorrecting)
            {
                if (gyroAngle < targetAngle - 8)
                {
                    motorFrontLeft.setPower(-0.3);
                    motorFrontRight.setPower(0.3);
                    motorBackLeft.setPower(-0.3);
                    motorBackRight.setPower(0.3);
                }
                else if (gyroAngle < targetAngle - 1)
                {
                    motorFrontLeft.setPower(-0.1);
                    motorFrontRight.setPower(0.1);
                    motorBackLeft.setPower(-0.1);
                    motorBackRight.setPower(0.1);
                }
                else if (gyroAngle > targetAngle + 1)
                {
                    motorFrontLeft.setPower(0.1);
                    motorFrontRight.setPower(-0.1);
                    motorBackLeft.setPower(0.1);
                    motorBackRight.setPower(-0.1);
                }
                else if (gyroAngle > targetAngle + 8)
                {
                    motorFrontLeft.setPower(0.8);
                    motorFrontRight.setPower(-0.8);
                    motorBackLeft.setPower(0.8);
                    motorBackRight.setPower(-0.8);
                }
                else
                {
                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    angleCorrecting = false;
                }
            }
            else
            {
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }

            if (currentDetections == null)
            {
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }


            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("target", targetAngle);
            telemetry.update();


        }

        tagPosition[0] = aprilTagX;
        tagPosition[1] = aprilTagY;

        return tagPosition;
    }

    public void FlipClaw()
    {
        boolean isFlipped = false;

        while (!isFlipped)
        {
            if(clawPositionStatus == 0)
            {
                servoPivot.setPosition(servoPivotRotatePosition);
                clawPositionStatus = 1;
                timeStart = System.currentTimeMillis();
            }
            else if(clawPositionStatus == 1 & System.currentTimeMillis() >= (timeStart + 100))  // add delay without sleep
            {
                if(activeClaw == 1)
                {
                    servoRotate.setPosition(servoRotateBottom);
                }
                else
                {
                    servoRotate.setPosition(servoRotateTop);
                }

                clawPositionStatus = 2;
                timeStart = System.currentTimeMillis();
            }
            else if(clawPositionStatus == 2 & System.currentTimeMillis() >= (timeStart + 200))
            {
                servoPivot.setPosition(servoPivotGrabPosition);
                clawPositionStatus = 0;

                if(activeClaw == 1)
                {
                    activeClaw = 2;
                }
                else
                {
                    activeClaw = 1;
                }

                isFlipped = true;
            }

        }
    }
    public void sleep(int millis)
    {
        runtime.reset();
        while(runtime.milliseconds() < millis)
        {
            // lole ! ! ! ! !
        }
    }

    public void AutonomousLeft()
    {
        DriveRobot("forward", 12, 0.5, "none");
        TurnRobot("counter", 45, 0.2);
        DriveRobot("forward", 4, 0.5, "none");
        servoPivot.setPosition(servoPivotGrabPosition);
        sleep(500);
        servoClaw1.setPosition(servoClaw1Open);
        sleep(1000);

        DriveRobot("backward", 4, 0.5, "none");
        TurnRobot("clockwise", 45, 0.2);
        //DriveRobot("right", 34, 0.5);

    }

    public void AutonomousMid()
    {
        DriveRobot("forward", 5, 0.5, "none");
        DriveRobot("right", 2, 0.5, "none");
        DriveRobot("forward", 12, 0.5, "claw - pivot grab");

        servoClaw1.setPosition(servoClaw1Open);// drop purple pixel
        sleep(200);

        DriveRobot("backward", 2, 0.5, "none"); // drive robot backwards

        FlipClaw(); // ruby down

        DriveRobot("right", 25, 0.5, "none");
        DriveRobot("forward", 2, 0.5, "none");
        TurnRobot("clockwise", 85, 0.5);

        servoPivot.setPosition(servoPivotPlacePosition);// tilt claw
        MoveSlide(650, "up"); // raise arm

        double[] tagPosition = ReadAprilTag(6);
        DriveRobot("forward", (tagPosition[1] - 7.5), 0.5, "none");

        if(tagPosition[0] - 0.0 > 0)
        {
            DriveRobot("right", (tagPosition[0] - 0.0), 0.5, "none");
        }
        else if(tagPosition[0] - 0.0 < 0)
        {
            DriveRobot("left", (tagPosition[0] - 0.0), 0.5, "none");
        }

        servoClaw2.setPosition(servoClaw2Open);// release yellow pixel
        sleep(100);

        DriveRobot("backward", 4, 0.5, "none");
        MoveSlide(0, "down");
        FlipClaw();



        telemetry.addData("x", tagPosition[0]);
        telemetry.addData("y", tagPosition[1]);
        telemetry.update();

        //DriveRobot("forward", 5, 0.5, "none");



    }

    public void AutonomousRight()
    {
        DriveRobot("forward", 12, 0.5, "none");
        DriveRobot("right", 8, 0.5, "none");
        servoPivot.setPosition(servoPivotGrabPosition);
        sleep(500);
        servoClaw1.setPosition(servoClaw1Open);

        sleep(1000);
        DriveRobot("right", 24, 0.5, "none");
        servoClaw2.setPosition(servoClaw2Open);
    }

    public void initAprilTag()
    {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM)
        {
            visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        }
        else
        {
            visionPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, aprilTag);
        }
    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag()
    {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections)
        {
            if (detection.metadata != null)
            {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));



                if (detection.id == 6)
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
                }


            }
            else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));

                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }

        }
        if (angleCorrecting)
        {
            if (gyroAngle < targetAngle - 8)
            {
                motorFrontLeft.setPower(-0.3);
                motorFrontRight.setPower(0.3);
                motorBackLeft.setPower(-0.3);
                motorBackRight.setPower(0.3);
            }
            else if (gyroAngle < targetAngle - 1)
            {
                motorFrontLeft.setPower(-0.1);
                motorFrontRight.setPower(0.1);
                motorBackLeft.setPower(-0.1);
                motorBackRight.setPower(0.1);
            }
            else if (gyroAngle > targetAngle + 1)
            {
                motorFrontLeft.setPower(0.1);
                motorFrontRight.setPower(-0.1);
                motorBackLeft.setPower(0.1);
                motorBackRight.setPower(-0.1);
            }
            else if (gyroAngle > targetAngle + 8)
            {
                motorFrontLeft.setPower(0.8);
                motorFrontRight.setPower(-0.8);
                motorBackLeft.setPower(0.8);
                motorBackRight.setPower(-0.8);
            }
            else
            {
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                angleCorrecting = false;
            }
        }
        else
        {
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }

        if (currentDetections == null)
        {
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }


        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("target", targetAngle);
        telemetry.update();
    }

    public void init()
    {
        RobotSetup();

        servoClaw1.setPosition(servoClaw1Closed);
        servoClaw2.setPosition(servoClaw2Closed);

        servoDroneLauncher.setPosition(servoDroneHoldPosition);

        runtime.reset();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        cam.setPipeline(new PropRecognition());

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
                                  {
                                      @Override
                                      public void onOpened()
                                      {
                                          cam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
                                      }

                                      @Override
                                      public void onError(int errorCode)
                                      {

                                      }
                                  }
        );

    }


    public void start()
    {

        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoDroneLauncher.setPosition(servoDroneHoldPosition);
        cam.closeCameraDevice();
        telemetry.addData(("propPosition"), propPosition);
        telemetry.update();
        initAprilTag();
        //sleep(500);
        //telemetryAprilTag();

        //visionPortal.close();

        propPosition = "test";

        telemetry.addData("position", propPosition);
        if(propPosition.equalsIgnoreCase("left"))
        {
            AutonomousLeft();
        }
        else if(propPosition.equalsIgnoreCase("middle"))
        {
            AutonomousMid();
        }
        else if(propPosition.equalsIgnoreCase("right"))
        {
            AutonomousRight();
        }
        else if(propPosition.equalsIgnoreCase("test"))
        {
            double[] tagPosition = ReadAprilTag(3);

            telemetry.addData("x", tagPosition[0]);
            telemetry.addData("y", tagPosition[1]);
            telemetry.update();
        }

    }

    public void loop()
    {
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        //gyroAngle = orientation.getYaw(AngleUnit.DEGREES);
        //telemetry.addData("gyroangle", gyroAngle);
        //telemetry.update();
    }


    class PropRecognition extends OpenCvPipeline
    {
        Mat YCbCr = new Mat();
        Mat outPut = new Mat();

        Scalar color = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input)
        {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2HSV);

            Rect rectLeft = new Rect(260, 250, 300, 300);
            Rect rectRight = new Rect(800, 280, 300, 300);

            input.copyTo(outPut);

            Imgproc.rectangle(outPut, rectLeft, color, 2);
            Imgproc.rectangle(outPut, rectRight, color, 2);

            Mat matLeftCrop = YCbCr.submat(rectLeft);
            Mat matRightCrop = YCbCr.submat(rectRight);

            Core.extractChannel(matLeftCrop, matLeftCrop, 2);
            Core.extractChannel(matRightCrop, matRightCrop, 2);

            Scalar colorLeftAvg = Core.mean(matLeftCrop);
            Scalar colorRightAvg = Core.mean(matRightCrop);

            telemetry.addData("middleVal", (int)colorLeftAvg.val[0]);
            telemetry.addData("rightVal", (int)colorRightAvg.val[0]);

            if (colorMiddleValue == 0 && runtime.milliseconds() > 8000)
            {
                colorMiddleValue = (int)colorLeftAvg.val[0];
                colorRightValue = (int)colorRightAvg.val[0];
            }
            else if ((colorMiddleValue < (int) colorLeftAvg.val[0] + 10) && (colorMiddleValue > (int) colorLeftAvg.val[0] - 10))
            {
                propPosition = "middle";
            }
            else if ((colorRightValue < (int) colorRightAvg.val[0] - 10) || (colorRightValue > (int) colorRightAvg.val[0] + 10))
            {
                propPosition = "right";
            }
            else
            {
                propPosition = "left";
            }
            telemetry.addData("middlestart", colorMiddleValue);
            telemetry.addData("rightstart", colorRightValue);
            telemetry.addData(("propPosition"), propPosition);

            return(outPut);
        }
    }

}
