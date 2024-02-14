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

import com.acmerobotics.roadrunner.ftc.DriveView;
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
public class AutonomousBackdrop extends OpMode
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
    final double servoPivotGrabPosition = 0.88; // + lowers angle

    //top is the side with ruby
    double servoRotateTop = 0.155; // + rotates clockwise
    double servoRotateBottom = 0.825;

    final double servoDroneHoldPosition = 0.5;
    final double servoDroneLaunchPosition = 0.15;

    private ElapsedTime runtime = new ElapsedTime();

    public final int ticksToInch = 336;

    public String propPosition = "Unknown";

    public int countMiddleValue = 0;
    public int countRightValue = 0;

    double cameraOffsetX = 4; // offset from camera
    double cameraOffsetY = 6.0;

    double orientationStart;


    Mat matLeftCropBlue1 = new Mat();
    Mat matRightCropBlue1 = new Mat();
    Mat matLeftCropRed1 = new Mat();
    Mat matRightCropRed1 = new Mat();
    Mat matLeftCropRed2 = new Mat();
    Mat matRightCropRed2 = new Mat();
    Mat matLeftCrop = new Mat();
    Mat matRightCrop = new Mat();

    String propColor = "unknown";

    public void RobotSetup()
    {
        imu = hardwareMap.get(IMU.class, "imu");

        double rotationX = -90;  // control hub X rotation angle
        double rotationY = -33.8;  // control hub Y rotation angle
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

        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void DriveRobot(String driveDirection, double driveDistance, double driveSpeed, String runConcurrent)
    {
        int targetLeft;
        int targetBack;

        double driveSpeedOriginal = driveSpeed;
        double driveSpeedLow = 0.3;
        double driveSpeedMiddle = 0.45;
        double driveSpeedFast = 0.6;
        double driveSpeedDeadZoneLow = 4;
        double driveSpeedDeadZoneMiddle = 8;
        double driveSpeedDeadZoneFast = 12;

        int driveTimeout = 100000;


        if(runConcurrent.equalsIgnoreCase("claw - pivot grab"))
        {
            // Moves claw to grab position
            servoPivot.setPosition(servoPivotGrabPosition);
        }
        else if(runConcurrent.equalsIgnoreCase("claw - pivot drive"))
        {
            // Moves claw to drive position
            servoPivot.setPosition(servoPivotDrivePosition);
        }
        else if(runConcurrent.equalsIgnoreCase("timeout - 2 seconds"))
        {
            // Stop moving after 2 seconds or drive distance reached
            driveTimeout = 2000;
        }

        runtime.reset();

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

        runtime.reset();
        if(driveDirection.equalsIgnoreCase("forward"))
        {
            while(odometryPodLeft.getCurrentPosition() < odometryPodLeft.getTargetPosition() && runtime.milliseconds() < driveTimeout)
            {
                if(runtime.milliseconds() < 300)
                {
                    // Ease into start
                    driveSpeed = 0.3;
                }
                else
                {
                    driveSpeed = driveSpeedOriginal;
                }
                //check for dead zone 4 inches from target position.
                if(odometryPodLeft.getTargetPosition() - odometryPodLeft.getCurrentPosition() < driveSpeedDeadZoneLow * ticksToInch)
                {
                    if(driveSpeed > driveSpeedLow)
                    {
                        driveSpeed = driveSpeedLow;
                    }
                }
                else if(odometryPodLeft.getTargetPosition() - odometryPodLeft.getCurrentPosition() < driveSpeedDeadZoneMiddle * ticksToInch)
                {
                    if(driveSpeed > driveSpeedMiddle)
                    {
                        driveSpeed = driveSpeedMiddle;
                    }
                }
                else if(odometryPodLeft.getTargetPosition() - odometryPodLeft.getCurrentPosition() < driveSpeedDeadZoneFast * ticksToInch)
                {
                    if(driveSpeed > driveSpeedFast)
                    {
                        driveSpeed = driveSpeedFast;
                    }
                }

                motorFrontLeft.setPower(driveSpeed);
                motorFrontRight.setPower(driveSpeed);
                motorBackLeft.setPower(driveSpeed);
                motorBackRight.setPower(driveSpeed);
            }

            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }
        else if(driveDirection.equalsIgnoreCase("backward"))
        {
            driveSpeed *= -1;
            driveSpeedOriginal *= -1;
            driveSpeedLow *= -1;
            driveSpeedMiddle *= -1;
            driveSpeedFast *= -1;

            while(odometryPodLeft.getCurrentPosition() > odometryPodLeft.getTargetPosition() && runtime.milliseconds() < driveTimeout)
            {
                if(runtime.milliseconds() < 300)
                {
                    // Ease into start
                    driveSpeed = -0.3;
                }
                else
                {
                    driveSpeed = driveSpeedOriginal;
                }

                if(odometryPodLeft.getCurrentPosition() - odometryPodLeft.getTargetPosition() < driveSpeedDeadZoneLow * ticksToInch)
                {
                    if(driveSpeed > driveSpeedLow)
                    {
                        driveSpeed = driveSpeedLow;
                    }
                }
                else if(odometryPodLeft.getCurrentPosition() - odometryPodLeft.getTargetPosition() < driveSpeedDeadZoneMiddle * ticksToInch)
                {
                    if(driveSpeed > driveSpeedMiddle)
                    {
                        driveSpeed = driveSpeedMiddle;
                    }
                }
                else if(odometryPodLeft.getCurrentPosition() - odometryPodLeft.getTargetPosition() < driveSpeedDeadZoneFast * ticksToInch)
                {
                    if(driveSpeed > driveSpeedFast)
                    {
                        driveSpeed = driveSpeedFast;
                    }
                }

                motorFrontLeft.setPower(driveSpeed);
                motorFrontRight.setPower(driveSpeed);
                motorBackLeft.setPower(driveSpeed);
                motorBackRight.setPower(driveSpeed);
            }

            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }
        else if(driveDirection.equalsIgnoreCase("left"))
        {

            while(odometryPodBack.getCurrentPosition() > odometryPodBack.getTargetPosition() && runtime.milliseconds() < driveTimeout)
            {
                if(runtime.milliseconds() < 300)
                {
                    // Ease into start
                    driveSpeed = 0.3;
                }
                else
                {
                    driveSpeed = driveSpeedOriginal;
                }

                //check for dead zone very close target position.
                if(odometryPodBack.getCurrentPosition() - odometryPodBack.getTargetPosition() < driveSpeedDeadZoneLow * ticksToInch)
                {
                    if(driveSpeed > driveSpeedLow)
                    {
                        driveSpeed = driveSpeedLow;
                    }
                    driveSpeed = driveSpeedLow;
                }
                //check for dead zone close to target position.
                else if(odometryPodBack.getCurrentPosition() - odometryPodBack.getTargetPosition() < driveSpeedDeadZoneMiddle * ticksToInch)
                {
                    if(driveSpeed > driveSpeedMiddle)
                    {
                        driveSpeed = driveSpeedMiddle;
                    }
                }
                else if(odometryPodBack.getCurrentPosition() - odometryPodBack.getTargetPosition() < driveSpeedDeadZoneFast * ticksToInch)
                {
                    if(driveSpeed > driveSpeedFast)
                    {
                        driveSpeed = driveSpeedFast;
                    }
                }

                motorFrontLeft.setPower(driveSpeed * -1);
                motorFrontRight.setPower(driveSpeed);
                motorBackLeft.setPower(driveSpeed);
                motorBackRight.setPower(driveSpeed * -1);

            }

            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

        }
        else if(driveDirection.equalsIgnoreCase("right"))
        {
            while(odometryPodBack.getCurrentPosition() < odometryPodBack.getTargetPosition() && runtime.milliseconds() < driveTimeout)
            {
                if(runtime.milliseconds() < 300)
                {
                    // Ease into start
                    driveSpeed = 0.3;
                }
                else
                {
                    driveSpeed = driveSpeedOriginal;
                }
                //check for dead zone very close to target position.
                if(odometryPodBack.getTargetPosition() - odometryPodBack.getCurrentPosition() < driveSpeedDeadZoneLow * ticksToInch)
                {
                    if(driveSpeed > driveSpeedLow)
                    {
                        driveSpeed = driveSpeedLow;
                    }
                }
                //check for dead zone close to target position.
                else if(odometryPodBack.getTargetPosition() - odometryPodBack.getCurrentPosition() < driveSpeedDeadZoneMiddle * ticksToInch)
                {
                    if(driveSpeed > driveSpeedMiddle)
                    {
                        driveSpeed = driveSpeedMiddle;
                    }
                }
                else if(odometryPodBack.getTargetPosition() - odometryPodBack.getCurrentPosition() < driveSpeedDeadZoneFast * ticksToInch)
                {
                    if(driveSpeed > driveSpeedFast)
                    {
                        driveSpeed = driveSpeedFast;
                    }
                }

                telemetry.update();

                motorFrontLeft.setPower(driveSpeed);
                motorFrontRight.setPower(driveSpeed * -1);
                motorBackLeft.setPower(driveSpeed* -1);
                motorBackRight.setPower(driveSpeed);
            }

            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }
    }


    public void RotateToOrientation (double orientationTarget)
    {
        // Zero is start orientation
        orientationTarget = orientationStart + orientationTarget;

        double degreeAllowedError = 0.4;  // allow for error if gyro position not precise enough
        double motorSpeedFull = 0.5;
        double motorSpeedFast = 0.35;
        double motorSpeedSlow = 0.15;

        double orientationCurrent = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if(orientationCurrent - orientationTarget > 180)
        {
            orientationTarget += 360;
        }
        else if(orientationTarget - orientationCurrent > 180)
        {
            orientationTarget -= 360;
        }

        telemetry.addData("start", orientationStart);
        telemetry.addData("target", orientationTarget);
        telemetry.addData("actual", orientationCurrent);
        telemetry.update();

        boolean rotationComplete = false;

        boolean rotateOn = true; // for testing (turns on and off wheel motors);

        while (!rotationComplete)
        {
            orientationCurrent = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (orientationTarget < -180 & orientationCurrent > 90) // gyro flipped from -180 side to +180 side
            {
                orientationTarget += 360;
            }
            else if (orientationTarget > 180 & orientationCurrent < -90) // gyro flipped from +180 side to -180 side
            {
                orientationTarget -= 360;
            }

            if (orientationCurrent < orientationTarget - 30)
            {
                // rotate left full speed
                if(rotateOn)
                {
                    motorFrontLeft.setPower(-1.0 * motorSpeedFull);
                    motorFrontRight.setPower(motorSpeedFull);
                    motorBackLeft.setPower(-1.0 * motorSpeedFull);
                    motorBackRight.setPower(motorSpeedFull);
                }
                telemetry.addLine("rotate left full speed");
            }
            else if (orientationCurrent > orientationTarget + 30)
            {
                // rotate right full speed
                if(rotateOn)
                {
                    motorFrontLeft.setPower(motorSpeedFull);
                    motorFrontRight.setPower(-1.0 * motorSpeedFull);
                    motorBackLeft.setPower(motorSpeedFull);
                    motorBackRight.setPower(-1.0 * motorSpeedFull);
                }
                telemetry.addLine("rotate right full speed");
            }
            else if (orientationCurrent < orientationTarget - 15)
            {
                // rotate left fast
                if(rotateOn)
                {
                    motorFrontLeft.setPower(-1.0 * motorSpeedFast);
                    motorFrontRight.setPower(motorSpeedFast);
                    motorBackLeft.setPower(-1.0 * motorSpeedFast);
                    motorBackRight.setPower(motorSpeedFast);
                }
                telemetry.addLine("rotate left fast");
            }
            else if (orientationCurrent > orientationTarget + 15)
            {
                // rotate right fast
                if(rotateOn)
                {
                    motorFrontLeft.setPower(motorSpeedFast);
                    motorFrontRight.setPower(-1.0 * motorSpeedFast);
                    motorBackLeft.setPower(motorSpeedFast);
                    motorBackRight.setPower(-1.0 * motorSpeedFast);
                }
                telemetry.addLine("rotate right fast");
            }
            else if (orientationCurrent < orientationTarget - degreeAllowedError)
            {
                // rotate left slow
                if(rotateOn)
                {
                    motorFrontLeft.setPower(-1.0 * motorSpeedSlow);
                    motorFrontRight.setPower(motorSpeedSlow);
                    motorBackLeft.setPower(-1.0 * motorSpeedSlow);
                    motorBackRight.setPower(motorSpeedSlow);
                }
                telemetry.addLine("rotate left slow");
            }
            else if (orientationCurrent > orientationTarget + degreeAllowedError)
            {
                // rotate right slow
                if(rotateOn)
                {
                    motorFrontLeft.setPower(motorSpeedSlow);
                    motorFrontRight.setPower(-1.0 * motorSpeedSlow);
                    motorBackLeft.setPower(motorSpeedSlow);
                    motorBackRight.setPower(-1.0 * motorSpeedSlow);
                }
                telemetry.addLine("rotate right slow");
            }
            else
            {
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                telemetry.addLine("stop rotation");
                sleep(200);

                orientationCurrent = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (orientationCurrent > orientationTarget - degreeAllowedError && orientationCurrent < orientationTarget + degreeAllowedError)
                {
                    // if orientation within error end while loop by setting rotationComplete to true

                    telemetry.addLine("end loop");
                    rotationComplete = true;
                }
            }


            telemetry.addData("start", orientationStart);
            telemetry.addData("target", orientationTarget);
            telemetry.addData("current", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

    public void RotateRobot (double degreeRotationAmount)
    {
        telemetry.addLine("rotate function");
        telemetry.update();

        double degreeAllowedError = 0.5;  // allow for error if gyro position not precise enough
        double motorSpeedFull = 0.5;
        double motorSpeedFast = 0.35;
        double motorSpeedSlow = 0.2;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double gyroOrientation = orientation.getYaw(AngleUnit.DEGREES);
        double targetOrientation = gyroOrientation + degreeRotationAmount;

        boolean rotationComplete = false;

        boolean rotateOn = true; // for testing (turns on and off wheel motors);

        while (!rotationComplete)
        {
            orientation = imu.getRobotYawPitchRollAngles();
            gyroOrientation = orientation.getYaw(AngleUnit.DEGREES);

            if (targetOrientation < -180 & gyroOrientation > 90) // gyro flipped from -180 side to +180 side
            {
                targetOrientation = targetOrientation + 360;
            }
            else if (targetOrientation > 180 & gyroOrientation < -90) // gyro flipped from +180 side to -180 side
            {
                targetOrientation = targetOrientation + 360;
            }

            telemetry.addData("target", targetOrientation);
            telemetry.addData("actual", gyroOrientation);
            telemetry.update();

            if (gyroOrientation < targetOrientation - 20)
            {
                // rotate left full speed
                if(rotateOn)
                {
                    motorFrontLeft.setPower(-1.0 * motorSpeedFull);
                    motorFrontRight.setPower(motorSpeedFull);
                    motorBackLeft.setPower(-1.0 * motorSpeedFull);
                    motorBackRight.setPower(motorSpeedFull);
                }
                telemetry.addLine("rotate left full speed");
            }
            else if (gyroOrientation > targetOrientation + 20)
            {
                // rotate right full speed
                if(rotateOn)
                {
                    motorFrontLeft.setPower(motorSpeedFull);
                    motorFrontRight.setPower(-1.0 * motorSpeedFull);
                    motorBackLeft.setPower(motorSpeedFull);
                    motorBackRight.setPower(-1.0 * motorSpeedFull);
                }
                telemetry.addLine("rotate right full speed");
            }
            else if (gyroOrientation < targetOrientation - 8)
            {
                // rotate left fast
                if(rotateOn)
                {
                    motorFrontLeft.setPower(-1.0 * motorSpeedFast);
                    motorFrontRight.setPower(motorSpeedFast);
                    motorBackLeft.setPower(-1.0 * motorSpeedFast);
                    motorBackRight.setPower(motorSpeedFast);
                }
                telemetry.addLine("rotate left fast");
            }
            else if (gyroOrientation > targetOrientation + 8)
            {
                // rotate right fast
                if(rotateOn)
                {
                    motorFrontLeft.setPower(motorSpeedFast);
                    motorFrontRight.setPower(-1.0 * motorSpeedFast);
                    motorBackLeft.setPower(motorSpeedFast);
                    motorBackRight.setPower(-1.0 * motorSpeedFast);
                }
                telemetry.addLine("rotate right fast");
            }
            else if (gyroOrientation < targetOrientation - degreeAllowedError)
            {
                // rotate left slow
                if(rotateOn)
                {
                    motorFrontLeft.setPower(-1.0 * motorSpeedSlow);
                    motorFrontRight.setPower(motorSpeedSlow);
                    motorBackLeft.setPower(-1.0 * motorSpeedSlow);
                    motorBackRight.setPower(motorSpeedSlow);
                }
                telemetry.addLine("rotate left slow");
            }
            else if (gyroOrientation > targetOrientation + degreeAllowedError)
            {
                // rotate right slow
                if(rotateOn)
                {
                    motorFrontLeft.setPower(motorSpeedSlow);
                    motorFrontRight.setPower(-1.0 * motorSpeedSlow);
                    motorBackLeft.setPower(motorSpeedSlow);
                    motorBackRight.setPower(-1.0 * motorSpeedSlow);
                }
                telemetry.addLine("rotate right slow");
            }
            else
            {
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                telemetry.addLine("stop rotation");
                sleep(100);  // short sleep to allow refresh

                if (gyroOrientation > targetOrientation - degreeAllowedError && gyroOrientation < targetOrientation + degreeAllowedError)
                {
                    // if orientation within error end while loop by setting rotationComplete to true

                    telemetry.addLine("end loop");
                    rotationComplete = true;
                }
            }
        }
    }
    public void MoveSlide(int slideTickPosition, String slideDirection)
    {

        double slideSpeed = 1.0;
        boolean inPosition = false;

        if(slideDirection.equalsIgnoreCase("up slow"))
        {
            slideSpeed = 0.5;
            slideDirection = "up";
        }
        else if(slideDirection.equalsIgnoreCase("down slow"))
        {
            slideSpeed = 0.3;
            slideDirection = "down";
        }

        motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!inPosition)
        {
            if(slideDirection.equalsIgnoreCase("up"))
            {
                if ((motorSlideLeft.getCurrentPosition()) < slideTickPosition)
                {
                    motorSlideLeft.setPower(slideSpeed);
                    motorSlideRight.setPower(slideSpeed);
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
                if(motorSlideLeft.getCurrentPosition() < 100)
                {
                    slideSpeed = 0.25;
                }

                if ((motorSlideLeft.getCurrentPosition()) > slideTickPosition)
                {
                    motorSlideLeft.setPower(slideSpeed * -1.0);
                    motorSlideRight.setPower(slideSpeed * -1.0);
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

    public double[] ReadAprilTagV2(int tagId, String placePosition)
    {
        double[] tagPosition = new double[2]; // return values (if Y 0.0 then failed to find tag)
        double aprilTagX = 0.0;
        double aprilTagY = 0.0;
        boolean tagFound = false;

        double orientationSquareToTag = 0;

        if(tagId >= 1 && tagId <= 6)
        {
            orientationSquareToTag = -90;
        }
        else if(tagId == 8 || tagId == 9)
        {
            orientationSquareToTag = 90;
        }

        RotateToOrientation(orientationSquareToTag);

        runtime.reset();

        while(tagFound == false && runtime.milliseconds() < 2000)  // allow 2 seconds to find tag
        {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
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
                        tagFound = true;
                        aprilTagX = detection.ftcPose.x;
                        aprilTagY = detection.ftcPose.y;
                    }

                }
                else
                {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                    aprilTagX = 0.0;
                    aprilTagY = 0.0;
                }
            }
        }

        tagPosition[0] = aprilTagX;
        tagPosition[1] = aprilTagY;

        if(!placePosition.equalsIgnoreCase("none"))
        {
            double distancePlacePosition = 0;
            double distanceFromSelectedTag = 0;

            if(tagPosition[1] == 0.0)
            {
                telemetry.addData("no tag", tagPosition[1]);
                // no tag detected after rotation
            }
            else
            {
                servoPivot.setPosition(servoPivotPlacePosition);// tilt claw
                MoveSlide(650, "up"); // raise arm

                if(tagId == 1 || tagId == 4)
                {
                    if(placePosition.equalsIgnoreCase("left"))
                    {
                        distanceFromSelectedTag = -3.0;
                    }
                    else if(placePosition.equalsIgnoreCase("middle"))
                    {
                        distanceFromSelectedTag = 6.0;
                    }
                    else if(placePosition.equalsIgnoreCase("right"))
                    {
                        distanceFromSelectedTag = 15.0;
                    }
                }
                else if(tagId == 2 || tagId == 5)
                {
                    if(placePosition.equalsIgnoreCase("left"))
                    {
                        distanceFromSelectedTag = -6.0;//-9.0 before
                    }
                    else if(placePosition.equalsIgnoreCase("middle"))
                    {
                        distanceFromSelectedTag = 0.0;
                    }
                    else if(placePosition.equalsIgnoreCase("right"))
                    {
                        distanceFromSelectedTag = 9.0;
                    }
                }
                else if(tagId == 3 || tagId == 6)
                {
                    if(placePosition.equalsIgnoreCase("left"))
                    {
                        distanceFromSelectedTag = -15.0;
                    }
                    else if(placePosition.equalsIgnoreCase("middle"))
                    {
                        distanceFromSelectedTag = -6.0;
                    }
                    else if(placePosition.equalsIgnoreCase("right"))
                    {
                        distanceFromSelectedTag = 3.0;
                    }
                }

                distancePlacePosition = distanceFromSelectedTag - tagPosition[0] + cameraOffsetX;

                if (distancePlacePosition < 0.0)
                {
                    // move left
                    DriveRobot("left", (distancePlacePosition * -1.0) , 0.5, "none");
                }
                else if (distancePlacePosition > 0.0)
                {
                    // move right
                    DriveRobot("right", distancePlacePosition, 0.5, "none");
                }

                DriveRobot("forward", tagPosition[1] - cameraOffsetY - 1, 0.5, "none");

                servoClaw2.setPosition(servoClaw2Open);
            }
        }

        return tagPosition;

    }
    public double[] ReadAprilTag(int tagId, String placePosition)
    {
        double[] tagPosition = new double[3]; // return values (if Y 0.0 then failed to find tag)
        double aprilTagX = 0.0;
        double aprilTagY = 0.0;

        int attemptCurrent = 1;
        int attemptMax = 3;

        boolean tagFound = false;
        boolean isSquareToTag = false;

        angleCorrecting = false;

        runtime.reset();

        while(isSquareToTag == false & runtime.milliseconds() < 3000 & attemptCurrent <= attemptMax)
        {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());
            telemetry.addData("current runtime", runtime.milliseconds());
            telemetry.update();

            //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            //gyroAngle = orientation.getYaw(AngleUnit.DEGREES);

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
                        if (detection.ftcPose.yaw > 1 || detection.ftcPose.yaw < -1) // check if square to tag
                        {
                            RotateRobot(detection.ftcPose.yaw * -1);
                            if (detection.ftcPose.yaw > 5 & detection.ftcPose.x > -1 & attemptCurrent < attemptMax)
                            {
                                DriveRobot("left", 2, 0.5, "none");
                                sleep(700);
                            }
                            else if (detection.ftcPose.yaw < -5 & detection.ftcPose.x < 1 & attemptCurrent < attemptMax)                            {
                                DriveRobot("right", 2, 0.5, "none");
                                sleep(700);
                            }
                            else
                            {
                                sleep(200);
                            }

                            if (attemptCurrent == attemptMax)
                            {
                                isSquareToTag = true;
                            }
                            attemptCurrent++;
                        }
                        else
                        {
                            isSquareToTag = true;
                        }
                        aprilTagX = detection.ftcPose.x;
                        aprilTagY = detection.ftcPose.y;
                        degSquare = detection.ftcPose.yaw;

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

                    aprilTagX = 0.0;
                    aprilTagY = 0.0;
                }

                //aprilTagX = detection.ftcPose.x;
                //aprilTagY = detection.ftcPose.y;

            }

        }

        tagPosition[0] = aprilTagX;
        tagPosition[1] = aprilTagY;
        tagPosition[2] = degSquare;

        if(!placePosition.equalsIgnoreCase("none"))
        {
            double distancePlacePosition = 0;
            double distanceFromSelectedTag = 0;

            if(tagPosition[1] == 0.0)
            {
                telemetry.addData("no tag", tagPosition[2]);
                // no tag detected after rotation
            }
            else
            {
                servoPivot.setPosition(servoPivotPlacePosition);// tilt claw
                MoveSlide(650, "up"); // raise arm

                if(tagId == 1 || tagId == 4)
                {
                    if(placePosition.equalsIgnoreCase("left"))
                    {
                        distanceFromSelectedTag = -3.0;
                    }
                    else if(placePosition.equalsIgnoreCase("middle"))
                    {
                        distanceFromSelectedTag = 6.0;
                    }
                    else if(placePosition.equalsIgnoreCase("right"))
                    {
                        distanceFromSelectedTag = 15.0;
                    }
                }
                else if(tagId == 2 || tagId == 5)
                {
                    if(placePosition.equalsIgnoreCase("left"))
                    {
                        distanceFromSelectedTag = -9.0;
                    }
                    else if(placePosition.equalsIgnoreCase("middle"))
                    {
                        distanceFromSelectedTag = 0.0;
                    }
                    else if(placePosition.equalsIgnoreCase("right"))
                    {
                        distanceFromSelectedTag = 9.0;
                    }
                }
                else if(tagId == 3 || tagId == 6)
                {
                    if(placePosition.equalsIgnoreCase("left"))
                    {
                        distanceFromSelectedTag = -15.0;
                    }
                    else if(placePosition.equalsIgnoreCase("middle"))
                    {
                        distanceFromSelectedTag = -6.0;
                    }
                    else if(placePosition.equalsIgnoreCase("right"))
                    {
                        distanceFromSelectedTag = 3.0;
                    }
                }

                distancePlacePosition = distanceFromSelectedTag - tagPosition[0] + cameraOffsetX;

                if (distancePlacePosition < 0.0)
                {
                    // move left
                    DriveRobot("left", (distancePlacePosition * -1.0) , 0.5, "none");
                }
                else if (distancePlacePosition > 0.0)
                {
                    // move right
                    DriveRobot("right", distancePlacePosition, 0.5, "none");
                }

                DriveRobot("forward", tagPosition[1] - cameraOffsetY - 2, 0.5, "none");

                servoClaw2.setPosition(servoClaw2Open);
            }
        }

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

    public void AutonomousLeftRed()
    {
        DriveRobot("forward", 2, 0.5, "none");// 5
        DriveRobot("right", 2, 0.5, "none");
        DriveRobot("forward", 12, 1, "claw - pivot grab");
        RotateToOrientation(55);
        DriveRobot("forward", 3, 0.5, "none");

        servoClaw1.setPosition(servoClaw1Open);// drop purple pixel
        sleep(200);

        DriveRobot("backward", 3, 0.5, "none");
        FlipClaw();
        RotateToOrientation(-90);
        DriveRobot("forward", 24, 0.5, "none");
        DriveRobot("left", 12, 0.5, "none");
        ReadAprilTagV2(5, "left");

        DriveRobot("backward", 2, 0.5, "none"); // drive robot backwards
        MoveSlide(0, "down");
        FlipClaw();
        servoRotate.setPosition(servoRotateTop);
    }

    public void AutonomousMidRed()
    {
        DriveRobot("forward", 2, 0.5, "none");// 5
        DriveRobot("right", 3, 0.5, "none");
        DriveRobot("forward", 17, 1, "claw - pivot grab"); //12

        servoClaw1.setPosition(servoClaw1Open);// drop purple pixel
        sleep(200);

        DriveRobot("backward", 2, 0.5, "none"); // drive robot backwards

        FlipClaw(); // ruby down

        DriveRobot("right", 25, 0.5, "none");
        DriveRobot("forward", 2, 0.5, "none");
        RotateToOrientation(-90);

        servoPivot.setPosition(servoPivotDrivePosition);// tilt claw

        double[] tagPosition = ReadAprilTagV2(6, "middle");

        DriveRobot("backward", 2, 0.5, "none");
        MoveSlide(0, "down");
        FlipClaw();
        servoRotate.setPosition(servoRotateTop);
    }

    public void AutonomousRightRed()
    {
        DriveRobot("forward", 2, 0.5, "none");// 5
        DriveRobot("right", 9, 0.5, "none");
        DriveRobot("forward", 8, 1, "claw - pivot grab"); //12

        servoClaw1.setPosition(servoClaw1Open);// drop purple pixel
        sleep(200);

        DriveRobot("backward", 2, 0.5, "none"); // drive robot backwards

        FlipClaw(); // ruby down

        DriveRobot("right", 15, 0.5, "none");
        DriveRobot("forward", 8, 0.5, "none");
        RotateToOrientation(-90);
        DriveRobot("left", 4, 0.5, "none");

        servoPivot.setPosition(servoPivotDrivePosition);// tilt claw up

        double[] tagPosition = ReadAprilTagV2(6, "right");

        DriveRobot("backward", 2, 0.5, "none");
        MoveSlide(0, "down");
        FlipClaw();
        servoRotate.setPosition(servoRotateTop);
    }

    public void AutonomousLeftBlue()
    {
        DriveRobot("forward", 2, 0.5, "none");// 5
        DriveRobot("left", 12, 0.5, "none");
        DriveRobot("forward", 8, 1, "claw - pivot grab"); //12

        servoClaw1.setPosition(servoClaw1Open);// drop purple pixel
        sleep(200);

        DriveRobot("backward", 2, 0.5, "none"); // drive robot backwards

        FlipClaw(); // ruby down

        DriveRobot("left", 15, 0.5, "none");
        DriveRobot("forward", 8, 0.5, "none");
        RotateToOrientation(-90);
        DriveRobot("left", 2, 0.5, "none");

        servoPivot.setPosition(servoPivotDrivePosition);// tilt claw up

        double[] tagPosition = ReadAprilTagV2(2, "left");
        sleep(200);

        DriveRobot("backward", 2, 0.5, "none");
        MoveSlide(0, "down");
        FlipClaw();
        servoRotate.setPosition(servoRotateTop);
    }

    public void AutonomousMidBlue()
    {
        DriveRobot("forward", 2, 0.5, "none");// 5
        DriveRobot("left", 3, 0.5, "none");
        DriveRobot("forward", 19, 1, "claw - pivot grab"); //12

        servoClaw1.setPosition(servoClaw1Open);// drop purple pixel
        sleep(200);

        DriveRobot("backward", 6, 0.5, "none"); // drive robot backwards

        FlipClaw(); // ruby down
        servoPivot.setPosition(servoPivotDrivePosition);

        DriveRobot("left", 25, 0.5, "none");
        DriveRobot("forward", 2, 0.5, "none");
        RotateToOrientation(-90);

        servoPivot.setPosition(servoPivotDrivePosition);// tilt claw

        double[] tagPosition = ReadAprilTagV2(2, "middle");

        DriveRobot("backward", 2, 0.5, "none");
        MoveSlide(0, "down");
        FlipClaw();
        servoRotate.setPosition(servoRotateTop);
    }

    public void AutonomousRightBlue()
    {
        DriveRobot("forward", 2, 0.5, "none");
        DriveRobot("left", 2, 0.5, "none");
        DriveRobot("forward", 12, 1, "claw - pivot grab");
        RotateToOrientation(125);
        DriveRobot("forward", 2, 0.5, "none");

        servoClaw1.setPosition(servoClaw1Open);// drop purple pixel
        sleep(200);

        DriveRobot("backward", 2, 0.5, "none");
        FlipClaw();
        servoPivot.setPosition(servoPivotDrivePosition);
        RotateToOrientation(-90);
        DriveRobot("forward", 24, 0.5, "none");
        //DriveRobot("right", 12, 0.5, "none");
        ReadAprilTagV2(2, "right");

        DriveRobot("backward", 2, 0.5, "none");
        MoveSlide(0, "down");
        FlipClaw();
        servoRotate.setPosition(servoRotateTop);
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
        orientationStart = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if(propColor.equalsIgnoreCase("blue"))
        {
            if(orientationStart < 180)
            {
                orientationStart += 180;
            }
            else
            {
                orientationStart -= 180;
            }
        }

        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoDroneLauncher.setPosition(servoDroneHoldPosition);
        cam.closeCameraDevice();
        telemetry.addData(("propPosition"), propPosition);
        telemetry.update();
        initAprilTag();

        if(propColor.equalsIgnoreCase("red"))
        {
            if (propPosition.equalsIgnoreCase("left"))
            {
                AutonomousLeftRed();
            }
            else if (propPosition.equalsIgnoreCase("middle"))
            {
                AutonomousMidRed();
            }
            else if (propPosition.equalsIgnoreCase("right"))
            {
                AutonomousRightRed();
            }
        }

        else if(propColor.equalsIgnoreCase("blue"))
        {
            if (propPosition.equalsIgnoreCase("left"))
            {
                AutonomousLeftBlue();
            }
            else if (propPosition.equalsIgnoreCase("middle"))
            {
                AutonomousMidBlue();
            }
            else if (propPosition.equalsIgnoreCase("right"))
            {
                AutonomousRightBlue();
            }
        }
        /*

        else if(propPosition.equalsIgnoreCase("test"))
        {
            DriveRobot("forward", 18.0, 0.6, "claw - pivot drive");
            RotateToOrientation(90);

            DriveRobot("forward", 6.0, 0.6, "none");

            double[] tagPosition = ReadAprilTagV2(9, "none");

            telemetry.addData("position0", tagPosition[0]);
            telemetry.addData("position1", tagPosition[1]);

            if(tagPosition[1] == 0.0)
            {
                telemetry.addData("no tag", tagPosition[1]);
                // no tag detected after rotation
            }
            else
            {
                if (tagPosition[0] - cameraOffsetX > 0)
                {
                    // move left
                    DriveRobot("left", tagPosition[0] - cameraOffsetX, 0.5, "none");
                }
                else
                {
                    // move right
                    DriveRobot("right", (tagPosition[0] - cameraOffsetX) * -1.0, 0.5, "none");
                }


                servoPivot.setPosition(servoPivotGrabPosition);// tilt claw
                servoClaw1.setPosition(servoClaw1Open);
                servoClaw2.setPosition(servoClaw2Open);

                MoveSlide(295, "up slow");
                sleep(400);
                DriveRobot("forward", tagPosition[1] - cameraOffsetY - 6.5, 0.3,"none");
                sleep(400);
                servoClaw1.setPosition(servoClaw1Closed);
                sleep(400);
                FlipClaw();
                sleep(400);
                MoveSlide(265, "down slow");
                sleep(400);
                servoClaw2.setPosition(servoClaw2Closed);
                sleep(400);
                DriveRobot("backward", 5.0, 0.3,"claw - pivot drive");
                sleep(400);
                FlipClaw();

                MoveSlide(0, "down slow");
                sleep(400);

                motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }

            telemetry.addData("x", tagPosition[0]);
            telemetry.addData("y", tagPosition[1]);
            telemetry.update();


            //RotateRobot(180);
        }
        else if(propPosition.equalsIgnoreCase("test2"))
        {
            RotateToOrientation(90);
            sleep(500);
            RotateToOrientation(-150);
            //sleep(3000);
            //RotateToOrientation(180);
            sleep(500);
            RotateToOrientation(0);
            sleep(500);
            telemetry.addData("start", orientationStart);
            telemetry.addData("current", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }
        */
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
        Scalar colorBlueLower1 = new Scalar(78, 158, 0);
        Scalar colorBlueUpper1 = new Scalar(138, 255, 255);

        Scalar colorRedLower1 = new Scalar(0, 50, 50);
        Scalar colorRedUpper1 = new Scalar(10, 255, 255);
        Scalar colorRedLower2 = new Scalar(175, 50, 20);
        Scalar colorRedUpper2 = new Scalar (180, 255, 255);

        public Mat processFrame(Mat input)
        {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2HSV);

            Rect rectLeft = new Rect(260, 250, 300, 300);
            Rect rectRight = new Rect(800, 280, 300, 300);

            input.copyTo(outPut);

            Imgproc.rectangle(outPut, rectLeft, color, 2);
            Imgproc.rectangle(outPut, rectRight, color, 2);

            matLeftCrop = YCbCr.submat(rectLeft);
            matRightCrop = YCbCr.submat(rectRight);

            Core.inRange(matLeftCrop, colorBlueLower1, colorBlueUpper1, matLeftCropBlue1);
            Core.inRange(matRightCrop, colorBlueLower1, colorBlueUpper1, matRightCropBlue1);

            Core.bitwise_not(matLeftCropBlue1, matLeftCropBlue1);
            Core.bitwise_not(matRightCropBlue1, matRightCropBlue1);

            Core.inRange(matLeftCrop, colorRedLower1, colorRedUpper1, matLeftCropRed1);
            Core.inRange(matRightCrop, colorRedLower1, colorRedUpper1, matRightCropRed1);
            Core.inRange(matLeftCrop, colorRedLower2, colorRedUpper2, matLeftCropRed2);
            Core.inRange(matRightCrop, colorRedLower2, colorRedUpper2, matRightCropRed2);

            Core.bitwise_not(matLeftCropRed1, matLeftCropRed1);
            Core.bitwise_not(matRightCropRed1, matRightCropRed1);
            Core.bitwise_not(matLeftCropRed2, matLeftCropRed2);
            Core.bitwise_not(matRightCropRed2, matRightCropRed2);

            int countMiddleBlue = 90000 - Core.countNonZero(matLeftCropBlue1);
            int countRightBlue = 90000 - Core.countNonZero(matRightCropBlue1);

            int countMiddleRed = 180000 - Core.countNonZero(matLeftCropRed1) - Core.countNonZero(matLeftCropRed2);
            int countRightRed = 180000 - Core.countNonZero(matRightCropRed1) - Core.countNonZero(matRightCropRed2);

            int countMiddle = 0;
            int countRight = 0;

            if (countMiddleValue == 0 && runtime.milliseconds() > 6000)
            {
                if(countMiddleRed > countMiddleBlue)
                {
                    // Red Prop
                    propColor = "red";
                    countMiddleValue = countMiddleRed;
                    countRightValue = countRightRed;
                }
                else
                {
                    // Blue Prop
                    propColor = "blue";
                    countMiddleValue = countMiddleBlue;
                    countRightValue = countRightBlue;
                }
            }

            if (propColor != "unknown")
            {
                if (propColor == "red")
                {
                    countMiddle = countMiddleRed;
                    countRight = countRightRed;
                }
                else
                {
                    countMiddle = countMiddleBlue;
                    countRight = countRightBlue;
                }

                if ((countMiddle - countMiddleValue < 2500 && countMiddle - countMiddleValue > -2500))
                {
                    propPosition = "middle";
                }
                else if (countRight - countRightValue > 2500 || countRight - countRightValue < -2500)
                {
                    propPosition = "right";
                }
                else
                {
                    propPosition = "left";
                }
            }
            else
            {
                propPosition = "unknown";
            }

            telemetry.addData(("Prop Position"), propPosition + " - " + propColor);
            if(propPosition == "unknown")
            {
                telemetry.addData("Calculating Middle Value", countMiddle);
                telemetry.addData("Calculating Middle Value", countRight);
            }
            else
            {
                telemetry.addData("Initial Middle Value", countMiddleValue);
                telemetry.addData("Initial Right Value", countRightValue);
                telemetry.addData("Middle Change", countMiddle - countMiddleValue);
                telemetry.addData("Right Change", countRight - countRightValue);
                telemetry.addLine("*** Safe to Start ***");
            }
            //telemetry.update();
            return(outPut);
        }
    }
}
