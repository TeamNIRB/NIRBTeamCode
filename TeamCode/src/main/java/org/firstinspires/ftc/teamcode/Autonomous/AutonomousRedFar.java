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

    double gyroAngle;
    double gyroTargetAngle;

    private DcMotor motorBackLeft;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorFrontRight;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;


    private DcMotor odometryPodLeft;
    private DcMotor odometryPodBack;
    private DcMotor odometryPodRight;

    private Servo servoClaw1;
    private Servo servoClaw2;
    private Servo servoRotate;
    private Servo servoPivot;


    public String propPosition = "Unknown";

    public int colorMiddleValue = 0;
    public int colorRightValue = 0;


    private final int camWidth = 1080;
    private final int camHeight = 720;

    double servoClaw1Open = 0.7;
    double servoClaw1Closed = 0.4;

    double servoClaw2Open = 0.85;
    double servoClaw2Closed = 0.55;

    double servoPivotPlacePosition = 0.35;
    double servoPivotRotatePosition = 0.4;
    double servoPivotGrabPosition = 0.56;


    private ElapsedTime runtime = new ElapsedTime();

    public final int ticks_to_inch = 336;


    double rotationX = -90;  // enter the desired X rotation angle here.
    double rotationY = -34;  // enter the desired Y rotation angle here.
    double rotationZ = 180;  // enter the desired Z rotation angle here.

    public void DriveRobot(String driveDirection, int driveDistance, double driveSpeed)
    {
        int targetLeft;
        int targetBack;

        targetLeft = odometryPodLeft.getCurrentPosition() + (int)(driveDistance * ticks_to_inch);
        targetBack = odometryPodBack.getCurrentPosition() + (int)(driveDistance * ticks_to_inch);

        odometryPodLeft.setTargetPosition(targetLeft);
        odometryPodBack.setTargetPosition(targetBack);

        if(driveDirection.equalsIgnoreCase("forward"))
        {
            while(odometryPodLeft.getCurrentPosition() < odometryPodLeft.getTargetPosition())
            {
                motorFrontRight.setPower(driveSpeed);
                motorFrontLeft.setPower(driveSpeed);
                motorBackRight.setPower(driveSpeed);
                motorBackLeft.setPower(driveSpeed);
            }

            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }
        else if(driveDirection.equalsIgnoreCase("backward"))
        {
            driveSpeed *= -1;
            while(odometryPodLeft.getCurrentPosition() > odometryPodLeft.getTargetPosition())
            {
                motorFrontRight.setPower(driveSpeed);
                motorFrontLeft.setPower(driveSpeed);
                motorBackRight.setPower(driveSpeed);
                motorBackLeft.setPower(driveSpeed);
            }

            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }
        else if(driveDirection.equalsIgnoreCase("left"))
        {
            targetBack = odometryPodBack.getCurrentPosition() + (int)((driveDistance * -1) * ticks_to_inch);
            odometryPodBack.setTargetPosition(targetBack);

            while(odometryPodBack.getCurrentPosition() > odometryPodBack.getTargetPosition())
            {
                motorFrontRight.setPower(driveSpeed);
                motorFrontLeft.setPower(driveSpeed * -1);
                motorBackRight.setPower(driveSpeed * -1);
                motorBackLeft.setPower(driveSpeed);
            }

            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
        }

        else if(driveDirection.equalsIgnoreCase("right"))
        {
            targetBack = odometryPodBack.getCurrentPosition() + (int)(driveDistance * ticks_to_inch);
            odometryPodBack.setTargetPosition(targetBack);

            while(odometryPodBack.getCurrentPosition() < odometryPodBack.getTargetPosition())
            {
                motorFrontRight.setPower(driveSpeed * -1);
                motorFrontLeft.setPower(driveSpeed);
                motorBackRight.setPower(driveSpeed);
                motorBackLeft.setPower(driveSpeed * -1);
            }

            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
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

            sleep(5000);
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
        DriveRobot("forward", 12, 0.5);
        TurnRobot("counter", 45, 0.2);
        DriveRobot("forward", 4, 0.5);
        servoPivot.setPosition(servoPivotGrabPosition);
        sleep(500);
        servoClaw1.setPosition(servoClaw1Open);
        sleep(1000);

        DriveRobot("backward", 4, 0.5);
        TurnRobot("clockwise", 45, 0.2);
        //DriveRobot("right", 34, 0.5);

    }

    public void AutonomousMid()
    {
        DriveRobot("forward", 5, 0.5);
        DriveRobot("right", 2, 0.5);
        DriveRobot("forward", 12, 0.5);

        servoPivot.setPosition(servoPivotGrabPosition);
        sleep(500);
        servoClaw1.setPosition(servoClaw1Open);

        sleep(1000);
        DriveRobot("right", 30, 0.5);
        servoClaw2.setPosition(servoClaw2Open);
    }

    public void AutonomousRight()
    {
        DriveRobot("forward", 12, 0.5);
        DriveRobot("right", 8, 0.5);
        servoPivot.setPosition(servoPivotGrabPosition);
        sleep(500);
        servoClaw1.setPosition(servoClaw1Open);

        sleep(1000);
        DriveRobot("right", 24, 0.5);
        servoClaw2.setPosition(servoClaw2Open);
    }
    public void RobotSetup()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        Orientation hubRotation = xyzOrientation(rotationX, rotationY, rotationZ);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));



        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

        servoClaw1 = hardwareMap.get(Servo.class, "ClawServo1");
        servoClaw2 = hardwareMap.get(Servo.class, "ClawServo2");
        servoPivot = hardwareMap.get(Servo.class, "PivotServo");
        servoRotate = hardwareMap.get(Servo.class, "RotateServo");


        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);


        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odometryPodRight = hardwareMap.get(DcMotor.class, "FrontRight");
        odometryPodLeft = hardwareMap.get(DcMotor.class, "PodLeft");
        odometryPodBack = hardwareMap.get(DcMotor.class, "FrontLeft");
    }

    public void init()
    {
        RobotSetup();

        servoClaw1.setPosition(servoClaw1Closed);
        servoClaw2.setPosition(servoClaw2Closed);


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

        cam.closeCameraDevice();
        telemetry.addData(("propPosition"), propPosition);

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

    }

    public void loop()
    {
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        //gyroAngle = orientation.getYaw(AngleUnit.DEGREES);
        //telemetry.addData("gyroangle", gyroAngle);
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

        private void initAprilTag()
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
        private void telemetryAprilTag()
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

                    if (detection.id == 9)
                    {
                        double degSquare = detection.ftcPose.yaw;
                        if (degSquare < 0)
                        {
                            motorFrontLeft.setPower(0.2);
                            motorFrontRight.setPower(-0.2);
                            motorBackLeft.setPower(-0.2);
                            motorBackRight.setPower(0.2);

                        }
                        else
                        {
                            motorFrontLeft.setPower(0);
                            motorFrontRight.setPower(0);
                            motorBackLeft.setPower(0);
                            motorBackRight.setPower(0);
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
        }
    }

}
