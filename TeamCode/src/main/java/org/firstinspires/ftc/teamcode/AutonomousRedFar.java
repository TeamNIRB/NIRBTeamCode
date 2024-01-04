
package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.*;
import org.openftc.easyopencv.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.PWMOutputEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous
public class ImageTest extends OpMode
{
    OpenCvWebcam cam = null;

    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor FrontRight;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;


    private DcMotor PodLeft;
    private DcMotor PodBack;
    private DcMotor PodRight;

    private Servo ClawServo1;
    private Servo ClawServo2;
    private Servo RotateServo;
    private Servo PivotServo;


    public String propPosition = "Unknown";

    public int middleColorValue = 0;
    public int rightColorValue = 0;


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

    public void drive(String direction, int inches, double speed)
    {
        int targetLeft;
        int targetBack;

        targetLeft = PodLeft.getCurrentPosition() + (int)(inches * ticks_to_inch);
        targetBack = PodBack.getCurrentPosition() + (int)(inches * ticks_to_inch);

        PodLeft.setTargetPosition(targetLeft);
        PodBack.setTargetPosition(targetBack);

        if(direction.equalsIgnoreCase("forward"))
        {
            while(PodLeft.getCurrentPosition() < PodLeft.getTargetPosition())
            {
                FrontRight.setPower(speed);
                FrontLeft.setPower(speed);
                BackRight.setPower(speed);
                BackLeft.setPower(speed);
            }


            FrontRight.setPower(0);
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }

        else if(direction.equalsIgnoreCase("backward"))
        {
            speed *= -1;
            while(PodLeft.getCurrentPosition() < PodLeft.getTargetPosition())
            {
                FrontRight.setPower(speed);
                FrontLeft.setPower(speed);
                BackRight.setPower(speed);
                BackLeft.setPower(speed);
            }

            FrontRight.setPower(0);
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }
        else if(direction.equalsIgnoreCase("left"))
        {
            targetBack = PodBack.getCurrentPosition() + (int)((inches * -1) * ticks_to_inch);
            PodBack.setTargetPosition(targetBack);


            while(PodBack.getCurrentPosition() > PodBack.getTargetPosition())
            {
                FrontRight.setPower(speed);
                FrontLeft.setPower(speed * -1);
                BackRight.setPower(speed * -1);
                BackLeft.setPower(speed);

            }

            FrontRight.setPower(0);
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }

        else if(direction.equalsIgnoreCase("right"))
        {

            while(PodBack.getCurrentPosition() < PodBack.getTargetPosition())
            {
                FrontRight.setPower(speed * -1);
                FrontLeft.setPower(speed);
                BackRight.setPower(speed);
                BackLeft.setPower(speed * -1);

            }

            FrontRight.setPower(0);
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }

        else if(direction.equalsIgnoreCase("rotate"))
        {

        }





        //telemetry.addData("podleft", PodLeft.getCurrentPosition());
        //telemetry.addData("target", PodLeft.getTargetPosition());


    }

    public void sleep(int millis)
    {
        runtime.reset();
        while(runtime.milliseconds() < millis)
        {
            // lole ! ! ! ! !
        }
    }

    public void autonomousLeft()
    {
        //fill with autonomous script.
    }

    public void autonomousMid()
    {
        //drive(5, 0.5);
        telemetry.addLine("running middle");
        sleep(2000);
        telemetry.addLine("sleep");

    }

    public void autonomousRight()
    {
        //fill with autonomous script.
    }
    public void setup()
    {
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

        ClawServo1 = hardwareMap.get(Servo.class, "ClawServo1");
        ClawServo2 = hardwareMap.get(Servo.class, "ClawServo2");
        PivotServo = hardwareMap.get(Servo.class, "PivotServo");
        RotateServo = hardwareMap.get(Servo.class, "RotateServo");


        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);


        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PodRight = hardwareMap.get(DcMotor.class, "FrontRight");
        PodLeft = hardwareMap.get(DcMotor.class, "PodLeft");
        PodBack = hardwareMap.get(DcMotor.class, "FrontLeft");
    }

    public void init()
    {
        setup();

        ClawServo1.setPosition(servoClaw1Closed);
        ClawServo2.setPosition(servoClaw2Closed);


        runtime.reset();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        cam.setPipeline(new propRecognition());

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

        drive("left", 20, 0.5);
        /*

        if(propPosition.equalsIgnoreCase("left"))
        {
            autonomousLeft();
        }

        if(propPosition.equalsIgnoreCase("middle"))
        {
            autonomousMid();
        }

        if(propPosition.equalsIgnoreCase("right"))
        {
            autonomousRight();
        }


         */
    }

    public void loop()
    {
        //
    }


    class propRecognition extends OpenCvPipeline
    {
        Mat YCbCr = new Mat();
        Mat outPut = new Mat();

        Scalar color = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input)
        {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2HSV);

            Rect leftRect = new Rect(212, 100, 200, 200);
            //Rect middleRect = new Rect(425, 1, 424, 719);
            Rect rightRect = new Rect(820, 115, 200, 200);

            input.copyTo(outPut);

            Imgproc.rectangle(outPut, leftRect, color, 2);
            //Imgproc.rectangle(outPut, middleRect, color, 2);
            Imgproc.rectangle(outPut, rightRect, color, 2);


            Mat leftCrop = YCbCr.submat(leftRect);
            //Mat middleCrop = YCbCr.submat(middleRect);
            Mat rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            //Core.extractChannel(middleCrop, middleCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);


            Scalar leftavg = Core.mean(leftCrop);
            //Scalar midavg = Core.mean(middleCrop);
            Scalar rightavg = Core.mean(rightCrop);

            telemetry.addData("middleVal", (int)leftavg.val[0]);
            telemetry.addData("rightVal", (int)rightavg.val[0]);
            //telemetry.addData("middleVal", (int)midavg.val[0]);

            if (middleColorValue == 0 && runtime.milliseconds() > 6000)
            {
                middleColorValue = (int)leftavg.val[0];
                rightColorValue = (int)rightavg.val[0];
            }
            else if (middleColorValue > (int) leftavg.val[0] + 5)
            {
                propPosition = "Middle";
            }
            else if (rightColorValue > (int) rightavg.val[0] + 5)
            {
                propPosition = "Right";
            }
            else
            {
                propPosition = "Left";
            }
            telemetry.addData("middlestart", middleColorValue);
            telemetry.addData("rightstart", rightColorValue);
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
                            FrontLeft.setPower(0.2);
                            FrontRight.setPower(-0.2);
                            BackLeft.setPower(-0.2);
                            BackRight.setPower(0.2);

                        }
                        else
                        {
                            FrontLeft.setPower(0);
                            FrontRight.setPower(0);
                            BackLeft.setPower(0);
                            BackRight.setPower(0);
                        }
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

            if (currentDetections == null)
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


        }   // end method telemetryAprilTag()

    }

}
