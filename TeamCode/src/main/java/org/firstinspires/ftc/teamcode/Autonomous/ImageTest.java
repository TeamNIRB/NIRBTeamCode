
package org.firstinspires.ftc.teamcode.Autonomous;

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


@Autonomous
public class ImageTest extends OpMode
{
    OpenCvWebcam cam = null;

    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor FrontRight;

    public boolean isLeft = false;
    public boolean isRight = false;
    public boolean isMiddle = false;


    private final int camWidth = 1080;
    private final int camHeight = 720;

    private ElapsedTime runtime = new ElapsedTime();

    public void autonomousLeft()
    {
        //fill with autonomous script.
    }

    public void autonomousMid()
    {
        //fill with autonomous script.
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


        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);


        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void init()
    {
        setup();

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


    public void loop()
    {

        if(isMiddle)
        {
            autonomousMid();
        }

        if(isLeft)
        {
            autonomousLeft();
        }

        if(isRight)
        {
            autonomousRight();
        }

    }


    class propRecognition extends OpenCvPipeline
    {
        Mat YCbCr = new Mat();
        Mat outPut = new Mat();

        Scalar color = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input)
        {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2HSV);

            Rect leftRect = new Rect(1, 1, 424, 719);
            Rect middleRect = new Rect(425, 1, 424, 719);
            Rect rightRect = new Rect(850, 1, 424, 719);

            input.copyTo(outPut);

            Imgproc.rectangle(outPut, leftRect, color, 2);
            Imgproc.rectangle(outPut, middleRect, color, 2);
            Imgproc.rectangle(outPut, rightRect, color, 2);


            Mat leftCrop = YCbCr.submat(leftRect);
            Mat middleCrop = YCbCr.submat(middleRect);
            Mat rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(middleCrop, middleCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);


            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(middleCrop);
            Scalar rightavg = Core.mean(rightCrop);


                //check left
            if( (int) (leftavg.val[0]) > (int) (rightavg.val[0]) )
            {
                telemetry.addData("leftVal", (int)leftavg.val[0]);
                telemetry.addData("rightVal", (int)rightavg.val[0]);
                isLeft = true;
                telemetry.addData("Left", isLeft);
            }

                //check right
            if((int) (rightavg.val[0]) > (int) (leftavg.val[0]))
            {
                telemetry.addData("leftVal", (int) leftavg.val[0]);
                telemetry.addData("rightVal", (int) rightavg.val[0]);
                isRight = true;
                telemetry.addData("Right", isRight);
            }

                //check middle
            if((int)(midavg.val[0]) > (int) leftavg.val[0] && (int) (midavg.val[0]) > (int) (rightavg.val[0]))
            {
                isMiddle = true;
                telemetry.addData("Middle", isMiddle);
            }
            return(outPut);
        }
    }
}
