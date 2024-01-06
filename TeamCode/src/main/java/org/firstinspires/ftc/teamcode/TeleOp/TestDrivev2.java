package org.firstinspires.ftc.teamcode.Teleop;

//Imports


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class TestDrivev2 extends LinearOpMode
{

    double motorSpeed; // controls the speed of the robot

    // robot movement motors
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor FrontRight;

    // linear slide
    double armSpeed; // controls the speed of the arm
    private DcMotor SlideLeft;
    private DcMotor SlideRight;


    /*
    defines the servos used in the claw

    ClawServo1 is the bottom claw and ClawServo2 is the top claw
    the top claw is the side with black tape

    RotateServo rotates the claw and PivotServo moves the claw up and down

     */

    private Servo ClawServo1;
    private Servo ClawServo2;
    private Servo RotateServo;
    private Servo PivotServo;

    private Servo DroneLauncher;


    // defining variables for the driver controller
    float driveLeftStickY;
    float driveLeftStickX;
    float driveRightStickX;
    int driveButtonX;
    int driveButtonY;
    int driveButtonA;
    int driveButtonB;
    int driveRightBumper;
    float driveRightTrigger;
    float driveLeftTrigger;


    // defining variables for the arm controller
    float armRightStickY;
    float armLeftStickY;

    int armButtonA;
    int armButtonB;
    int armButtonX;
    int armButtonY;
    float armRightBumper;
    float armLeftBumper;

    float armLeftTrigger;

    int armButtonXStatus = 0; //determines if button X was pressed
    int armButtonYStatus = 0; //determines if button Y was pressed


    // linear slide variables
    int slideTickPosition = 0;
    int slideLeftPos = 0;
    int slideRightPos = 0;

    //servo position constants

    // bottom claw
    // closed is zero position
    final double servoClaw1Open = 0.7;
    final double servoClaw1Closed = 0.4;

    // top claw
    // closed is zero position
    final double servoClaw2Open = 0.85;
    final double servoClaw2Closed = 0.55;

    // pivot servo
    // grab position is zero position
    final double servoPivotPlacePosition = 0.35;
    final double servoPivotRotatePosition = 0.4;
    final double servoPivotGrabPosition = 0.56;

    // rotate servo     TOP IS THE SIDE WITH TAPE
    // top position is zero position
    final double servoRotateTop = 0.3;
    final double servoRotateBottom = 0.96;
    final double servoDroneLaunchPosition = 0.65;




    // miscellaneous variables
    double timeStart;
    int activeClaw = 1;
    int slideStatus = 1; //1=drive 2=pre-hang 3=hang
    int isHanging = 0;
    int clawPositionStatus = 0;//0=grab 1=rotate

    boolean activeX = false;

    int clawXPosition;


    //lower number = less pressure required to reach level
    final double lowSpeedPressure = 0.55;
    final double medSpeedPressure = 0.15;

    final double lowDriveSpeed = 0.2;
    final double medDriveSpeed = 0.35;
    final double highDriveSpeed = 0.55;

    public void claw()
    {
        armRightBumper = (gamepad2.right_bumper) ? 1 : 0;
        armLeftBumper = (gamepad2.left_bumper) ? 1 : 0;
        armButtonA = (gamepad2.a) ? 1 : 0;
        armButtonB = (gamepad2.b) ? 1 : 0;
        boolean armX = (gamepad2.x);
        armButtonY = (gamepad2.y) ? 1 : 0;

        // bottom claw
        if (activeClaw == 1 || armButtonB == 1)
        {
            if (armLeftBumper == 1)
            {
                ClawServo1.setPosition(servoClaw1Open);
            }
            if (armRightBumper == 1)
            {
                ClawServo1.setPosition(servoClaw1Closed);
            }
        }

        // top claw
        if (activeClaw == 2 || armButtonB == 1)
        {
            if (armLeftBumper == 1)
            {
                ClawServo2.setPosition(servoClaw2Open);//top claw
            }
            if (armRightBumper == 1)
            {
                ClawServo2.setPosition(servoClaw2Closed);
            }
        }

        //change active claw
        if(slideStatus == 1)
        {

            if(clawPositionStatus == 0 & armButtonY == 1)
            {
                PivotServo.setPosition(servoPivotRotatePosition);
                clawPositionStatus = 1;
                timeStart = System.currentTimeMillis();
            }


            else if(clawPositionStatus == 1 & System.currentTimeMillis() >= (timeStart + 300))
            {
                if(activeClaw == 1)
                {
                    RotateServo.setPosition(servoRotateBottom);
                }

                else
                {
                    RotateServo.setPosition(servoRotateTop);
                }

                clawPositionStatus = 2;
                timeStart = System.currentTimeMillis();
            }

            else if(clawPositionStatus == 2 & System.currentTimeMillis() >= (timeStart + 600))
            {

                PivotServo.setPosition(servoPivotGrabPosition);
                clawPositionStatus = 0;

                if(activeClaw == 1)
                {
                    activeClaw = 2;
                }

                else
                {
                    activeClaw = 1;
                }

            }
            else if(clawPositionStatus == 0)
            {

                if(SlideLeft.getCurrentPosition() * -1.0 >= 300)
                {
                    PivotServo.setPosition(servoPivotPlacePosition);
                }

                else
                {
                    PivotServo.setPosition(servoPivotGrabPosition);
                }

            }
        }
        else
        {
            PivotServo.setPosition(servoPivotGrabPosition);
        }


        if(armX)
        {
            PivotServo.setPosition(servoPivotRotatePosition);
        }
        if(armButtonA == 1)
        {
            PivotServo.setPosition(servoPivotGrabPosition);
        }

    }

    public void slide()
    {

        armRightStickY = gamepad2.right_stick_y;
        armLeftStickY = gamepad2.left_stick_y;
        armLeftTrigger = gamepad2.left_trigger;
        driveButtonA = (gamepad2.y) ? 1 : 0;

        if(armRightStickY < 0)  // Up
        {

            SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armSpeed = 0.8;
            SlideLeft.setPower(armRightStickY * armSpeed); //Moves arm according to the stick (should use encoder)
            SlideRight.setPower(armRightStickY * armSpeed);

            if(slideStatus == 3)
            {
                slideStatus = 1;
            }

        }

        else if(armRightStickY > 0) // Down
        {

            SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(SlideLeft.getCurrentPosition() * -1.0 >= 500)
            {
                armSpeed = 0.8;
            }

            else
            {
                armSpeed = 0.25;
            }

            SlideLeft.setPower(armRightStickY * armSpeed);
            SlideRight.setPower(armRightStickY * armSpeed);

            if(slideStatus == 3)
            {
                slideStatus = 1;
            }

        }
        else
        {

            SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SlideLeft.setPower(-0.05);
            SlideRight.setPower(-0.05);

        }

        if (armButtonX == 1 & armButtonXStatus == 0)
        {

            if (slideStatus == 1)
            {
                slideStatus = 2;
            }

            else if (slideStatus == 2)
            {
                slideStatus = 1;
            }

            armButtonXStatus = 1;
        }

        else if (armButtonX == 0)
        {
            armButtonXStatus = 0;
        }

        /*
        if (armButtonA == 1)
        {
            slideTickPosition = 500;
            slideStatus = 3;
        }
        */

        else if(slideStatus == 3)
        {
            if (isHanging == 0)
            {

                if ((SlideLeft.getCurrentPosition() * -1.0) > slideTickPosition)
                {

                    SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    SlideLeft.setPower(1);
                    SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    SlideRight.setPower(1);

                }

                else
                {

                    isHanging = 1;
                    slideLeftPos = SlideLeft.getCurrentPosition();
                    slideRightPos = SlideRight.getCurrentPosition();

                }

            }
            else if (isHanging == 1)
            {

                SlideLeft.setPower(0.5);
                SlideRight.setPower(0.5);
                SlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                SlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                SlideLeft.setTargetPosition(slideLeftPos);
                SlideRight.setTargetPosition(slideRightPos);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

        }

        if(armLeftTrigger >= 0.3)
        {
            SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }
    public void mecanum()
    {

        driveLeftStickY = gamepad1.left_stick_y;
        driveLeftStickX = gamepad1.left_stick_x;
        driveRightStickX = gamepad1.right_stick_x;
        driveButtonA = (gamepad1.a) ? 1 : 0;
        driveButtonB = (gamepad1.b) ? 1 : 0;
        driveButtonX = (gamepad1.x) ? 1 : 0;
        driveButtonY = (gamepad1.y) ? 1 : 0;
        driveRightBumper = (gamepad1.right_bumper) ? 1 : 0;
        driveRightTrigger = (gamepad1.right_trigger);
        driveLeftTrigger = (gamepad1.left_trigger);



        BackLeft.setPower((driveLeftStickY + driveLeftStickX - driveRightStickX) * motorSpeed);
        FrontLeft.setPower((driveLeftStickY - driveLeftStickX - driveRightStickX) * motorSpeed);
        BackRight.setPower((driveLeftStickY - driveLeftStickX + driveRightStickX) * motorSpeed);
        FrontRight.setPower((driveLeftStickY + driveLeftStickX + driveRightStickX) * motorSpeed);


        if (driveRightBumper == 1)
        {
            DroneLauncher.setPosition(servoDroneLaunchPosition);
        }

        motorSpeed = 1.0;
        if(driveLeftTrigger >= 0.2)
        {
            motorSpeed *= 0.2;
        }

        if(driveRightTrigger >= 0.2)
        {
            motorSpeed *= 0.5;
        }

        /*
        //low speed
        if(driveRightTrigger >= lowSpeedPressure && driveRightTrigger <= 1.0)
        {
            motorSpeed = lowDriveSpeed;
        }

        //med speed
        else if(driveRightTrigger >= medSpeedPressure && driveRightTrigger < lowSpeedPressure)
        {
            motorSpeed = medDriveSpeed;
        }

        else
        {
            motorSpeed = highDriveSpeed;
        }

        */


    }


    //press ctrl shift . Or right click in between {  } and click "Folding" and "Fold code block" to hide telemetry1 and initialize.
    public void telemetry1()
    {

        telemetry.addData("SlideLeft", SlideLeft.getCurrentPosition());
        telemetry.addData("SlideRight", SlideRight.getCurrentPosition());
        telemetry.addData("Tick", slideTickPosition);
        telemetry.addData("Hanging", isHanging);
        telemetry.addData("Y status", armButtonYStatus);
        telemetry.addData("Pivot", PivotServo.getPosition());
        telemetry.addData("Rotate", RotateServo.getPosition());
        telemetry.addData("Claw1", ClawServo1.getPosition());
        telemetry.addData("Claw2", ClawServo2.getPosition());
        telemetry.addData("right trigger", driveRightTrigger);

        telemetry.update();

    }

    public void initialize()
    {
        // setting motor hardware to variables
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        SlideLeft = hardwareMap.get(DcMotor.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotor.class, "SlideRight");

        // setting servo hardware to variables
        DroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");
        ClawServo1 = hardwareMap.get(Servo.class, "ClawServo1");
        ClawServo2 = hardwareMap.get(Servo.class, "ClawServo2");
        PivotServo = hardwareMap.get(Servo.class, "PivotServo");
        RotateServo = hardwareMap.get(Servo.class, "RotateServo");


        // set motor direction
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideRight.setDirection(DcMotorSimple.Direction.FORWARD);



        // set zero power behavior
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();


        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        sleep(200);
        PivotServo.setPosition(servoPivotRotatePosition);
        sleep(500);
        RotateServo.setPosition(servoRotateTop);
        activeClaw = 1;
        sleep(500);

        ClawServo2.setPosition(servoClaw2Open);
        ClawServo1.setPosition(servoClaw1Open);
        PivotServo.setPosition(servoPivotGrabPosition);
    }



    @Override
    public void runOpMode()
    {

        initialize();

        while(opModeIsActive())
        {

            mecanum();
            claw();
            slide();
            telemetry1();

        }

    }

}
