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
    // robot movement motors
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    // linear slide
    private DcMotor motorSlideLeft;
    private DcMotor motorSlideRight;


    /*
    defines the servos used in the claw

    servoClaw1 is the bottom claw and servoClaw2 is the top claw
    the top claw is the side with ruby

    servoRotate rotates the claw and servoPivot moves the claw up and down

     */

    private Servo servoClaw1;
    private Servo servoClaw2;
    private Servo servoRotate;
    private Servo servoPivot;
    private Servo servoDroneLauncher;

    double motorSpeed; // controls the speed of the robot
    double armSpeed; // controls the speed of the arm

    // defining variables for the driver controller
    float driveLeftStickY;
    float driveLeftStickX;
    float driveRightStickX;
    boolean driveButtonX;
    boolean driveButtonY;
    boolean driveButtonA;
    boolean driveButtonB;
    boolean driveRightBumper;
    float driveRightTrigger;
    float driveLeftTrigger;

    // defining variables for the arm controller
    float armRightStickY;
    float armLeftStickY;

    boolean armButtonA;
    boolean armButtonB;
    boolean armButtonX;
    boolean armButtonY;
    boolean armRightBumper;
    boolean armLeftBumper;

    float armLeftTrigger;

    int armButtonXStatus = 0; //determines if button X was pressed
    int armButtonYStatus = 0; //determines if button Y was pressed

    // linear slide variables
    int slideTickPosition = 0;
    int motorSlideLeftPos = 0;
    int motorSlideRightPos = 0;

    //servo position constants
    //bottom claw
    final double servoClaw1Open = 0.71;  // - more closed
    final double servoClaw1Closed = 0.41;

    //top claw
    final double servoClaw2Open = 0.54; // - more closed
    final double servoClaw2Closed = 0.24;

    final double servoPivotPlacePosition = 0.70;
    final double servoPivotRotatePosition = 0.65;
    final double servoPivotDrivePosition = 0.55;
    final double servoPivotGrabPosition = 0.87; // + lowers angle

    //top is the side with ruby
    double servoRotateTop = 0.171; // + rotates clockwise
    double servoRotateBottom = 0.848;

    final double servoDroneHoldPosition = 0.5;
    final double servoDroneLaunchPosition = 0.15;


    // miscellaneous variables
    double timeStart;
    int activeClaw = 1;
    int slideStatus = 1; // 1=drive 2=pre-hang (no longer used) 3=hang
    boolean isHanging = false;
    int clawPositionStatus = 0; // 0=grab 1=rotate 2=finish rotate 3=drive

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
        armLeftBumper = gamepad2.left_bumper; // open claw(s)
        armRightBumper = gamepad2.right_bumper; // close claw(s)
        armButtonA = gamepad2.a; // lift and hang
        armButtonB = gamepad2.b; // operate both claws
        armButtonX = gamepad2.x; // tilt claw
        armButtonY = gamepad2.y; // flip claw

        // bottom claw
        if (activeClaw == 1 || armButtonB)
        {
            if (armLeftBumper)
            {
                servoClaw1.setPosition(servoClaw1Open);
            }
            if (armRightBumper)
            {
                servoClaw1.setPosition(servoClaw1Closed);
            }
        }

        // top claw
        if (activeClaw == 2 || armButtonB)
        {
            if (armLeftBumper)
            {
                servoClaw2.setPosition(servoClaw2Open);//top claw
            }
            if (armRightBumper)
            {
                servoClaw2.setPosition(servoClaw2Closed);
            }
        }

        //change active claw (flip claw)
        if(clawPositionStatus == 0 & armButtonY)
        {
            servoPivot.setPosition(servoPivotRotatePosition);
            clawPositionStatus = 1;
            timeStart = System.currentTimeMillis();
        }
        else if(clawPositionStatus == 1 & System.currentTimeMillis() >= (timeStart + 300))  // add delay without sleep
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
        else if(clawPositionStatus == 2 & System.currentTimeMillis() >= (timeStart + 600))
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

        }
        else if(clawPositionStatus == 0)
        {
            if (motorSlideLeft.getCurrentPosition() * -1.0 >= 300)
            {
                servoPivot.setPosition(servoPivotPlacePosition);
            }
            else
            {
                servoPivot.setPosition(servoPivotGrabPosition);
            }
        }
        else if(clawPositionStatus == 3)
        {
            if (motorSlideLeft.getCurrentPosition() * -1.0 >= 300)
            {
                servoPivot.setPosition(servoPivotGrabPosition);
            }
            else
            {
                servoPivot.setPosition(servoPivotDrivePosition);
            }
        }

        if (armButtonX & armButtonXStatus == 0 & clawPositionStatus != 3)
        {
            clawPositionStatus = 3;
            armButtonXStatus = 1;
        }
        else if (armButtonX & armButtonXStatus == 0 & clawPositionStatus == 3)
        {
            clawPositionStatus = 0;
            armButtonXStatus = 1;
        }
        else if (!armButtonX)
        {
            armButtonXStatus = 0;
        }

    }

    public void slide()
    {
        armLeftStickY = gamepad2.left_stick_y;
        armRightStickY = gamepad2.right_stick_y;
        armLeftTrigger = gamepad2.left_trigger;
        driveButtonA = gamepad2.y;

        if(armRightStickY < 0)  // Up
        {
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armSpeed = 1.0;
            motorSlideLeft.setPower(armRightStickY * armSpeed); //Moves arm according to the stick (should use encoder)
            motorSlideRight.setPower(armRightStickY * armSpeed);
        }
        else if(armRightStickY > 0) // Down
        {
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(motorSlideLeft.getCurrentPosition() * -1.0 >= 500)
            {
                armSpeed = 1.0;
            }
            else
            {
                armSpeed = 0.25;
            }

            motorSlideLeft.setPower(armRightStickY * armSpeed);
            motorSlideRight.setPower(armRightStickY * armSpeed);

            if(slideStatus == 3) // release hanging robot
            {
                slideStatus = 1;
                isHanging = false;
            }
        }
        else
        {
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorSlideLeft.setPower(-0.05);
            motorSlideRight.setPower(-0.05);
        }

        if (armButtonA)
        {
            slideTickPosition = 500;
            slideStatus = 3;
        }
        else if(slideStatus == 3)
        {
            if (isHanging)
            {
                if ((motorSlideLeft.getCurrentPosition() * -1.0) > slideTickPosition)
                {
                    motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorSlideLeft.setPower(1);
                    motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorSlideRight.setPower(1);
                }
                else
                {
                    isHanging = true;
                    motorSlideLeftPos = motorSlideLeft.getCurrentPosition();
                    motorSlideRightPos = motorSlideRight.getCurrentPosition();
                }
            }
            else if (isHanging)
            {
                motorSlideLeft.setPower(0.5);
                motorSlideRight.setPower(0.5);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorSlideLeft.setTargetPosition(motorSlideLeftPos);
                motorSlideRight.setTargetPosition(motorSlideRightPos);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        if(armLeftTrigger >= 0.3)
        {
            motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void mecanum()
    {
        driveLeftStickY = gamepad1.left_stick_y;
        driveLeftStickX = gamepad1.left_stick_x;
        driveRightStickX = gamepad1.right_stick_x;
        driveButtonA = gamepad1.a;
        driveButtonB = gamepad1.b;
        driveButtonX = gamepad1.x;
        driveButtonY = gamepad1.y;
        driveRightBumper = gamepad1.right_bumper;
        driveLeftTrigger = gamepad1.left_trigger;
        driveRightTrigger = gamepad1.right_trigger;

        motorFrontLeft.setPower((driveLeftStickY - driveLeftStickX - driveRightStickX) * motorSpeed);
        motorFrontRight.setPower((driveLeftStickY + driveLeftStickX + driveRightStickX) * motorSpeed);
        motorBackLeft.setPower((driveLeftStickY + driveLeftStickX - driveRightStickX) * motorSpeed);
        motorBackRight.setPower((driveLeftStickY - driveLeftStickX + driveRightStickX) * motorSpeed);

        // launch drone
        if (driveRightBumper)
        {
            servoDroneLauncher.setPosition(servoDroneLaunchPosition);
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
    }

    //press ctrl shift . Or right click in between {  } and click "Folding" and "Fold code block" to hide telemetry1 and initialize.
    public void telemetry1()
    {
        telemetry.addData("SlideLeft", motorSlideLeft.getCurrentPosition());
        telemetry.addData("SlideRight", motorSlideRight.getCurrentPosition());
        telemetry.addData("Tick", slideTickPosition);
        telemetry.addData("Hanging", isHanging);
        telemetry.addData("Y status", armButtonYStatus);
        telemetry.addData("Claw1", servoClaw1.getPosition());
        telemetry.addData("Claw2", servoClaw2.getPosition());
        telemetry.addData("Pivot", servoPivot.getPosition());
        telemetry.addData("Rotate", servoRotate.getPosition());
        telemetry.addData("Right Trigger", driveRightTrigger);

        telemetry.update();
    }

    public void initialize()
    {
        // setting motor hardware to variables
        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");
        motorSlideLeft = hardwareMap.get(DcMotor.class, "SlideLeft");
        motorSlideRight = hardwareMap.get(DcMotor.class, "SlideRight");

        // setting servo hardware to variables
        servoClaw1 = hardwareMap.get(Servo.class, "ClawServo1");
        servoClaw2 = hardwareMap.get(Servo.class, "ClawServo2");
        servoPivot = hardwareMap.get(Servo.class, "PivotServo");
        servoRotate = hardwareMap.get(Servo.class, "RotateServo");
        servoDroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");

        // set motor direction
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // set zero power behavior
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sleep(200);
        servoPivot.setPosition(servoPivotRotatePosition);
        sleep(500);
        servoRotate.setPosition(servoRotateTop);
        activeClaw = 1;
        sleep(500);

        servoClaw1.setPosition(servoClaw1Open);
        servoClaw2.setPosition(servoClaw2Open);
        servoPivot.setPosition(servoPivotGrabPosition);
    }

    @Override
    public void runOpMode()
    {
        initialize();
        servoDroneLauncher.setPosition(0);

        while(opModeIsActive())
        {
            mecanum();
            claw();
            slide();
            telemetry1();
        }

    }

}
