package org.firstinspires.ftc.teamcode;

//NIRB Team main drive program -- Mecanum Drive; Pivot Arm; Vertical Griper

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Autonomous

public class testpod extends LinearOpMode
{

    /*
    Declaring all of the hardware
    */
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor FrontRight;
    private DcMotor SlideLeft;
    private DcMotor SlideRight;
    private DcMotor PodLeft;
    private DcMotor PodBack;
    private DcMotor PodRight;
    private Servo ClawServo1;
    private Servo ClawServo2;
    private Servo RotateServo;
    private Servo PivotServo;


    int activeClaw = 1;
    double motorSpeed; //Controls the speed of the motors throughout the program
    double armSpeed; //Controls the speed of the arm throughout the program

    double timeStart;
    int slideTickPosition = 0;
    int clawPositionStatus = 0;

    //servo position variables

    //bottom claw
    double servoClaw1Open = 0.7;
    double servoClaw1Closed = 0.4;

    double servoClaw2Open = 0.85;
    double servoClaw2Closed = 0.55;

    double servoPivotPlacePosition = 0.35;
    double servoPivotRotatePosition = 0.4;
    double servoPivotGrabPosition = 0.56;

    //top is the side with tape
    double servoRotateTop = 0.3;
    double servoRotateBottom = 0.96;
    //0.3

    final int ticks_to_inch = 336;

    public void moveForward(int targetDistance, double driveSpeed)
    {
        targetDistance /= ticks_to_inch;
        int currentDistance = PodLeft.getCurrentPosition() / ticks_to_inch;

        while(currentDistance <= targetDistance)
        {
            BackLeft.setPower(driveSpeed);
            BackRight.setPower(driveSpeed);
            FrontLeft.setPower(driveSpeed);
            FrontRight.setPower(driveSpeed);
        }
        BackLeft.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
    }

    public void autonomous()
    {
        moveForward(3, 0.5);

    }



    @Override
    public void runOpMode()
    {

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        SlideLeft = hardwareMap.get(DcMotor.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotor.class, "SlideRight");
        PodRight = hardwareMap.get(DcMotor.class, "FrontRight");
        PodLeft = hardwareMap.get(DcMotor.class, "PodLeft");
        PodBack = hardwareMap.get(DcMotor.class, "FrontLeft");

        ClawServo1 = hardwareMap.get(Servo.class, "ClawServo1");
        ClawServo2 = hardwareMap.get(Servo.class, "ClawServo2");
        PivotServo = hardwareMap.get(Servo.class, "PivotServo");
        RotateServo = hardwareMap.get(Servo.class, "RotateServo");

        /*
        Reverse motors
        */

        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //Actuator.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
        Set brake behavior
        */

        //PivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        while(opModeIsActive())
        {
            autonomous();
            telemetry.addData("SlideLeft", SlideLeft.getCurrentPosition());
            telemetry.addData("SlideRight", SlideRight.getCurrentPosition());
            telemetry.addData("Tick", slideTickPosition);
            telemetry.addData("Pivot", PivotServo.getPosition());
            telemetry.addData("Rotate", RotateServo.getPosition());
            telemetry.addData("Claw1", ClawServo1.getPosition());
            telemetry.addData("Claw2", ClawServo2.getPosition());

            telemetry.update();
        }
    }
}
