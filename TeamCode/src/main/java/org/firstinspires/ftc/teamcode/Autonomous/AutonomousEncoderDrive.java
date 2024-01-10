package org.firstinspires.ftc.teamcode.Autonomous;

//package org.firstinspires.ftc.teamcode;

// Autonomous test program using dead wheels
// Everything is currently untested

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class AutonomousEncoderDrive extends LinearOpMode
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

    private ElapsedTime runtime = new ElapsedTime();

    public void strafe(int inch)
    {

    }
    public void openClaw()
    {
        if(activeClaw == 1)
        {
            ClawServo1.setPosition(servoClaw1Open);
        }
        if(activeClaw == 2)
        {
            ClawServo2.setPosition(servoClaw2Open);
        }
    }

    public void closeClaw()
    {
        if(activeClaw == 1)
        {
            ClawServo1.setPosition(servoClaw1Closed);
        }
        if(activeClaw == 2)
        {
            ClawServo2.setPosition(servoClaw2Closed);
        }
    }

    //USES SLEEP TEMPORARILY
    public void flipClaw()
    {
        if(activeClaw == 1)
        {
            PivotServo.setPosition(servoPivotRotatePosition);
            sleep(200);
            RotateServo.setPosition(servoRotateBottom);
            sleep(200);
            PivotServo.setPosition(servoPivotGrabPosition);
            activeClaw = 2;
        }

        if(activeClaw == 2)
        {
            PivotServo.setPosition(servoPivotRotatePosition);
            sleep(200);
            RotateServo.setPosition(servoRotateTop);
            sleep(200);
            PivotServo.setPosition(servoPivotGrabPosition);
            activeClaw = 1;
        }
    }

    public void zeroPosition()
    {
        PivotServo.setPosition(servoPivotGrabPosition);
        ClawServo1.setPosition(servoClaw1Closed);
        ClawServo2.setPosition(servoClaw2Closed);
        sleep(200);
        RotateServo.setPosition(servoRotateTop);
        sleep(200);
        PivotServo.setPosition(servoPivotGrabPosition);
        activeClaw = 1;
    }

    public void autonomous()
    {
        zeroPosition();
        sleep(200);
        drive(5, 0.5, 5);
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


        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideRight.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
        Set brake behavior
        */

        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        drive(5, 0.5, 5);
    }

public void drive(int inches, double speed, double timeout)
{
    int targetLeft;
    int targetRight;

    if(opModeIsActive())
    {
        targetLeft = PodLeft.getCurrentPosition() + (int)(inches * ticks_to_inch);
        targetRight = PodRight.getCurrentPosition() + (int)(inches * ticks_to_inch);

        PodLeft.setTargetPosition(targetLeft);

        while(opModeIsActive() && PodLeft.getCurrentPosition() < PodLeft.getTargetPosition())
        {
            FrontRight.setPower(speed);
            FrontLeft.setPower(speed);
            BackLeft.setPower(speed);
            BackRight.setPower(speed);
        }

        runtime.reset();

        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);


        while(opModeIsActive() && (runtime.seconds() < timeout))
        {
            telemetry.addData("Running to",  " %7d :%7d", targetLeft,  targetRight);
            telemetry.addData("Currently at",  " at %7d :%7d", PodLeft.getCurrentPosition(), PodRight.getCurrentPosition());
            telemetry.update();
        }

    }
}
}
