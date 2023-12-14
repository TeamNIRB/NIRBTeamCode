package org.firstinspires.ftc.teamcode.Autonomous;
package org.firstinspires.ftc.teamcode;

// Autonomous test program using dead wheels
// Everything is currently untested

//Imports

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    public void moveForward(int inches, double driveSpeed)
    {

        int currentDistance = PodLeft.getCurrentPosition() / ticks_to_inch;
        int target = currentDistance + inches;

        while(currentDistance < target)
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

    public void moveBackward(int inches, double driveSpeed)
    {

        int currentDistance = PodLeft.getCurrentPosition() / ticks_to_inch;
        int target = currentDistance + inches;
        driveSpeed *= -1;

        while(currentDistance < target)
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

    public void strafeLeft(int inches, double driveSpeed)
    {

        int currentDistance = PodBack.getCurrentPosition() / ticks_to_inch;
        int target = currentDistance + inches;
        while(currentDistance < target)
        {
            BackLeft.setPower(driveSpeed);
            BackRight.setPower(driveSpeed);
            FrontLeft.setPower(driveSpeed *= -1);
            FrontRight.setPower(driveSpeed *= -1);
        }
        BackLeft.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
    }

    public void strafeRight(int inches, double driveSpeed)
    {
        int currentDistance = PodBack.getCurrentPosition() / ticks_to_inch;
        int target = currentDistance + inches;
        while(currentDistance < target)
        {
            BackLeft.setPower(driveSpeed *= -1);
            BackRight.setPower(driveSpeed *= -1);
            FrontLeft.setPower(driveSpeed);
            FrontRight.setPower(driveSpeed);
        }
        BackLeft.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
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
        //movement test code
        moveForward(3, 0.5);
        sleep(500);
        moveBackward(3, 0.5);
        sleep(500);
        strafeLeft(3, 0.5);
        sleep(500);
        strafeRight(3, 0.5);
        sleep(500);
        
        //claw test code
        openClaw();
        sleep(200);
        flipClaw();
        sleep(200);
        openClaw();
        sleep(200);
        closeClaw();
        sleep(200);
        flipClaw();
        sleep(200);
        closeClaw();
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

        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive())
        {
            autonomous();

            telemetry.update();
        }
    }
}
