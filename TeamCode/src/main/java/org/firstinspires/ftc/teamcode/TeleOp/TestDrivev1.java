//NIRB Team main drive program -- Mecanum Drive; Pivot Arm; Vertical Griper

//Imports

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.acmerobotics.roadrunner.geometry.Pose2d;

@TeleOp

public class TestDrivev1 extends LinearOpMode {

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
    private DcMotor PodRight;
    private DcMotor PodBack;
    //private DcMotor Actuator;
    //private DcMotor PivotMotor;
    private Servo ClawServo1;
    private Servo ClawServo2;
    private Servo RotateServo;
    private Servo PivotServo;
    private Servo DroneLauncher;

    /*
    Declaring all of the Contoller inputs
    */

    float driveLeftStickY;
    float driveLeftStickX;
    float armRightStickY;
    float driveRightStickX;

    float driveButtonX;
    float driveButtonY;
    float driveButtonA;
    float driveButtonB;
    float driveRightBumper;
    float driveRightTrigger;

    float armRightBumper;
    float armLeftBumper;
    float slidePosition;
    float armLeftStickY;

    int activeClaw = 1;
    double motorSpeed; //Controlls the speed of the motors throughout the program
    double armSpeed; //Controlls the speed of the arm throughout the program
    //double ActuatorSpeed;
    int armButtonA;
    int armButtonB;
    int armButtonX;
    int armButtonXStatus = 0; //determines if button X was pressed
    int armButtonY;
    int armButtonYStatus = 0; //determines if button Y was pressed
    int isHanging = 0;
    double timeStart;
    int slideStatus = 1; //1=drive 2=pre-hang 3=hang
    int slideTickPosition = 0;
    int slideLeftPos = 0;
    int slideRightPos = 0;

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




    public void claw()
    {
        armRightBumper = (gamepad2.right_bumper) ? 1 : 0;
        armLeftBumper = (gamepad2.left_bumper) ? 1 : 0;
        armButtonA = (gamepad2.a) ? 1 : 0;
        armButtonB = (gamepad2.b) ? 1 : 0;
        armButtonX = (gamepad2.x) ? 1 : 0;
        armButtonY = (gamepad2.y) ? 1 : 0;

        if (activeClaw == 1 || armButtonB == 1)
        {
            if (armLeftBumper == 1)
            {
                ClawServo1.setPosition(servoClaw1Open);
            }
            if (armRightBumper == 1)
            {
                ClawServo1.setPosition(servoClaw1Closed);//bottom claw
            }
        }
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


        /*
        if(armButtonA == 1)
        {
            RotateServo.setPosition(servoRotateBottom);
        }
        if(armButtonX == 1)
        {
            RotateServo.setPosition(servoRotateTop);
        }
        */
    }

    public void slide()
    {
        armRightStickY = gamepad2.right_stick_y;
        armLeftStickY = gamepad2.left_stick_y;
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

            SlideLeft.setPower(armRightStickY * armSpeed); //Moves arm according to the stick (should use encoder)
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
            SlideLeft.setPower(-0.05); //Moves arm according to the stick (should use encoder)
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

        if (armButtonA == 1)
        {
            slideTickPosition = 500;
            slideStatus = 3;
        }

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


                //SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //SlideLeft.setPower(0.4);
                //SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //SlideRight.setPower(0.4);

            }
        }
    }
    /*
    public boolean flipClaw(int activeClawTarget)
    {
        if(activeClawTarget == 1)
        {

        }
        else
        {

        }
    }
    */

    public void mecanum()
    { //Function resposible for controlling drive motors

        /*
        Correlating declared variable names to controller inputs
        */

        driveLeftStickY = gamepad1.left_stick_y;
        driveLeftStickX = gamepad1.left_stick_x;
        driveRightStickX = gamepad1.right_stick_x;
        driveButtonA = (gamepad1.a) ? 1 : 0;
        driveButtonB = (gamepad1.b) ? 1 : 0;
        driveButtonX = (gamepad1.x) ? 1 : 0;
        driveButtonY = (gamepad1.y) ? 1 : 0;
        driveRightBumper = (gamepad1.right_bumper) ? 1 : 0;

        armRightStickY = gamepad2.right_stick_y;
        armLeftStickY = gamepad2.left_stick_y;
        armRightBumper = (gamepad2.right_bumper) ? 1 : 0;
        armLeftBumper = (gamepad2.left_bumper) ? 1 : 0;
        armButtonA = (gamepad2.a) ? 1 : 0;
        armButtonB = (gamepad2.b) ? 1 : 0;
        armButtonX = (gamepad2.x) ? 1 : 0;
        armButtonY = (gamepad2.y) ? 1 : 0;



        /*
        Bolean logic for the mecanum drive
        */
        motorSpeed = 0.5;
        BackLeft.setPower((driveLeftStickY + driveLeftStickX - driveRightStickX) * motorSpeed);
        FrontLeft.setPower((driveLeftStickY - driveLeftStickX - driveRightStickX) * motorSpeed);
        BackRight.setPower((driveLeftStickY - driveLeftStickX + driveRightStickX) * motorSpeed);
        FrontRight.setPower((driveLeftStickY + driveLeftStickX + driveRightStickX) * motorSpeed);


        if (driveRightBumper == 1)
        {
            DroneLauncher.setPosition(0.3);
        }

    }



    @Override
    public void runOpMode()
    {

        /*
        Correlating declared variable names to established control hub setups
        */

        // PivotMotor = hardwareMap.get(DcMotor.class, "PivotMotor");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        SlideLeft = hardwareMap.get(DcMotor.class, "SlideLeft");
        SlideRight = hardwareMap.get(DcMotor.class, "SlideRight");
        PodLeft = hardwareMap.get(DcMotor.class, "PodLeft");
        PodBack = hardwareMap.get(DcMotor.class, "FrontLeft");
        PodRight = hardwareMap.get(DcMotor.class, "FrontRight");

        //Actuator = hardwareMap.get(DcMotor.class, "Actuator");

        DroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");
        ClawServo1 = hardwareMap.get(Servo.class, "ClawServo1");
        ClawServo2 = hardwareMap.get(Servo.class, "ClawServo2");
        PivotServo = hardwareMap.get(Servo.class, "PivotServo");
        RotateServo = hardwareMap.get(Servo.class, "RotateServo");


        armButtonYStatus = 0;

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

        //telemetry.addData("is active", opModeIsActive());
        //telemetry.addData("StickY", gamepad2.right_stick_y);


        waitForStart();


        //Actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //SlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //SlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //SlideLeft.setTargetPosition(0);
        //SlideRight.setTargetPosition(0);
        //SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        sleep(200);
        PivotServo.setPosition(servoPivotRotatePosition);
        sleep(500);
        RotateServo.setPosition(servoRotateTop);
        activeClaw = 1;
        sleep(500);

        ClawServo2.setPosition(servoClaw2Open);
        ClawServo1.setPosition(servoClaw1Open);
        PivotServo.setPosition(servoPivotGrabPosition);

        /*
        //Actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Actuator.setPower(1);
        //Actuator.setTargetPosition(-1000);
        //Actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        //Actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */

        while(opModeIsActive())
        {   //While initialized...
            //motorSpeed = 1 - (9 * gamepad1.left_trigger / 10); //Set motor speed
            //armSpeed = 0.35 - (0.35 * (3 * gamepad2.left_trigger / 5)); //Set arm speed
            //armSpeed = 0.5;
            mecanum(); //Call function to update drive inputs
            claw();
            slide();

            //telemetry.addData("Actuator", Actuator.getCurrentPosition());
            telemetry.addData("SlideLeft", SlideLeft.getCurrentPosition());
            telemetry.addData("SlideRight", SlideRight.getCurrentPosition());
            telemetry.addData("Tick", slideTickPosition);
            telemetry.addData("Hanging", isHanging);
            telemetry.addData("podleft", PodLeft.getCurrentPosition());
            telemetry.addData("podback", PodBack.getCurrentPosition());
            telemetry.addData("podright", PodRight.getCurrentPosition());



            telemetry.addData("Y status", armButtonYStatus);
            telemetry.addData("Pivot", PivotServo.getPosition());
            telemetry.addData("Rotate", RotateServo.getPosition());
            telemetry.addData("Claw1", ClawServo1.getPosition());
            telemetry.addData("Claw2", ClawServo2.getPosition());

            telemetry.update();
        }
    }
}
