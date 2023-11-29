//NIRB Team main drive program -- Mecanum Drive; Pivot Arm; Vertical Griper

//Imports

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    private DcMotor Actuator;
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

    float armRightBumper;
    float armLeftBumper;
    float slidePosition;
    float armLeftStickY;

    double activeClaw = 1;
    double motorSpeed; //Controlls the speed of the motors throughout the program
    double armSpeed; //Controlls the speed of the arm throughout the program
    double actuatorSpeed;
    double armButtonA;
    double armButtonB;
    double armButtonX;
    double armButtonY;
    double armButtonYStatus = 0; //determines if button y was pressed
    double isHanging = 0;

    double slideStatus = 1; //1=drive 2=place 3=hang
    int slideTickPosition = 0;
    int slideLeftPos = 0;
    int slideRightPos = 0;

    public void mecanum() { //Function resposible for controlling drive motors

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
        motorSpeed = 0.7;
        BackLeft.setPower((driveLeftStickY + driveLeftStickX - driveRightStickX) * motorSpeed);
        FrontLeft.setPower((driveLeftStickY - driveLeftStickX - driveRightStickX) * motorSpeed);
        BackRight.setPower((driveLeftStickY - driveLeftStickX + driveRightStickX) * motorSpeed);
        FrontRight.setPower((driveLeftStickY + driveLeftStickX + driveRightStickX) * motorSpeed);



        if(armRightStickY < 0)
        {
            SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armSpeed = 0.5;
            SlideLeft.setPower(armRightStickY * armSpeed); //Moves arm according to the stick (should use encoder)
            SlideRight.setPower(armRightStickY * armSpeed);
        }
        else if(armRightStickY > 0)
        {

            SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armSpeed = 0.8;

            SlideLeft.setPower(armRightStickY * armSpeed); //Moves arm according to the stick (should use encoder)
            SlideRight.setPower(armRightStickY * armSpeed);
        }
        else
        {

            SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SlideLeft.setPower(-0.05); //Moves arm according to the stick (should use encoder)
            SlideRight.setPower(-0.05);
        }


        actuatorSpeed = 1;
        //Actuator.setPower(0.5);
        //Actuator.setTargetPosition(1000);
        //Actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(armLeftStickY != 0)
        {
            //Actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //Actuator.setPower(armLeftStickY * actuatorSpeed);
        }
        //telemetry.addData("StickY", armRightStickY);
        //telemetry.addData("armSpeed", armSpeed);
        //telemetry.update();


        if (armRightBumper == 1) {
            //FrontLeft.setPower (5);
        }
        if (armLeftBumper == 1) {
            //FrontLeft.setPower (-5);
        }
        /*
        if (activeClaw == 1)
        {
           if (armLeftBumper == 1)
           {
               RotateServo.setPosition(0.295);
           }
           if (armRightBumper == 1)
           {
               RotateServo.setPosition(0.96);
           }
        }
        */

        if (activeClaw == 1 || armButtonB == 1)
        {
            if (armLeftBumper == 1)
            {
                ClawServo1.setPosition(0.6);
            }
            if (armRightBumper == 1)
            {
                ClawServo1.setPosition(0.1);//bottom claw
            }
        }
        if (activeClaw == 2 || armButtonB == 1)
        {
            if (armLeftBumper == 1)
            {
                ClawServo2.setPosition(1);//top claw
            }
            if (armRightBumper == 1)
            {
                ClawServo2.setPosition(0.5);
            }
        }


        if (activeClaw == 1)
        {
            if (armLeftBumper == 1)
            {
                //PivotServo.setPosition(0.7); //placing position
                //PivotServo.setPosition(0.4); //flip position
            }
            if (armRightBumper == 1)
            {
                //PivotServo.setPosition(0.21);//grab position
            }
        }
        if (armButtonYStatus == 0)
        {
            if (armButtonY == 1)
            {
                armButtonYStatus = 1;
                PivotServo.setPosition(0.4); //flip position
                sleep(200);
                if (activeClaw == 1)
                {
                    RotateServo.setPosition(0.295);
                    activeClaw = 2;
                    sleep(500);
                    if(slideStatus == 1)
                    {
                        PivotServo.setPosition(0.21);
                    }
                    else
                    {
                        PivotServo.setPosition(0.65);
                    }

                }
                else
                {
                    RotateServo.setPosition(0.96);
                    activeClaw = 1;
                    sleep(500);
                    if(slideStatus == 1)
                    {
                        PivotServo.setPosition(0.21);
                    }
                    else
                    {
                        PivotServo.setPosition(0.65);
                    }
                }
            }
            else
            {
                //armButtonYStatus = 0;
            }
        }
        else
        {
            if (armButtonY == 0)
            {
                armButtonYStatus = 0;
            }
        }

       /* if (armLeftBumper == 1) {
            LeftClawServo.setPosition(0.65);
            RightClawServo.setPosition(0.55);
        }
        if (armRightBumper == 1) {
            LeftClawServo.setPosition(0.85);
            RightClawServo.setPosition(0.35);
        }*/

        if (driveButtonB == 1)
        {
            PivotServo.setPosition(0.21);

            Actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Actuator.setPower(1);
            Actuator.setTargetPosition(0);
            Actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideTickPosition = 0;
            slideStatus = 1;
        }
        else if (driveButtonY == 1)
        {
            PivotServo.setPosition(0.65);

            Actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Actuator.setPower(1);
            Actuator.setTargetPosition(11250);
            Actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideTickPosition = 1500;

            slideStatus = 2;
        }
        else if (driveButtonX == 1)
        {
            if (slideStatus != 3 )
            {
                PivotServo.setPosition(0.4);

                Actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Actuator.setPower(1);
                Actuator.setTargetPosition(7500);
                Actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slideTickPosition = 1500;
                sleep(500);
                slideStatus = 3;
            }

        }
        else if (driveButtonA == 1)
        {
            slideTickPosition = 300;

            Actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Actuator.setPower(0.2);
            Actuator.setTargetPosition(5000);
            Actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            slideStatus = 4;
        }


        //SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(slideStatus == 1 && Actuator.getCurrentPosition() < 2500)
        {
            if (SlideLeft.getCurrentPosition() > slideTickPosition + 10)
            {
                SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideLeft.setPower(0.5);
                SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideRight.setPower(0.5);
            }
            else
            {
                SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        else if (slideStatus == 2 || slideStatus == 3)
        {
            if (SlideLeft.getCurrentPosition() < slideTickPosition)
            {
                SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideLeft.setPower(-0.8);
                SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideRight.setPower(-0.8);


            }
            else if (SlideLeft.getCurrentPosition() > slideTickPosition + 50)
            {
                SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideLeft.setPower(0.8);
                SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideRight.setPower(0.8);
            }
        }
        else if (slideStatus == 4)
        {
            if (SlideLeft.getCurrentPosition() > slideTickPosition)
            {
                SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideLeft.setPower(0.8);
                SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SlideRight.setPower(0.8);
            }
            else
            {

                isHanging = 1;
                slideLeftPos = SlideLeft.getCurrentPosition();
                slideRightPos = SlideRight.getCurrentPosition();

            }

            if (isHanging == 1)
            {
                SlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                SlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                SlideLeft.setTargetPosition(slideLeftPos);
                SlideRight.setTargetPosition(slideRightPos);
                SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);            }
        }

        if (driveRightBumper == 1)
        {
            DroneLauncher.setPosition(0.3);
        }

    }



    @Override
    public void runOpMode() {

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
        Actuator = hardwareMap.get(DcMotor.class, "Actuator");

        DroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");
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
        SlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        SlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Actuator.setDirection(DcMotorSimple.Direction.FORWARD);

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


        Actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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


        PivotServo.setPosition(0.4); //flip position
        sleep(200);
        RotateServo.setPosition(0.96);
        activeClaw = 1;
        sleep(500);
        PivotServo.setPosition(0.21);
        ClawServo2.setPosition(1);
        ClawServo1.setPosition(0.6);
        /*
        Actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Actuator.setPower(1);
        Actuator.setTargetPosition(-1000);
        Actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        Actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */

        while(opModeIsActive()) { //While initialized...
            //motorSpeed = 1 - (9 * gamepad1.left_trigger / 10); //Set motor speed
            //armSpeed = 0.35 - (0.35 * (3 * gamepad2.left_trigger / 5)); //Set arm speed
            //armSpeed = 0.5;
            mecanum(); //Call function to update drive inputs

            telemetry.addData("Actuator", Actuator.getCurrentPosition());
            telemetry.addData("SlideLeft", SlideLeft.getCurrentPosition());
            telemetry.addData("SlideRight", SlideRight.getCurrentPosition());
            telemetry.addData("Tick", slideTickPosition);

            telemetry.update();

        }
    }
}

