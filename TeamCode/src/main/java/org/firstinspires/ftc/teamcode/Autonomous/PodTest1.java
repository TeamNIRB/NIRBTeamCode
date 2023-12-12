package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous

public class PodTest1 extends LinearOpMode {

    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor FrontRight;
    private Servo ClawServo1;
    private Servo ClawServo2;
    private Servo RotateServo;
    private Servo PivotServo;
    private DcMotor PodLeft;
    private DcMotor PodRight;
    private DcMotor PodBack;
    //private DcMotor PivotMotor;
    //private Servo ClawServo;
    //private ColorSensor ColorSensor;
    //private ElapsedTime runtime = new ElapsedTime();

    //bottom claw
    double servoClaw1Open = 0.7;
    double servoClaw1Closed = 0.4;
    int inch = 337;

    double servoClaw2Open = 0.85;
    double servoClaw2Closed = 0.55;

    double servoPivotPlacePosition = 0.4;
    double servoPivotRotatePosition = 0.4;
    double servoPivotGrabPosition = 0.56;

    //top is the side with tape
    double servoRotateTop = 0.3;
    double servoRotateBottom = 0.96;
    //0.3
    int PodLeftPos = 0;



    public void moveForward(int move)
    {
        BackLeft.setPower(0.5);
        FrontLeft.setPower(0.5);
        BackRight.setPower(0.5);
        FrontRight.setPower(0.5);

        if((PodLeft.getCurrentPosition() / inch) >= move) {
            BackLeft.setPower(0.0);
            FrontLeft.setPower(0.0);
            BackRight.setPower(0.0);
            FrontRight.setPower(0.0);
        }
    }
    public void moveBackward(int t)
    {
        BackLeft.setPower(-0.5);
        FrontLeft.setPower(-0.5);
        BackRight.setPower(-0.5);
        FrontRight.setPower(-0.5);

        sleep(t);

        BackLeft.setPower(0.0);
        FrontLeft.setPower(0.0);
        BackRight.setPower(0.0);
        FrontRight.setPower(0.0);
    }
    public void strafeLeft(int t)
    {
        BackLeft.setPower(0.5);
        FrontLeft.setPower(-0.5);
        BackRight.setPower(-0.5);
        FrontRight.setPower(0.5);

        sleep(t);

        BackLeft.setPower(0.0);
        FrontLeft.setPower(0.0);
        BackRight.setPower(0.0);
        FrontRight.setPower(0.0);
    }
    public void strafeRight(int t)
    {
        BackLeft.setPower(-0.5);
        FrontLeft.setPower(0.5);
        BackRight.setPower(0.5);
        FrontRight.setPower(-0.5);

        sleep(t);

        BackLeft.setPower(0.0);
        FrontLeft.setPower(0.0);
        BackRight.setPower(0.0);
        FrontRight.setPower(0.0);
    }
    public void autonomous() {

        moveForward(5);

    }

    @Override
    public void runOpMode() {
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        PodLeft = hardwareMap.get(DcMotor.class, "PodLeft");
        PodBack = hardwareMap.get(DcMotor.class, "FrontLeft");
        PodRight = hardwareMap.get(DcMotor.class, "FrontRight");

        ClawServo1 = hardwareMap.get(Servo.class, "ClawServo1");
        ClawServo2 = hardwareMap.get(Servo.class, "ClawServo2");
        PivotServo = hardwareMap.get(Servo.class, "PivotServo");
        RotateServo = hardwareMap.get(Servo.class, "RotateServo");

        //PivotMotor = hardwareMap.get(DcMotor.class, "PivotMotor");
        //ClawServo = hardwareMap.get(Servo.class, "ClawServo");
        //ColorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");

        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //PivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //PivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();
        if (opModeIsActive()) {
            autonomous();
        }
    }
}
