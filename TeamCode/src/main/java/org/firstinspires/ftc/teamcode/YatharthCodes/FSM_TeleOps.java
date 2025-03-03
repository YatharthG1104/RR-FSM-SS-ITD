package org.firstinspires.ftc.teamcode.YatharthCodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOps FSM YG")

public class FSM_TeleOps extends OpMode {
    // An Enum is used to represent lift states.

    public enum LiftState {
        Drive,
       // RetractAll,//retract all sorts of arm etc
        SubmersibleReady,//aka picking up a block with the slide
        SamplePicked,
        Transfer,
        FinishedBasket,
        SpecimenPicked,
        SpecimenHanged,
       // Idle,
        Stop
       // FINAL
    };

    private LiftState CurrentState;
    private boolean EmergencyStop;

   // LiftState liftState = LiftState.LIFT_START;

    //Initialize Non drive motors and servos
    DcMotor leftFront = null;
    DcMotor leftRear = null;
    DcMotor rightFront = null;
    DcMotor rightRear = null;
    DcMotor deliveryArmLeft = null;
    DcMotor deliveryArmRight = null;
    DcMotor FrontSlide = null;

    Servo Claw = null;
    Servo Wrist = null;
    Servo TwistLeft = null;
    Servo TwistRight = null;
    CRServo FrontWrist = null;
    Servo FrontClaw = null;
    Servo ElbowLeft = null;
    Servo ElbowRight = null;

    // Color Sensor Hardware Map
    ColorSensor sensorColor = null;

    ElapsedTime liftTimer = new ElapsedTime();

    //Define all Delivery Arm Encoder positions and power
    public int Delivery_Arm_Resting_Enc = 100;
    public int Delivery_Arm_HangReady_Enc = 1200;
    public int Delivery_Arm_BasketReady_Enc = 2600;
    public int Delivery_Arm_HangDone_Enc = 1000;
    public int Delivery_Arm_IntakeDone_Enc = 300;
    public int Delivery_Arm_Transfer_Enc = 450;
    public double Delivery_Arm_Extend_Power = 0.9;
    public double Delivery_Arm_Retract_Power = -0.9;

    //Define all Front Slide Arm Encoder positions and power
    public int Front_Slide_Resting_Enc = 0;
    public int Front_Slide_Intake_Enc = 450;
    public int Front_Slide_Transfer_Enc = 0;
    public double Front_Slide_Extend_Power = -0.9;
    public double Front_Slide_Retract_Power = 0.9;

    public double lfPower;
    public double rfPower;
    public double rbPower;
    public double lbPower;

    @Override
    public void init() {
        liftTimer.reset();
        CurrentState = LiftState.Drive;
        EmergencyStop = false;

      //  Pose2d InitialPose = new Pose2d(0,0,0);     // Beginning pose

        //Importing the hardware maps for all drive motors and setting the robot position
       // MecanumDrive drive = new MecanumDrive(hardwareMap, InitialPose);

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Define Hardware Map for all components
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setDirection(Servo.Direction.FORWARD);

        deliveryArmLeft = hardwareMap.get(DcMotor.class, "Delivery ArmL");
        deliveryArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        deliveryArmLeft.setDirection(DcMotor.Direction.REVERSE);          //Delivery arm left motor set reversed
        deliveryArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        deliveryArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders

        deliveryArmRight = hardwareMap.get(DcMotor.class, "Delivery ArmR");
        deliveryArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        deliveryArmRight.setDirection(DcMotor.Direction.REVERSE);          //Delivery arm right motor set reversed
        deliveryArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        deliveryArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders

        ElbowLeft = hardwareMap.get(Servo.class, "Elbow Left");
        //ElbowLeft.setDirection(Servo.Direction.REVERSE);

        ElbowRight = hardwareMap.get(Servo.class, "Elbow Right");
       // ElbowRight.setDirection(Servo.Direction.FORWARD);

        FrontSlide = hardwareMap.get(DcMotor.class, "Front Slide");
        FrontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//Front Slide zero power behavior
        FrontSlide.setDirection(DcMotor.Direction.FORWARD);          //Front Slide motor set forward
        FrontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Front Slide motor reset
        FrontSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Front Slide run using encoders

        FrontWrist = hardwareMap.get(CRServo.class, "Front Wrist");

        FrontClaw = hardwareMap.get(Servo.class, "Front Claw");
        FrontClaw.setDirection(Servo.Direction.FORWARD);

        TwistLeft = hardwareMap.get(Servo.class, "Twist Left");
        //TwistLeft.setDirection(Servo.Direction.REVERSE);

        TwistRight = hardwareMap.get(Servo.class, "Twist Right");
        TwistRight.setDirection(Servo.Direction.FORWARD);

        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Wrist.setDirection(Servo.Direction.FORWARD);

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        //in the init we are making sure evreything is reset

    }

    public void loop() {

        switch (CurrentState) {
            case Drive:
                //Driving
                move();
                //Regular Operations
                TeleOperations();
                //Other States
                if(gamepad1.right_bumper){
                    CurrentState = LiftState.SubmersibleReady;
                } else
                if(gamepad1.left_bumper){
                    CurrentState = LiftState.SamplePicked;
                } else
                if(gamepad1.b){
                    CurrentState =LiftState.Stop;
                }
            break;

            case SubmersibleReady:
                Actions.runBlocking(new SequentialAction(
                        new MotorAction2(FrontSlide, Front_Slide_Intake_Enc,Front_Slide_Extend_Power),
                        new DoubleServoAction(TwistLeft, TwistRight, 0.85, -0.85)
                ));
                CurrentState = LiftState.Drive;
            break;

            case SamplePicked:
                Actions.runBlocking(new SequentialAction(
                        new DoubleServoAction(TwistLeft, TwistRight, 0.5, -0.5),
                        new MotorAction2(FrontSlide, Front_Slide_Resting_Enc,Front_Slide_Retract_Power),
                        new DoubleServoAction(TwistLeft, TwistRight, 0.1, -0.1)
                ));
                CurrentState = LiftState.Drive;
                break;

            case Stop:
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                EmergencyStop = true;

                if(gamepad1.x) {
                    EmergencyStop = false;
                    CurrentState = LiftState.Drive;
                }

        }
    }


    public void move(){
        double y = -gamepad1. left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        lfPower = (y + x + rx) / denominator;
        lbPower = (y - x + rx) / denominator;
        rfPower = (y - x - rx) / denominator;
        rbPower = (y + x - rx) / denominator;

        leftFront.setPower(lfPower);
        leftRear.setPower(lbPower);
        rightFront.setPower(rfPower);
        rightRear.setPower(rbPower);
    }

    public void TeleOperations() {
        //Back Claw
        if(gamepad2.x){
            Claw.setPosition(-1);
        }
        if(gamepad2.b){
            Claw.setPosition(1);
        }
        //FrontSlide
        double FrontSlideStick = -gamepad2.right_stick_x;
        double FrontSlidePower = Range.clip(FrontSlideStick, -0.9, 0.9);
        FrontSlide.setPower(FrontSlidePower);
        //Back Elbow
        if(gamepad1.a){
            ElbowLeft.setPosition(-0.1);
            ElbowRight.setPosition(0.1);
        }
        if(gamepad2.dpad_up){
            ElbowLeft.setPosition(0.7);
            ElbowRight.setPosition(-0.7);
        }
        if(gamepad2.dpad_down){
            ElbowLeft.setPosition(-0.55);
            ElbowRight.setPosition(0.55);
        }
        //Front Twist
        if(gamepad2.left_bumper){
            TwistRight.setPosition(-0.1);
            TwistLeft.setPosition(0.1);
        }
        if(gamepad2.right_bumper){
            TwistRight.setPosition(-0.85);
            TwistLeft.setPosition(0.85);
        }
        // Front Claw
        if(gamepad2.y){
            FrontClaw.setPosition(1);
        }
        if(gamepad2.a){
            FrontClaw.setPosition(-1);
        }
        // Front Wrist
        if(gamepad2.right_stick_button) {
            FrontWrist.setPower(1);
        } else if(gamepad2.left_stick_button) {
            FrontWrist.setPower(-1);
        }
        else
            FrontWrist.setPower(0);
        //Delivery Arm
        double DeliveryArmStick = gamepad2.left_stick_y;
        double DeliveryArmPower = Range.clip(DeliveryArmStick, -0.9, 0.9);
        deliveryArmLeft.setPower(DeliveryArmPower);//0.5
        deliveryArmRight.setPower(-DeliveryArmPower);
    }

    // Servo action build for roadrunner to use
    public static class ServoAction implements Action {
        Servo servo;
        double position;
        ElapsedTime timer= null;

        public ServoAction(Servo srv, double pos) {
            this.servo= srv;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
                servo.setPosition(position);
            }

            return timer.seconds() < 0.5;
        }
    }

    //CRServo Action build for roadrunner to use
    public static class CRServoAction implements Action {
        CRServo crServo;
        double power;
        ElapsedTime timer= null;

        public CRServoAction(CRServo crsrv, double pow ) {
            this.crServo = crsrv;
            // this.seconds = sec;
            this.power = pow;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                crServo.setPower(power);
            }
            return timer.seconds() < 2;
        }
    }

    // Double Motor Action build for roadrunner to use
    public static class DoubleMotorAction implements Action {
        DcMotor motor1;
        DcMotor motor2;
        double position_tgt1;
        double position_tgt2;
        double power1;
        double power2;
        ElapsedTime timer = null;

        public DoubleMotorAction (DcMotor mot1, DcMotor mot2, double pos1, double pos2, double pow1, double pow2) {
            this.motor1 = mot1;
            this.position_tgt1 = pos1;
            this.power1 = pow1;
            this.motor2 = mot2;
            this.position_tgt2 = pos2;
            this.power2 = pow2;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (motor1.getCurrentPosition() < position_tgt1) {
                timer = new ElapsedTime();
                motor1.setPower(power1);
                motor2.setPower(power2);
                motor1.setTargetPosition((int) (position_tgt1));
                // motor2.setTargetPosition((int) (position_tgt2));
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            return timer.seconds() < 1;
        }
    }

    // Double Servo action build for roadrunner to use
    public static class DoubleServoAction implements Action {
        Servo servo1;
        double position1;
        Servo servo2;
        double position2;
        ElapsedTime timer= null;

        public DoubleServoAction(Servo srv1, Servo srv2,double pos1,double pos2) {
            this.servo1 = srv1;
            this.servo2 = srv2;
            this.position1 = pos1;
            this.position2 = pos2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
                servo1.setPosition(position1);
                servo2.setPosition(position2);
            }

            return timer.seconds() < 0.5;
        }
    }

    // Motor Action-2 build for roadrunner to use
    public static class MotorAction2 implements Action {
        DcMotor motor;
        double position_tgt;
        double power;

        public MotorAction2(DcMotor mot, double pos, double pow) {
            this.motor = mot;
            this.position_tgt = pos;
            this.power = pow;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (motor.getCurrentPosition() != position_tgt) {
                motor.setPower(power);
                motor.setTargetPosition((int) (position_tgt));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return true;
            } else {
                motor.setPower(0);
                return false;
            }
        }
    }
}