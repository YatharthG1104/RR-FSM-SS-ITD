package org.firstinspires.ftc.teamcode.YatharthCodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Basket Auto")
public class RR_Basket_Auto extends LinearOpMode {

    // Time into the Autonomous round
    public ElapsedTime mRuntime = new ElapsedTime();

    //Initialize Non drive motors and servos
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

    //Define all Claw positions
    public static double Claw_Open_Pos = -0.9;
    public static double Claw_Close_Pos = 0.9;
    public static double Claw_Initial_Pos = 0.7;

    //Define all Delivery Arm Encoder positions and power
    public static int Delivery_Arm_Resting_Enc = 100;
    public static int Delivery_Arm_HangReady_Enc = 1200;
    public static int Delivery_Arm_BasketReady_Enc = 2000;
    public static int Delivery_Arm_HangDone_Enc = 1000;
    public static int Delivery_Arm_IntakeDone_Enc = 300;
    public static int Delivery_Arm_Transfer_Enc = 450;
    public static double Delivery_Arm_Extend_Power = 0.0;// Change to 0.9
    public static double Delivery_Arm_Retract_Power = 0.0;// Change to -0.9

    //Define all Elbow positions
    public static double ElbowL_Intake_Pos = 0.0;
    public static double ElbowR_Intake_Pos = 0.0;
    public static double ElbowL_Transfer_Pos = 0.63;
    public static double ElbowR_Transfer_Pos = 0.63;
    public static double ElbowL_Level1_Pos = 0.15;
    public static double ElbowR_Level1_Pos = 0.15;
    public static double ElbowL_Hang_Pos = 0.5;
    public static double ElbowR_Hang_Pos = 0.5;
    public static double ElbowL_Basket_Pos = 0.15;// Change to 0.1
    public static double ElbowR_Basket_Pos = 0.15;// Change to 0.1

    //Define all Front Slide Arm Encoder positions and power
    public static int Front_Slide_Resting_Enc = 0;
    public static int Front_Slide_Intake_Enc = 450;
    public static int Front_Slide_Transfer_Enc = -70;
    public static int Front_Slide_Hold_Enc = 0;
    public static double Front_Slide_Extend_Power = 0.9;
    public static double Front_Slide_Retract_Power = -0.9;

    //Define all Twist positions
    public static double TwistL_Intake_Pos = 0.95;
    public static double TwistR_Intake_Pos = 0.95;
    public static double TwistL_Transfer_Pos = -0.85;
    public static double TwistR_Transfer_Pos = -0.85;
    public static double TwistL_IntakeReady_Pos = 0.5;
    public static double TwistR_IntakeReady_Pos = 0.5;
    public static double TwistL_Rest_Pos = 0.0;
    public static double TwistR_Rest_Pos = 0.0;

    //Define all Front Claw Positions
    public static double FrontClaw_Open_Pos = 0.0;
    public static double FrontClaw_Close_Pos = 0.9;

    // Wait Variable
    public static double Wait = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d InitialPose = new Pose2d(0,0,0);     // Beginning pose

        //Importing the hardware maps for all drive motors and setting the robot position
        MecanumDrive drive = new MecanumDrive(hardwareMap, InitialPose);

        //Define Hardware Map for all components
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.scaleRange(0,1);
        Claw.setDirection(Servo.Direction.FORWARD);

        deliveryArmLeft = hardwareMap.get(DcMotor.class, "Delivery ArmL");
        deliveryArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        deliveryArmLeft.setDirection(DcMotor.Direction.FORWARD);          //Delivery arm left motor set reversed
        deliveryArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        deliveryArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders

        deliveryArmRight = hardwareMap.get(DcMotor.class, "Delivery ArmR");
        deliveryArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        deliveryArmRight.setDirection(DcMotor.Direction.REVERSE);          //Delivery arm right motor set reversed
        deliveryArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        deliveryArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders

        ElbowLeft = hardwareMap.get(Servo.class, "Elbow Left");
        ElbowLeft.scaleRange(-1,1);
        ElbowLeft.setDirection(Servo.Direction.REVERSE);


        ElbowRight = hardwareMap.get(Servo.class, "Elbow Right");
        ElbowRight.scaleRange(-1,1);
        ElbowRight.setDirection(Servo.Direction.FORWARD);


        FrontSlide = hardwareMap.get(DcMotor.class, "Front Slide");
        FrontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//Front Slide zero power behavior
        FrontSlide.setDirection(DcMotor.Direction.REVERSE);          //Front Slide motor set forward
        FrontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Front Slide motor reset
        FrontSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Front Slide run using encoders

        FrontWrist = hardwareMap.get(CRServo.class, "Front Wrist");
        //  FrontWrist.setDirection(Servo.Direction.FORWARD);

        FrontClaw = hardwareMap.get(Servo.class, "Front Claw");
        FrontClaw.setDirection(Servo.Direction.FORWARD);

        TwistLeft = hardwareMap.get(Servo.class, "Twist Left");
        TwistLeft.scaleRange(-1,1);
        TwistLeft.setDirection(Servo.Direction.REVERSE);

        TwistRight = hardwareMap.get(Servo.class, "Twist Right");
        TwistRight.scaleRange(-1,1);
        TwistRight.setDirection(Servo.Direction.FORWARD);

        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Wrist.scaleRange(-1,1);
        Wrist.setDirection(Servo.Direction.FORWARD);

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        //set  start game clock
        mRuntime.reset();// Zero game clock

        waitForStart();

        if (isStopRequested()) return;

        // Pre-loaded specimen drop in upper basket
        Actions.runBlocking(
               new SequentialAction(
                       new ServoAction(Claw,Claw_Close_Pos),
                       new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_Hang_Pos,ElbowR_Hang_Pos),
                       new ParallelAction(
                              new MotorAction2(FrontSlide, Front_Slide_Hold_Enc, Front_Slide_Retract_Power),
                              drive.actionBuilder(new Pose2d(0,0,0))
                                      .strafeToLinearHeading(new Vector2d(6,24), Math.toRadians(-50))
                                      .build(),
                            //  new DoubleMotorAction(deliveryArmLeft,deliveryArmRight,Delivery_Arm_BasketReady_Enc,Delivery_Arm_BasketReady_Enc,Delivery_Arm_Extend_Power,Delivery_Arm_Extend_Power),
                              new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_Basket_Pos,ElbowR_Basket_Pos)
                      ),
                       new SleepAction(Wait),
                       new ServoAction(Claw, Claw_Open_Pos),

    //Second sample pick up and drop in upper basket
                       new ParallelAction(
                               drive.actionBuilder(new Pose2d(5,24,Math.toRadians(-50)))
                                       .strafeToLinearHeading(new Vector2d(18,23), Math.toRadians(-10))
                                       .build(),
                               new MotorAction2(FrontSlide, Front_Slide_Intake_Enc, Front_Slide_Extend_Power),
                               new ServoAction(FrontClaw, FrontClaw_Open_Pos),
                               new DoubleServoAction(TwistLeft,TwistRight, TwistL_IntakeReady_Pos, TwistR_IntakeReady_Pos),
                              // new ParallelAction(
                              //        new MotorAction2(deliveryArmLeft, Delivery_Arm_Transfer_Enc, Delivery_Arm_Retract_Power),
                              //         new MotorAction2(deliveryArmRight, Delivery_Arm_Transfer_Enc, Delivery_Arm_Retract_Power)
                             //  ),
                               new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_Transfer_Pos,ElbowR_Transfer_Pos)
                       ),
                       new DoubleServoAction(TwistLeft,TwistRight, TwistL_Intake_Pos, TwistR_Intake_Pos),
                       new ServoAction(FrontClaw, FrontClaw_Close_Pos),
                               new ParallelAction(
                                       new DoubleServoAction(TwistLeft,TwistRight, TwistL_IntakeReady_Pos, TwistR_IntakeReady_Pos),
                                       new MotorAction2(FrontSlide, Front_Slide_Transfer_Enc, Front_Slide_Retract_Power)
                               ),
                       new DoubleServoAction(TwistLeft,TwistRight, TwistL_Transfer_Pos, TwistR_Transfer_Pos),
                       new ServoAction(FrontClaw, FrontClaw_Open_Pos),
                       new SleepAction(Wait),
                       new ServoAction(Claw, Claw_Close_Pos),
                       new ParallelAction(
                               drive.actionBuilder(new Pose2d(18,23,Math.toRadians(-10)))
                                       .strafeToLinearHeading(new Vector2d(4,24), Math.toRadians(-50))
                                       .build(),
                             //  new DoubleMotorAction(deliveryArmLeft,deliveryArmRight,Delivery_Arm_BasketReady_Enc,Delivery_Arm_BasketReady_Enc,Delivery_Arm_Extend_Power,Delivery_Arm_Extend_Power),
                               new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_Basket_Pos,ElbowR_Basket_Pos)
                       ),
                       new SleepAction(Wait),
                       new ServoAction(Claw, Claw_Open_Pos),
                       // Second Transfer
                       new ParallelAction(
                               drive.actionBuilder(new Pose2d(4,24,Math.toRadians(-50)))
                                       .strafeToLinearHeading(new Vector2d(17,25), Math.toRadians(10))
                                       .build(),
                               new MotorAction2(FrontSlide, Front_Slide_Intake_Enc, Front_Slide_Extend_Power),
                               new ServoAction(FrontClaw, FrontClaw_Open_Pos),
                               new DoubleServoAction(TwistLeft,TwistRight, TwistL_IntakeReady_Pos, TwistR_IntakeReady_Pos),
                               // new ParallelAction(
                               //         new MotorAction2(deliveryArmLeft, Delivery_Arm_Transfer_Enc, Delivery_Arm_Retract_Power),
                               //         new MotorAction2(deliveryArmRight, Delivery_Arm_Transfer_Enc, Delivery_Arm_Retract_Power)
                               //  ),
                               new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_Transfer_Pos,ElbowR_Transfer_Pos)
                       ),
                       new DoubleServoAction(TwistLeft,TwistRight, TwistL_Intake_Pos, TwistR_Intake_Pos),
                       new ServoAction(FrontClaw, FrontClaw_Close_Pos),
                       new ParallelAction(
                               new DoubleServoAction(TwistLeft,TwistRight, TwistL_IntakeReady_Pos, TwistR_IntakeReady_Pos),
                               new MotorAction2(FrontSlide, Front_Slide_Transfer_Enc, Front_Slide_Retract_Power)
                       ),
                       new DoubleServoAction(TwistLeft,TwistRight, TwistL_Transfer_Pos, TwistR_Transfer_Pos),
                       new ServoAction(FrontClaw, FrontClaw_Open_Pos),
                       new SleepAction(Wait),
                       new ServoAction(Claw, Claw_Close_Pos),
                       new ParallelAction(
                               drive.actionBuilder(new Pose2d(17,25,Math.toRadians(10)))
                                       .strafeToLinearHeading(new Vector2d(4,24), Math.toRadians(-50))
                                       .build(),
                               //  new DoubleMotorAction(deliveryArmLeft,deliveryArmRight,Delivery_Arm_BasketReady_Enc,Delivery_Arm_BasketReady_Enc,Delivery_Arm_Extend_Power,Delivery_Arm_Extend_Power),
                               new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_Basket_Pos,ElbowR_Basket_Pos)
                       ),
                       new SleepAction(Wait),
                       new ServoAction(Claw, Claw_Open_Pos),
                       // Third Transfer
                /*       new ParallelAction(
                               drive.actionBuilder(new Pose2d(6,24,Math.toRadians(-50))) // Another way of running a trajectory (not recommended because trajectories take time to build and will slow down your code, always try to build them beforehand)
                                       .strafeTo(new Vector2d(25,25))
                                       .turnTo(Math.toRadians(90))
                                       .build(),
                               new MotorAction2(FrontSlide, Front_Slide_Intake_Enc, Front_Slide_Extend_Power),
                               new ServoAction(FrontClaw, FrontClaw_Open_Pos),
                               new DoubleServoAction(TwistLeft,TwistRight, TwistL_IntakeReady_Pos, TwistR_IntakeReady_Pos),
                               // new ParallelAction(
                               //         new MotorAction2(deliveryArmLeft, Delivery_Arm_Transfer_Enc, Delivery_Arm_Retract_Power),
                               //         new MotorAction2(deliveryArmRight, Delivery_Arm_Transfer_Enc, Delivery_Arm_Retract_Power)
                               //  ),
                               new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_Transfer_Pos,ElbowR_Transfer_Pos)
                       ),
                       new DoubleServoAction(TwistLeft,TwistRight, TwistL_Intake_Pos, TwistR_Intake_Pos),
                       new ServoAction(FrontClaw, FrontClaw_Close_Pos),
                       new ParallelAction(
                               new DoubleServoAction(TwistLeft,TwistRight, TwistL_IntakeReady_Pos, TwistR_IntakeReady_Pos),
                               new MotorAction2(FrontSlide, Front_Slide_Transfer_Enc, Front_Slide_Retract_Power)
                       ),
                       new DoubleServoAction(TwistLeft,TwistRight, TwistL_Transfer_Pos, TwistR_Transfer_Pos),
                       new ServoAction(FrontClaw, FrontClaw_Open_Pos),
                       new SleepAction(Wait),
                       new ServoAction(Claw, Claw_Close_Pos),
                       new ParallelAction(
                               drive.actionBuilder(new Pose2d(25,25,Math.toRadians(90))) // Another way of running a trajectory (not recommended because trajectories take time to build and will slow down your code, always try to build them beforehand)
                                       .strafeTo(new Vector2d(6,24))// Change 6 to 8
                                       .turnTo(Math.toRadians(-50))
                                       .build(),
                               //  new DoubleMotorAction(deliveryArmLeft,deliveryArmRight,Delivery_Arm_BasketReady_Enc,Delivery_Arm_BasketReady_Enc,Delivery_Arm_Extend_Power,Delivery_Arm_Extend_Power),
                               new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_Basket_Pos,ElbowR_Basket_Pos)
                       ),
                       new SleepAction(Wait),
                       new ServoAction(Claw, Claw_Open_Pos),*/
                       new ParallelAction(
                               drive.actionBuilder(new Pose2d(6,24,Math.toRadians(-50)))
                                       .strafeToLinearHeading(new Vector2d(52,5), Math.toRadians(90))
                                       .strafeTo(new Vector2d(53,-7))
                                       .build(),
                               new DoubleServoAction(ElbowLeft,ElbowRight, ElbowL_Level1_Pos,ElbowR_Level1_Pos)
                       )

               )
        );
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

    // Motor Action build for roadrunner to use
    public static class MotorAction implements Action {
        DcMotor motor;
        double position_tgt;
        double power;
        ElapsedTime timer = null;

        public MotorAction (DcMotor mot, double pos, double pow) {
            this.motor = mot;
            this.position_tgt = pos;
            this.power = pow;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (motor.getCurrentPosition() < position_tgt) {
                timer = new ElapsedTime();
                motor.setPower(power);
                motor.setTargetPosition((int) (position_tgt));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            return timer.seconds() < 1;
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
