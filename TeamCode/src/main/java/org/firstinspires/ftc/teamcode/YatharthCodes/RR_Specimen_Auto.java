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

@Autonomous(name = "Specimen Auto")
public class RR_Specimen_Auto extends LinearOpMode {
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

    //Define all Wrist positions and mode
    public static double Wrist_Intake_Pos = 0.25;
    public static double Wrist_Hang_Pos = 0.9;
    public static double Wrist_Rest_Pos = 0.25;

    //Define all Claw positions
    public static double Claw_Open_Pos = 1.0;
    public static double Claw_Close_Pos = 0.4;

    public int Delivery_Arm_Resting_Enc = 0;
    public int Delivery_Arm_HangReady_Enc = 1350;
    public int Delivery_Arm_HangDone_Enc = 1350;
    public int Delivery_Arm_IntakeDone_Enc = 300;
    public double Delivery_Arm_Extend_Power = 0.9;
    public double Delivery_Arm_Retract_Power = -0.9;
    public static int Delivery_Arm_HangIntake_Enc = 100;

    //Define all Elbow positions
    public static double ElbowL_Intake_Pos = 0.23;
    public static double ElbowR_Intake_Pos = 0.23;
    public static double ElbowL_Hang_Pos = 0.45;
    public static double ElbowR_Hang_Pos = 0.45;
    public static double ElbowL_HangDone_Pos = 0.65;
    public static double ElbowR_HangDone_Pos = 0.65;
    public static double ElbowL_Rest_Pos = 0.5;
    public static double ElbowR_Rest_Pos = 0.5;

    //Define all Twist positions
    public static double TwistL_Intake_Pos = -0.85;
    public static double TwistR_Intake_Pos = 0.85;
    public static double TwistL_Transfer_Pos = 0.8;
    public static double TwistR_Transfer_Pos = -0.8;
    public static double TwistL_IntakeReady_Pos = 0.65;
    public static double TwistR_IntakeReady_Pos = -0.65;
    public static double TwistL_Rest_Pos = 0.0;
    public static double TwistR_Rest_Pos = 0.0;

    //Define all Front Slide Arm Encoder positions and power
    public int Front_Slide_Resting_Enc = -50;
    public double Front_Slide_Retract_Power = -0.45;

    // Wait Variable
    public static double Wait = 0.5;
    public static double servo_stepsize = 0.01;
    public static double action_wait = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d InitialPose = new Pose2d(0,0,0);     // Beginning pose

        //Importing the hardware maps for all drive motors and setting the robot position
        MecanumDrive drive = new MecanumDrive(hardwareMap, InitialPose);

        //Define Hardware Map for all components
        Claw = hardwareMap.get(Servo.class, "Claw");
      //  Claw.scaleRange(0,1);
        Claw.setDirection(Servo.Direction.REVERSE);

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

      //  sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        //set  start game clock
        mRuntime.reset();// Zero game clock

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
            new SequentialAction(
                    //First specimen hang
                    new ServoAction(Claw, Claw_Close_Pos),
                    new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_Hang_Pos,ElbowR_Hang_Pos),
                    new SleepAction(0.3),
                    new ParallelAction(
                            drive.actionBuilder(new Pose2d(0,0,0))
                                    .lineToX(30)
                                    .build(),
                            new DoubleMotorAction(deliveryArmLeft,deliveryArmRight,Delivery_Arm_HangReady_Enc,Delivery_Arm_HangReady_Enc,Delivery_Arm_Extend_Power,Delivery_Arm_Extend_Power),
                            new MotorAction2(FrontSlide, Front_Slide_Resting_Enc, Front_Slide_Retract_Power)
                    ),
                    new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_HangDone_Pos,ElbowR_HangDone_Pos),
                    new ServoAction(Claw, Claw_Open_Pos),

                    //Drag one sample
                    new ParallelAction(
                        drive.actionBuilder(new Pose2d(30,0,0))
                                .setReversed(true)
                                .setTangent(Math.toRadians(0))
                                .lineToX(10)
                                .setTangent(-Math.PI/2)
                                .splineToConstantHeading(new Vector2d(53,-33), Math.PI/2)
                                .strafeTo(new Vector2d(54,-40))
                                .setReversed(true)
                                .setTangent(0)
                                .lineToX(12)
                                .build(),
                        new ServoAction(Wrist,Wrist_Intake_Pos),
                        new MotorAction2(deliveryArmLeft,Delivery_Arm_HangIntake_Enc,Delivery_Arm_Retract_Power),
                        new MotorAction2(deliveryArmRight,Delivery_Arm_HangIntake_Enc,Delivery_Arm_Retract_Power)
                    ),

                    //Intake and hang second specimen
                    new SequentialAction(
                    new DoubleServoAction(ElbowLeft,ElbowRight, ElbowL_Intake_Pos,ElbowR_Intake_Pos),
                    drive.actionBuilder(new Pose2d(12,-40,0))
                                    .waitSeconds(0.2)
                                    .setReversed(true)
                                    .setTangent(0)
                                    .lineToX(7)
                                    .build(),
                    new ServoAction(Claw, Claw_Close_Pos),
                    new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_Hang_Pos,ElbowR_Hang_Pos),
                    new DoubleMotorAction(deliveryArmLeft,deliveryArmRight,Delivery_Arm_IntakeDone_Enc,Delivery_Arm_IntakeDone_Enc, Delivery_Arm_Extend_Power,Delivery_Arm_Extend_Power),
                    new DoubleServoAction(TwistLeft,TwistRight, TwistL_IntakeReady_Pos, TwistR_IntakeReady_Pos),
                            new ParallelAction(
                                    drive.actionBuilder(new Pose2d(7,-40,0))
                                            .strafeToLinearHeading(new Vector2d(31, 15),0)
                                            .build(),
                                    new MotorAction2(FrontSlide, Front_Slide_Resting_Enc, Front_Slide_Retract_Power),
                                    new DoubleMotorAction(deliveryArmLeft,deliveryArmRight,Delivery_Arm_HangReady_Enc,Delivery_Arm_HangReady_Enc,Delivery_Arm_Extend_Power,Delivery_Arm_Extend_Power),
                                    new ServoAction(Wrist, Wrist_Hang_Pos)
                            ),
                    new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_HangDone_Pos,ElbowR_HangDone_Pos),
                    new ServoAction(Claw, Claw_Open_Pos),

        //Park and reset back slide
                            new ParallelAction(
                                    drive.actionBuilder(new Pose2d(30,15,0))
                                            .strafeTo(new Vector2d(7,-35))
                                            .build(),
                                    new MotorAction2(deliveryArmLeft,Delivery_Arm_Resting_Enc,Delivery_Arm_Retract_Power),
                                    new MotorAction2(deliveryArmRight,Delivery_Arm_Resting_Enc,Delivery_Arm_Retract_Power),
                                    new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_Intake_Pos,ElbowR_Intake_Pos)
                            )
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

            return timer.seconds() < action_wait;
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
            return timer.seconds() < action_wait;
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

            return timer.seconds() < action_wait;
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

    // Servo stepwise decrease action build for roadrunner to use
    public static class DecStepServoAction implements Action {
        Servo servo;
        double endposition;
        double startposition;
        double stepcount;
        double stepsize;
        ElapsedTime timer= null;

        public DecStepServoAction(Servo srv, double pos) {
            this.servo= srv;
            this.endposition = pos;
            this.startposition = srv.getPosition();
            this.stepsize = servo_stepsize;
            this.stepcount = (this.startposition - this.endposition)/this.stepsize;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
           for (int i= 0; i<= stepcount; i++) {
               double CurrentPosition = startposition - (stepsize*i);
               servo.setPosition(CurrentPosition);
               return true;
           }
           return false;
        }
    }

    // Servo stepwise decrease action build for roadrunner to use
    public static class IncStepServoAction implements Action {
        Servo servo;
        double endposition;
        double startposition;
        double stepcount;
        double stepsize;
        ElapsedTime timer= null;

        public IncStepServoAction(Servo srv, double pos) {
            this.servo= srv;
            this.endposition = pos;
            this.startposition = srv.getPosition();
            this.stepsize = servo_stepsize;
            this.stepcount = (-this.startposition + this.endposition)/this.stepsize;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (int i= 0; i<= stepcount; i++) {
                double CurrentPosition = startposition + (stepsize*i);
                servo.setPosition(CurrentPosition);
                return true;
            }
            return false;
        }
    }

}



/*  .strafeTo(new Vector2d(55,-42))
                                .strafeTo(new Vector2d(55,-50))
                                .setReversed(true)
                                .setTangent(0)
                                .lineToX(8)*/
//   .strafeToLinearHeading(new Vector2d(10,-30), 0)
// .strafeTo(new Vector2d(7,-30))

       /*

           //       new ServoAction(Wrist,Wrist_Intake_Pos),
                   //     new ParallelAction(
                     //           new MotorAction2(deliveryArmLeft,Delivery_Arm_HangIntake_Enc,Delivery_Arm_Retract_Power),
                       //         new MotorAction2(deliveryArmRight,Delivery_Arm_HangIntake_Enc,Delivery_Arm_Retract_Power)
                        //)
                ),
              /*  new DoubleServoAction(ElbowLeft,ElbowRight, ElbowL_Intake_Pos,ElbowR_Intake_Pos),
                new ServoAction(Claw, Claw_Close_Pos),
                new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_Hang_Pos,ElbowR_Hang_Pos),
                new DoubleMotorAction(deliveryArmLeft,deliveryArmRight,Delivery_Arm_IntakeDone_Enc,Delivery_Arm_IntakeDone_Enc, Delivery_Arm_Extend_Power,Delivery_Arm_Extend_Power),
                new DoubleServoAction(TwistLeft,TwistRight, TwistL_IntakeReady_Pos, TwistR_IntakeReady_Pos),
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(7,-35,0))
                                .strafeToLinearHeading(new Vector2d(30, 13),0)
                                .build(),
                        new MotorAction2(FrontSlide, Front_Slide_Resting_Enc, Front_Slide_Retract_Power),
                        new DoubleMotorAction(deliveryArmLeft,deliveryArmRight,Delivery_Arm_HangReady_Enc,Delivery_Arm_HangReady_Enc,Delivery_Arm_Extend_Power,Delivery_Arm_Extend_Power),
                        new ServoAction(Wrist, Wrist_Hang_Pos)
                ),
                new DoubleServoAction(ElbowLeft,ElbowRight,ElbowL_HangDone_Pos,ElbowR_HangDone_Pos),
                new ServoAction(Claw, Claw_Open_Pos),*/

