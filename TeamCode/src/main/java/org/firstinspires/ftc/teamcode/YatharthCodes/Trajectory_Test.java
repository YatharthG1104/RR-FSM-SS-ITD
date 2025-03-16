package org.firstinspires.ftc.teamcode.YatharthCodes;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "First trajectory test")

public class Trajectory_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d InitialPose = new Pose2d(0, 0, 0);


        //this takes care of drive motors in MecanumDrive class.
        // Define other non drive motors before you move forward
        MecanumDrive drive = new MecanumDrive(hardwareMap, InitialPose);

        // Define all servos here or put all hardware initialization separately
        // Servo servo= hardwareMap.servo.get("servo");
//        Servo W = hardwareMap.get(Servo.class, "Wrist");
        Servo C = hardwareMap.get(Servo.class, "Claw");
        DcMotor FS = hardwareMap.get(DcMotor.class, "Front Slide");
        DcMotor DAL = hardwareMap.get(DcMotor.class, "Delivery ArmL");
        DcMotor DAR = hardwareMap.get(DcMotor.class, "Delivery ArmR");
        Servo EL = hardwareMap.get(Servo.class, "Elbow Left");
        Servo ER = hardwareMap.get(Servo.class, "Elbow Right");
        Servo GL = hardwareMap.get(Servo.class, "Front Claw");
        CRServo GR = hardwareMap.get(CRServo.class, "Front Wrist");
        Servo TWL = hardwareMap.get(Servo.class, "Twist Left");
        Servo TWR = hardwareMap.get(Servo.class, "Twist Right");
        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "Color");

        EL.setDirection(Servo.Direction.REVERSE);
        EL.scaleRange(-1,1);

        ER.setDirection(Servo.Direction.FORWARD);
        ER.scaleRange(-1,1);

        TWL.setDirection(Servo.Direction.REVERSE);
        TWR.setDirection(Servo.Direction.FORWARD);

        DAR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        DAR.setDirection(DcMotor.Direction.REVERSE);          //Delivery arm left motor set reversed
        DAR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        DAR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders

        DAL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        DAL.setDirection(DcMotor.Direction.FORWARD);          //Delivery arm left motor set reversed
        DAL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        DAL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders

        FS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        FS.setDirection(DcMotor.Direction.REVERSE);          //Delivery arm left motor set reversed
        FS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        FS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders


        waitForStart();


// Velocity and Acceleration Constraints
 /*
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(150.0),
                new AngularVelConstraint(Math.PI / 2)
        ));*/
//        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10, 100);


//Method 1: Roadrunner Action Builder

        /*Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToX(28)
                        .waitSeconds(0.5)
                        .setReversed(true)
                        .setTangent(Math.toRadians(0))
                        .lineToX(15)
                        .strafeTo(new Vector2d(15, -25))
                        .strafeTo(new Vector2d(55, -35))
                        .strafeTo(new Vector2d(55, -40))
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(5)
                        .strafeTo(new Vector2d(55,-42))
                        .strafeTo(new Vector2d(55,-50))
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(5)
                        .strafeToLinearHeading(new Vector2d(3,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(3,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(3,-45), 0)
                        .build());*/

       /* Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToX(28)
                        .waitSeconds(0.5)
                        .setReversed(true)
                        .setTangent(Math.toRadians(0))
                        .lineToX(10)
                        .setTangent(-Math.PI/2)
                        .splineToConstantHeading(new Vector2d(55,-33), Math.PI/2)
                        .setTangent(Math.PI/2)
                        .splineToConstantHeading(new Vector2d(55,-40), Math.PI/2)
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(8)
                        .strafeTo(new Vector2d(55,-42))
                        .strafeTo(new Vector2d(55,-50))
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(8)
                        .strafeToLinearHeading(new Vector2d(5,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(5,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(5,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(5,-40), 0)
                        .build());*/

      /*  Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToX(28)
                        .waitSeconds(0.5)
                        .setReversed(true)
                        .setTangent(Math.toRadians(0))
                        .lineToX(10)
                        .setTangent(-Math.PI/2)
                        .splineToConstantHeading(new Vector2d(55,-33), Math.PI/2)
                      //  .splineToConstantHeading(new Vector2d(55,-33), Math.PI/2, new TranslationalVelConstraint(100), baseAccelConstraint)
                        .setTangent(Math.PI/2)
                        .splineToConstantHeading(new Vector2d(55,-40), Math.PI/2)
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(8)
                        .strafeTo(new Vector2d(55,-42))
                        .strafeTo(new Vector2d(55,-50))
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(8)
                        .strafeTo(new Vector2d(55,-50))
                        .strafeTo(new Vector2d(55,-57))
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(8)
                        .strafeToLinearHeading(new Vector2d(5,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(5,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(5,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(5,-40), 0)
                        .build());*/


 /*   Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .stopAndAdd(new ServoAction(W,0.5))
                       .afterTime(0.5, new SequentialAction(
                               new ServoAction(W,0.8),
                               new ServoAction(C, 0.5)
                       ))
                        .build());
*/


//Method 2: Trajectory Action Builder


    /*   TrajectoryActionBuilder Path1 = drive.actionBuilder(InitialPose)

                .lineToX(28)
               .waitSeconds(0.5)
                .afterTime(0,new ServoAction(W,0.3))
               .afterTime(0,new ServoAction(C,0.3));
                .afterTime(0.5, new ParallelAction(
                      new ServoAction(W, 0.5),
                      new ServoAction(C, 0.8)
        Action trajectorychosen;
       trajectorychosen= Path1.build();
*/


//Method 3: Action Runblocking - works

        Actions.runBlocking(
                new SequentialAction(
                      drive.actionBuilder(new Pose2d(0,0,0)) // Another way of running a trajectory (not recommended because trajectories take time to build and will slow down your code, always try to build them beforehand)
                              .lineToX(28.8)
                              .setReversed(true)
                              .setTangent(Math.toRadians(0))
                              .lineToX(8)
                              .setTangent(-Math.PI/2)
//                                .splineToConstantHeading(new Vector2d(53,-33), Math.PI/2)
//                                .strafeTo(new Vector2d(54,-40))
                              .splineToConstantHeading(new Vector2d(51,-30), Math.PI/2)
                              .strafeTo(new Vector2d(51,-40))
                              .setReversed(true)
                              .setTangent(0)
                              .lineToX(12)
                              .waitSeconds(0.5)
                              .setReversed(true)
                              .setTangent(0)
                              .lineToX(7)
                              .strafeTo(new Vector2d(0,0))
                              .build()


                        //trajectorychosen,
                        //new DoubleMotorAction(DAL,DAR,1200, 1200, 0.9,0.9),
                    //    new DoubleServoAction(EL,ER, 0.15,0.15),
                    //    new DoubleServoAction(TWL,TWR, 0.7,0.7),
                     //   new ServoAction(W, 0.4),
//                        new ServoAction(GL, 0),
//                       new AlignSensorAction(color, GR, GL, 1)
                       // new ServoAction(C, 0.55),
//                        new DoubleServoAction(TWL,TWR, -0.5,0.5),
                     //   new ServoAction(GL, 0.75),
                      //  new DoubleServoAction(EL,ER, 0.1,0.1),
                     //   new DoubleServoAction(TWL,TWR, 0.4,0.4),
                       // new ServoAction(GL, -0.5),
                       // new DoubleMotorAction(DAL,DAR,20, 20, -0.9,-0.9),
                         // new DoubleServoAction(TWL,TWR, 0,0),
                          //new ServoAction(GL, -1)
                        //new DoubleServoAction(EL,ER, 0.45,0.45),
                        //new ServoAction(GL, 1.0),
                        //new DoubleServoAction(EL,ER, 0.7,0.7),
//                        new DoubleServoAction(TWL,TWR, 0,0),
                   //     new ServoAction(GL, -1)
                       // new ServoAction(GL, -1.0),
                        //new ServoAction(GL, 1.0),
                        //new DoubleServoAction(EL,ER, 0.4,0.4)
                       // new DoubleMotorAction(DAL,DAR,2200, 2200, 0.9,0.9),
                        //new ServoAction(C, 1.0),
                        //new ServoAction(GL, 1.0),
                        //new ServoAction(GL, -1.0)
                        // new ServoAction(GL, 0.7),
                    //    new MotorAction2(FS,500, 0.3),
                      //  new DoubleServoAction(TWL,TWR, 0.95,0.95),
                       // new DoubleServoAction(TWL,TWR, 0.4,0.4),
                       // new DoubleMotorAction(DAL,DAR,490, 490, -0.9,-0.9)
                        //new DoubleServoAction(TWL,TWR, 0.1,0.1)
                        //new MotorAction2(FS,0,-0.3),
                       /* new ParallelAction(
                                new MotorAction2(DAL,500, -0.6),
                                new MotorAction2(DAR, 500, -0.6)*/
                        )
                      //drive.actionBuilder(new Pose2d(10,0,0)) // Another way of running a trajectory (not recommended because trajectories take time to build and will slow down your code, always try to build them beforehand)
                            //    .strafeTo(new Vector2d(10,10))
                              //  .build()(/
                       // new DoubleMotorAction(DAL,DAR,500, 500, -0.6,-0.6)



        //new ServoAction(GR,0.8),
                        //             new ServoAction(W, 1.0),
                        //    //     new ServoAction(GL, 1),
                        //  new ServoAction(GR, 1),
                        // new DoubleServoAction(EL,ER, 0.7,0.7)
        );
        telemetry.addData("Delivery ArmL Position: ", DAL.getCurrentPosition());
        telemetry.addData("Delivery ArmR Position: ", DAR.getCurrentPosition());
//        telemetry.addData("Wrist Position: ", W.getPosition());
        telemetry.addData("Claw Position: ", C.getPosition());
        telemetry.addData("Elbowleft Position: ", EL.getPosition());
        telemetry.addData("ElbowRight Position: ", ER.getPosition());
        telemetry.addData("Frontslide Position: ", FS.getCurrentPosition());
      //  telemetry.addData("Front Claw Position:", GR.getPosition());
        telemetry.addData("Front Claw Position:", GL.getPosition());
        telemetry.addData("Twist Left Position: ", TWL.getPosition());
        telemetry.addData("Twist Right Position: ", TWR.getPosition());
        telemetry.update();


      /*  Actions.runBlocking(
                new ParallelAction(
                       new MotorAction(DAL,2000,0.8),
                        new MotorAction(DAR, 2000, -0.8)

                )
        );*/
    }


    // Servo action build for roadrunner to use
    public static class ServoAction implements Action {
        Servo servo;
        double position;
        ElapsedTime timer = null;

        public ServoAction(Servo srv, double pos) {
            this.servo = srv;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                servo.setPosition(position);
            }

            return timer.seconds() < 1;
        }
    }

    // Motor Action build for roadrunner to use
    public static class MotorAction implements Action {
        DcMotor motor;
        double position_tgt;
        double power;
        ElapsedTime timer = null;

        public MotorAction(DcMotor mot, double pos, double pow) {
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
            return timer.seconds() < 0.5;
        }
    }

    //CRServo Action build for roadrunner to use
    public static class CRServoAction implements Action {
        CRServo crServo;
        double power;
        ElapsedTime timer = null;

        public CRServoAction(CRServo crsrv, double pow) {
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
        ElapsedTime timer = null;

        public DoubleServoAction(Servo srv1, Servo srv2, double pos1, double pos2) {
            this.servo1 = srv1;
            this.servo2 = srv2;
            this.position1 = pos1;
            this.position2 = pos2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                servo1.setPosition(position1);
                servo2.setPosition(position2);
            }

            return timer.seconds() < 1;
        }
    }

    //Double CRServo Action build for roadrunner to use
    public static class DoubleCRServoAction implements Action {
        CRServo crServo1;
        CRServo crServo2;
        double power1;
        double power2;
        int seconds1;
        int seconds2;
        ElapsedTime timer = null;

        public DoubleCRServoAction(CRServo crsrv1, CRServo crsrv2, int sec1, int sec2, double pow1, double pow2) {
            this.crServo1 = crsrv1;
            this.crServo2 = crsrv2;
            this.seconds1 = sec1;
            this.seconds2 = sec2;
            this.power1 = pow1;
            this.power2 = pow2;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer.seconds() < seconds1) {
                timer = new ElapsedTime();
                crServo1.setPower(power1);
                crServo2.setPower(power2);
            }
            return timer.seconds() < 0.5;
        }
    }

    // Double Motor and Servo Action build for roadrunner to use
    public static class DoubleServoAndMotorAction implements Action {

        DcMotor motor;
        Servo servo;
        double Motor_pos_target;
        double motor_power;
        double servPos;
        ElapsedTime timer = null;

        public DoubleServoAndMotorAction(DcMotor mot, Servo serv, double motPos, double motPow, double servoPos) {
            this.motor = mot;
            this.servo = serv;
            this.Motor_pos_target = motPos;
            this.motor_power = motPow;
            this.servPos = servoPos;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                servo.setPosition(servPos);
                motor.setPower(motor_power);
                motor.setTargetPosition((int) (Motor_pos_target));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            return timer.seconds() < 0.5;
        }
    }

    // Servo action-2 build for roadrunner to use
    public static class ServoAction2 implements Action {
        Servo servo;
        double position;

        public ServoAction2(Servo srv, double pos) {
            this.servo = srv;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(position);
            return false;
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

    public class AlignSensorAction implements Action {
        public RevColorSensorV3 colorSensor;
        public CRServo turningServo;
        public double position;
        public Servo servo;
        // public int alliance_color;

        // Thresholds
        public  int LINE_THRESHOLD = 100;  //color intensity
        public  int YELLOW_THRESHOLD = 50;  //color intensity
        public  int BLUE_THRESHOLD = 90;
        public  double MIN_DISTANCE_THRESHOLD = 5.0; // in cm

        public AlignSensorAction(RevColorSensorV3 colorSensor, CRServo turningServo, Servo srv, double pos) {
            this.colorSensor = colorSensor;
            this.turningServo = turningServo;
            this.position = pos;
            this.servo = srv;
            // this.alliance_color = all_col;

        };
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            // Get RGB values
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Get the current distance measurement
            double distance = colorSensor.getDistance(DistanceUnit.CM);

            telemetry.addData("Red Color", red);
            telemetry.addData("Green Color", green);
            telemetry.addData("Blue Color", blue);
            telemetry.addData("Distance", distance);
            telemetry.update();

            boolean isYellow = (red > LINE_THRESHOLD && green > YELLOW_THRESHOLD && blue < LINE_THRESHOLD);
            boolean isBlue = (red < BLUE_THRESHOLD && green < BLUE_THRESHOLD && blue > LINE_THRESHOLD);
            boolean isAligned = (red > LINE_THRESHOLD && green < LINE_THRESHOLD && blue < LINE_THRESHOLD);
            boolean isWithinDist = (distance <= MIN_DISTANCE_THRESHOLD);


            if (isAligned) {
                turningServo.setPower(0);
            } else {
                turningServo.setPower(1);
            }

            if (isWithinDist) {
                turningServo.setPower(0);
            } else {
                turningServo.setPower(1);
            }

            if ((isYellow || isBlue) && isAligned && isWithinDist) {
                servo.setPosition(position);
                return true;
            } else
                return false;
        }

    }



}