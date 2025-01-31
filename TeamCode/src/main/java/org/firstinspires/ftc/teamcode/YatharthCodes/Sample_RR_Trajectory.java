package org.firstinspires.ftc.teamcode.YatharthCodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Sample road runner Trajectory", group = "Autonomous")
public class Sample_RR_Trajectory extends LinearOpMode {

    double deliveryarm_high_tick = 3000.0;
    double deliveryarm_low_tick = 100.0;
    double deliveryarm_power= 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        // Lift class
        class Delivery_Arm {
            private DcMotorEx deliveryArm;

            public Delivery_Arm(HardwareMap hardwareMap) {
                deliveryArm = hardwareMap.get(DcMotorEx.class, "Lift motor");
                deliveryArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                deliveryArm.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            class LiftUp implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        deliveryArm.setPower(deliveryarm_power);
                        initialized = true;
                    }

                    double pos = deliveryArm.getCurrentPosition();
                    packet.put("liftPos", pos);


                    if (pos < deliveryarm_high_tick) {
                        return true; // Continue running
                    } else {
                        deliveryArm.setPower(0);
                        return false; // Stop action
                    }
                }
            }

            class LiftDown implements Action {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        deliveryArm.setPower(-deliveryarm_power);
                        initialized = true;
                    }

                    double pos = deliveryArm.getCurrentPosition();
                    packet.put("liftPos", pos);

                    if (pos > deliveryarm_low_tick) {
                        return true; // Continue running
                    } else {
                        deliveryArm.setPower(0);
                        return false; // Stop action
                    }
                }
            }

            public Action liftUp() {
                return new LiftUp();
            }

            public Action liftDown() {
                return new LiftDown();
            }
        }

        // Claw class
        class Claw {
            private Servo claw;

            public Claw(HardwareMap hardwareMap) {
                claw = hardwareMap.get(Servo.class, "claw");
            }

            class CloseClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    claw.setPosition(0.55);
                    return false;
                }
            }

            public Action closeClaw() {
                return new CloseClaw();
            }

            class OpenClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    claw.setPosition(1.0);
                    return false;
                }
            }

            public Action openClaw() {
                return new OpenClaw();
            }
        }

        // Initialize drive, claw, and lift
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Delivery_Arm lift = new Delivery_Arm(hardwareMap);

        // Vision output (mocked as an example)
        int visionOutputPosition = 1;

        // Build trajectory actions
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(33, 0), Math.toRadians(90))
                .waitSeconds(2)
                .splineTo(new Vector2d(48, 0), Math.toRadians(90))
                .strafeTo(new Vector2d(44, 30))
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(47, 30))
                .waitSeconds(3);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(37, 0), Math.toRadians(0))
                .splineTo(new Vector2d(18, 0), Math.toRadians(0))
                .waitSeconds(3);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(33, 180), Math.toRadians(180))
                .waitSeconds(2);

        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        // Close claw on init
        Actions.runBlocking(claw.closeClaw());

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Position during Init", visionOutputPosition);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (visionOutputPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (visionOutputPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftUp(),
                        claw.openClaw(),
                        lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}
