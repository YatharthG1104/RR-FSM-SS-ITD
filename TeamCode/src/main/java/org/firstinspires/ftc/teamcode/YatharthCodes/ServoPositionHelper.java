package org.firstinspires.ftc.teamcode.YatharthCodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Position Helper", group = "Concept")
public class ServoPositionHelper extends OpMode {

    private Servo axon = null;
    private double servoPosition = 0.5;
    private double positionAdjustment = 0.05;
    private final double STEP_ADJUSTMENT = 0.01;
    private final double MIN_POSITION = 0;
    private final double MAX_POSITION = 1;

    private boolean previousGamepadY = false;
    private boolean previousGamePadA = false;
    private boolean previousGamePadUp = false;
    private boolean previousGamePadDown = false;

    @Override
    public void init() {
        axon = hardwareMap.get(Servo.class, "axon");
        axon.setPosition(servoPosition);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        boolean currentGamepadY = gamepad1.y;
        boolean currentGamepadA = gamepad1.a;
        boolean currentGamepadUp = gamepad1.dpad_up;
        boolean currentGamepadDown = gamepad1.dpad_down;

        if (currentGamepadY && !previousGamepadY) {
            servoPosition += positionAdjustment;
        } else if (currentGamepadA && !previousGamePadA) {
            servoPosition -= positionAdjustment;
        }

        if (currentGamepadUp && !previousGamePadUp) {
            positionAdjustment += STEP_ADJUSTMENT;
        } else if (currentGamepadDown && !previousGamePadDown) {
            positionAdjustment -= STEP_ADJUSTMENT;
        }

        if (positionAdjustment < 0.01) {
            positionAdjustment = 0.01;
        } else if (positionAdjustment > 0.1) {
            positionAdjustment = 0.1;
        }

        if (servoPosition > MAX_POSITION) {
            servoPosition = MAX_POSITION;
        } else if (servoPosition < MIN_POSITION) {
            servoPosition = MIN_POSITION;
        }

        axon.setPosition(servoPosition);

        previousGamepadY = currentGamepadY;
        previousGamePadA = currentGamepadA;
        previousGamePadUp = currentGamepadUp;
        previousGamePadDown = currentGamepadDown;

        telemetry.addData("Servo Position", servoPosition);
        telemetry.addData("Servo Step Size", positionAdjustment);
        telemetry.update();
    }
}
