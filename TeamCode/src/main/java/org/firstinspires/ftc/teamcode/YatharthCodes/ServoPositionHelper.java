package org.firstinspires.ftc.teamcode.YatharthCodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Servo Position Helper", group = "Concept")
public class ServoPositionHelper extends OpMode {

    private Servo axon = null;
    private Servo el = null;
    private Servo er = null;
    private double servoPositionaxon = 0.5;
   // private double servoPositionel = 0;
    private double positionAdjustment = 0.05;
   // private final double STEP_ADJUSTMENT = 0.05;
    private final double MIN_POSITION = -1.0;
    private final double MAX_POSITION = 1.0;

    private boolean previousGamepadY = false;
    private boolean previousGamePadA = false;
    private boolean previousGamePadUp = false;
    private boolean previousGamePadDown = false;

    @Override
    public void init() {
//        axon = hardwareMap.get(Servo.class, "Wrist");
        //        axon.setDirection(Servo.Direction.FORWARD);
        //        axon.setPosition(servoPositionaxon);

      //  el = hardwareMap.get(Servo.class, "Elbow Left");
      //  el.scaleRange(0,1);
        er = hardwareMap.get(Servo.class, "Wrist");

      //  el.setDirection(Servo.Direction.REVERSE);
        er.setDirection(Servo.Direction.FORWARD);

      // el.setPosition(servoPositionaxon);
        er.setPosition(servoPositionaxon);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        boolean currentGamepadY = gamepad1.y;
        boolean currentGamepadA = gamepad1.a;
        boolean currentGamepadUp = gamepad1.dpad_up;
        boolean currentGamepadDown = gamepad1.dpad_down;

        if (currentGamepadY && !previousGamepadY) {
            servoPositionaxon += positionAdjustment;
        } else if (currentGamepadA && !previousGamePadA) {
            servoPositionaxon -= positionAdjustment;
        }
/*
        if (currentGamepadUp && !previousGamePadUp) {
            positionAdjustment += STEP_ADJUSTMENT;
        } else if (currentGamepadDown && !previousGamePadDown) {
            positionAdjustment -= STEP_ADJUSTMENT;
        }
*/
        if (positionAdjustment < 0.01) {
            positionAdjustment = 0.01;
        } else if (positionAdjustment > 0.1) {
            positionAdjustment = 0.1;
        }

        if (servoPositionaxon > MAX_POSITION) {
            servoPositionaxon = MAX_POSITION;
        } else if (servoPositionaxon < MIN_POSITION) {
            servoPositionaxon = MIN_POSITION;
        }

        //el.setPosition(servoPositionaxon);
        er.setPosition(servoPositionaxon);

        previousGamepadY = currentGamepadY;
        previousGamePadA = currentGamepadA;
        previousGamePadUp = currentGamepadUp;
        previousGamePadDown = currentGamepadDown;

       // telemetry.addData("El Servo Position", el.getPosition());
       telemetry.addData("Er Servo Position", er.getPosition());
        telemetry.addData("Target Servo Position", servoPositionaxon);
        //telemetry.addData("el Servo Position", servoPositionel);
        telemetry.addData("Servo Step Size", positionAdjustment);
        telemetry.update();
    }
}
