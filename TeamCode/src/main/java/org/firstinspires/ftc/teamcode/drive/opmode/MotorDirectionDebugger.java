package org.firstinspires.ftc.teamcode.drive.opmode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".(
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *1
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Motor Direction Debugger", group="Iterative Opmode")
public class MotorDirectionDebugger extends OpMode {
    // Declare OpMode members.

    private DcMotor leftUpDrive = null;

    private DcMotor leftSpin = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightSpin = null;
    private DcMotor lift;
    private DcMotor rightUpDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo claw;
    boolean isOn = true;

    private double leftPower;

    private Servo pincer;

    private double rightPower;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    private int rotation = 0;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftSpin = hardwareMap.get(DcMotor.class, "left_Spin");
        rightSpin = hardwareMap.get(DcMotor.class, "right_Spin");
        leftUpDrive = hardwareMap.get(DcMotor.class, "left_up");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_Back");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_Back");
        rightUpDrive = hardwareMap.get(DcMotor.class, "right_up");
        lift = hardwareMap.get(DcMotor.class, "lift");
        pincer = hardwareMap.get(Servo.class, "pincer");
        lift.setTargetPosition(rotation);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //claw = hardwareMap.get(Servo.class, "claw");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftSpin.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSpin.setDirection(DcMotorSimple.Direction.FORWARD);

        leftUpDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightUpDrive.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

       // lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double MotorMaxSpeed = 0.6;
        double leftFront = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double leftRear = Range.clip(gamepad1.left_stick_y + gamepad1.right_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double rightRear = -Range.clip(gamepad1.right_stick_y - gamepad1.left_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        double rightFront = -Range.clip(gamepad1.right_stick_y + gamepad1.right_stick_x, -MotorMaxSpeed, MotorMaxSpeed);
        leftUpDrive.setPower(leftFront);
        leftBackDrive.setPower(leftRear);
        rightUpDrive.setPower(-rightFront);
        rightBackDrive.setPower(rightRear);
        leftSpin.setPower(gamepad2.left_trigger);
        rightSpin.setPower(gamepad2.right_trigger);
        if (gamepad2.circle){
            lift.setPower(0.5);
            isOn = true;

        }
        else if(gamepad2.square){
            lift.setPower(0.0);
            isOn = false;
            rotation = 0;
        }


        lift.setTargetPosition(rotation);

        if (isOn) {
            if (gamepad2.dpad_up) {
                //lift.setPower(0.5);
                if (rotation >= 6000) {
                    rotation = rotation;
                }
                else{
                    rotation = rotation + 20;
                }

            } else if (gamepad2.dpad_down) {
                //lift.setPower(-0.5);
                rotation = rotation - 20;
            }
        }

        //rightPower = gamepad1.right_stick_y;
        //rightDrive.setPower(rightPower);


        if (gamepad2.right_bumper) {
            pincer.setPosition(0.3);
        }

        if (gamepad2.left_bumper) {
            pincer.setPosition(0);
        }
        telemetry.addData("Lift", "position" + rotation);
        telemetry.addData("Pos",pincer.getPosition());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}