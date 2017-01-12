/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name= "LauncherBot: Teleop POV", group="LauncherBot")
@Disabled
public class LauncherBotTeleop2_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareLauncherBot2 robot = new HardwareLauncherBot2();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.

    // Comment to test github.
    // Testing from Reily's computer

    @Override
    public void runOpMode() throws InterruptedException {
        double left = 0;
        double right = 0;
        boolean waterMill = false;
        double rubberbandspeed = 0;
        boolean rubberBand = false;
        double waterMillSpeed = 0;
        double max;
        boolean touchsensor;
        TouchSensor touchSensor;
        long current_time = 0;
        boolean ballinchute = true;
        float watermillback = 0.0f;
        float rubberbandback = 0.0f;
        touchSensor = hardwareMap.touchSensor.get("sensor_touch");


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Say", "Started");    //
        telemetry.update();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            waterMill = gamepad2.left_bumper;
            rubberBand = gamepad2.right_bumper;
            watermillback = gamepad2.left_trigger;
            rubberbandback = gamepad2.left_trigger;

            robot.pitcherMotorright.setPower(0.6);
            robot.pitcherMotorleft.setPower(0.6);

            //current_time = System.nanoTime();


            if (touchSensor.isPressed()) {
                ballinchute = true;
            }

            if (gamepad2.right_bumper)
                ballinchute = false;


            if (ballinchute && touchSensor.isPressed()) {
                waterMillSpeed = -0.2;
            }

            else if (ballinchute && !touchSensor.isPressed()) {
                waterMillSpeed = 0.0;
            }

            else waterMillSpeed = 0.2;

            if (watermillback> 0.5) {waterMillSpeed = -waterMillSpeed;}


            if (rubberBand) rubberbandspeed = 0.0;
            else rubberbandspeed = 1.0;

            if (rubberbandback> 0.5) {rubberbandspeed = -rubberbandspeed;}

            robot.WaterMillMotor.setPower(waterMillSpeed);
            robot.RubberBandMotor.setPower(rubberbandspeed);
            robot.backleftMotor.setPower(left);
            robot.backrightMotor.setPower(right);


            // Send telemetry message to signify robot running;
            telemetry.addData("left", "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            if (touchSensor.isPressed()) telemetry.addLine("Chute button is pressed");
            else telemetry.addLine("Chute button is NOT pressed");
            telemetry.update();
        }

    }
}
