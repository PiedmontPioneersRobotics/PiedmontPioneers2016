package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
public class HardwareLauncherBot2
{

    public DcMotor  WaterMillMotor   = null;
    public DcMotor  RubberBandMotor  = null;
    public DcMotor  backleftMotor   = null;
    public DcMotor  backrightMotor  = null;
    public DcMotor  pitcherMotorleft = null;
    public DcMotor  pitcherMotorright = null;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareLauncherBot2(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        WaterMillMotor   = hwMap.dcMotor.get("WaterMillmotor");
        RubberBandMotor  = hwMap.dcMotor.get("RubberBandmotor");
        backleftMotor   = hwMap.dcMotor.get("backleft");
        backrightMotor  = hwMap.dcMotor.get("backright");
        pitcherMotorleft   = hwMap.dcMotor.get("pitcherMotorleft");
        pitcherMotorright  = hwMap.dcMotor.get("pitcherMotorright");
        WaterMillMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        RubberBandMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backleftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backrightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        pitcherMotorleft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        pitcherMotorright.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        WaterMillMotor.setPower(0);
        RubberBandMotor.setPower(0);
        backleftMotor.setPower(0);
        backrightMotor.setPower(0);
        pitcherMotorleft.setPower(0);
        pitcherMotorright.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        WaterMillMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RubberBandMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pitcherMotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pitcherMotorright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

