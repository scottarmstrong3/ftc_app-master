
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * FollowLineOnOff - On/Off line follower
 *
 * Very simple line follower that uses a hard coded threshold and turns one motor on
 * while the other one is off.
 *
 * As a variant, the motors run at two differnent speeds, but are still switched
 * back and forth between the two of them.
 *
 * @author Jochen Fischer
 * @version 1.0, 10/15/2016
 */
@Autonomous(name="Follow Line On/Off", group="ElonDev")
public class FollowLineOnOff extends LinearOpMode {

    // define the robot hardware:
    HardwareDriveBot robot   = new HardwareDriveBot();   // Use the Drivebot hardware

    // parameters used by the controller
    double onSpeed  = 0.4;
    double offSpeed = 0.1;
    int threshold = (5+47)/2;   // = 26 ... threshold that separates white from black

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize the hardware
        // including the use of encoders
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Threshold", threshold);
        telemetry.update();

        Log.i("ROBOT", String.format("%d",threshold));

        // Wait for the user to press the PLAY button on the DS:
        waitForStart();

        // run the PID controller:
        int loopCounter = 0;
        while(opModeIsActive()) {
            // read the current light sensor value:
            int brightness = robot.sensorColor.alpha();

            // move the robot:
            if (brightness > threshold) {
                robot.motorLeft.setPower(onSpeed);
                robot.motorRight.setPower(offSpeed);
            }
            else {
                robot.motorLeft.setPower(offSpeed);
                robot.motorRight.setPower(onSpeed);
            }

            idle();
        }
    }
}
