/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BasicDCMotor04", group="Basic testing")
//@Disabled

public class Basic04 extends LinearOpMode {

    // Name of the Motors in the Robot Configuration
    String NameLeftFront     = "LF";
    //String NameLeftMiddle  = "LM";
    String NameLeftRear      = "LR";
    String NameRightFront    = "RF";
    //String NameRightMiddle = "RM";
    String NameRightRear     = "RR";

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower  = 0.2;
    double rightPower = 0.2;
    double timer;
    double t1 = 2.0;
    double t2 = 2.0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFront = null;
    //private DcMotor LeftMiddle = null;
    private DcMotor LeftRear = null;
    private DcMotor RightFront = null;
    private DcMotor RightMiddle = null;
    private DcMotor RightRear = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LeftFront  = hardwareMap.get(DcMotor.class, NameLeftFront);
        //LeftMiddle = hardwareMap.get(DcMotor.class, NameLeftMiddle);
        LeftRear = hardwareMap.get(DcMotor.class, NameLeftRear);
        RightFront = hardwareMap.get(DcMotor.class, NameRightFront);
        //RightMiddle = hardwareMap.get(DcMotor.class, NameRightMiddle);
        RightRear = hardwareMap.get(DcMotor.class, NameRightRear);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        //LeftMiddle.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);

        RightFront.setDirection(DcMotor.Direction.FORWARD);
        //RightMiddle.setDirection(DcMotor.Direction.FORWARD);
        RightRear.setDirection(DcMotor.Direction.FORWARD);

        //******************************************************************************************
        // Wait for the game to start (driver presses PLAY)
        //******************************************************************************************
        waitForStart();

        // Resetting the timer
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //  first corner
            // Send Power to wheels
            leftPower = 0.3;
            rightPower = -0.3;
            LeftFront.setPower(leftPower);
            LeftRear.setPower(leftPower);
            RightFront.setPower(rightPower);
            RightRear.setPower(rightPower);
            // Waiting for certain time
            timer = runtime.time();
            while(runtime.time() - timer <  0.2 && opModeIsActive());


            // first parallel
            leftPower = 0.3;
            rightPower = 0.3;
            LeftFront.setPower(leftPower);
            LeftRear.setPower(leftPower);
            RightFront.setPower(rightPower);
            RightRear.setPower(rightPower);
            // Waiting for certain time
            timer = runtime.time();
            while(runtime.time() - timer <  4.8 && opModeIsActive());


            //  second corner
            // Send Power to wheels
            leftPower = 0.3;
            rightPower = -0.3;
            LeftFront.setPower(leftPower);
            LeftRear.setPower(leftPower);
            RightFront.setPower(rightPower);
            RightRear.setPower(rightPower);
            // Waiting for certain time
            timer = runtime.time();
            while(runtime.time() - timer <  1.9 && opModeIsActive());


            //second parallel
            leftPower = 0.3;
            rightPower = 0.3;
            LeftFront.setPower(leftPower);
            LeftRear.setPower(leftPower);
            RightFront.setPower(rightPower);
            RightRear.setPower(rightPower);
            // Waiting for certain time
            timer = runtime.time();
            while(runtime.time() - timer <  4.6 && opModeIsActive());


            //third corner
            leftPower = 0.3;
            rightPower = -0.3;
            LeftFront.setPower(leftPower);
            LeftRear.setPower(leftPower);
            RightFront.setPower(rightPower);
            RightRear.setPower(rightPower);
            // Waiting for certain time
            timer = runtime.time();
            while(runtime.time() - timer <  1.3 && opModeIsActive());

            //third line
            leftPower = 0.3;
            rightPower = 0.3;
            LeftFront.setPower(leftPower);
            LeftRear.setPower(leftPower);
            RightFront.setPower(rightPower);
            RightRear.setPower(rightPower);
            // Waiting for certain time
            timer = runtime.time();
            while(runtime.time() - timer <  2.5 && opModeIsActive());
/*
            //third corner
            leftPower = 0.3;
            rightPower = -0.3;
            LeftFront.setPower(leftPower);
            LeftRear.setPower(leftPower);
            RightFront.setPower(rightPower);
            RightRear.setPower(rightPower);
            // Waiting for certain time
            timer = runtime.time();
            while(runtime.time() - timer <  1.2 && opModeIsActive());

            //fourth line
            leftPower = 0.3;
            rightPower = 0.3;
            LeftFront.setPower(leftPower);
            LeftRear.setPower(leftPower);
            RightFront.setPower(rightPower);
            RightRear.setPower(rightPower);
            // Waiting for certain time
            timer = runtime.time();
            while(runtime.time() - timer <  2.4 && opModeIsActive());
           */

            // STOPPING THE ROBOT
            // Send Power to wheels
            leftPower = 0;
            rightPower = 0;
            LeftFront.setPower(leftPower);
            LeftRear.setPower(leftPower);
            RightFront.setPower(rightPower);
            RightRear.setPower(rightPower);


            // Waiting for more time than autonon
            timer = runtime.time();
            while(runtime.time() - timer <  200 && opModeIsActive());


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

        }

    }
}
