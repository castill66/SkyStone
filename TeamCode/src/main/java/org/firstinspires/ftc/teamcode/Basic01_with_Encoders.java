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

@Autonomous(name="BasicDCMotor01_Encoders", group="Basic testing")
//@Disabled

public class Basic01_with_Encoders extends LinearOpMode {

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
    double TimeOut    = 10.0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFront = null;
    //private DcMotor LeftMiddle = null;
    private DcMotor LeftRear = null;
    private DcMotor RightFront = null;
    private DcMotor RightMiddle = null;
    private DcMotor RightRear = null;


    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // eg: Neverest 20
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


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


        // reset encoder count kept by left motor.
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // reset encoder count kept by left motor.
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // reset encoder count kept by left motor.
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // reset encoder count kept by left motor.
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        waitForStart();


        runtime.reset();

        // set left motor to run to 5000 encoder counts.
        LeftFront.setTargetPosition((int) (COUNTS_PER_INCH*12.0));
        // set left motor to run to 5000 encoder counts.
        LeftRear.setTargetPosition((int) (COUNTS_PER_INCH*12.0));
        // set left motor to run to 5000 encoder counts.
        RightFront.setTargetPosition((int) (COUNTS_PER_INCH*12.0));
        // set left motor to run to 5000 encoder counts.
        RightRear.setTargetPosition((int) (COUNTS_PER_INCH*12.0));


        // set left motor to run to target encoder position and stop with brakes on.
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // set left motor to run to target encoder position and stop with brakes on.
        LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // set left motor to run to target encoder position and stop with brakes on.
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // set left motor to run to target encoder position and stop with brakes on.
        RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Send calculated power to wheels
        LeftFront.setPower(leftPower);
        //LeftMiddle.setPower(leftPower);
        LeftRear.setPower(leftPower);
        RightFront.setPower(rightPower);
        //RightMiddle.setPower(rightPower);
        RightRear.setPower(rightPower);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && runtime.time() < TimeOut && (LeftFront.isBusy() && LeftRear.isBusy()&& RightFront.isBusy() && RightRear.isBusy())) {

            // Display it for the driver.
            telemetry.addData("LeftFront Encoder = ", "%7d", LeftFront.getCurrentPosition());
            telemetry.addData("LeftRear  Encoder = ", "%7d", LeftRear.getCurrentPosition());
            telemetry.addData("RightFront Encoder = ", "%7d", RightFront.getCurrentPosition());
            telemetry.addData("RightRear  Encoder = ", "%7d", RightRear.getCurrentPosition());
            telemetry.update();

        }
        // Send calculated power to wheels
        LeftFront.setPower(0.0);
        //LeftMiddle.setPower(leftPower);
        LeftRear.setPower(0.0);
        RightFront.setPower(0.0);
        //RightMiddle.setPower(rightPower);
        RightRear.setPower(0.0);
/*            if(runtime.time() > TimeOut)
            {
                leftPower = 0;
                rightPower = 0;

            }*/

/*            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();*/

    }
}
