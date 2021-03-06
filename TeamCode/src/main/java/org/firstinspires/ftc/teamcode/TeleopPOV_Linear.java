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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="Pushbot: Teleop POV", group="TeleOp")
//@Disabled
public class TeleopPOV_Linear extends LinearOpMode {

//    /* Declare OpMode members. */
//    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
//    double          clawOffset      = 0;                       // Servo mid position
//    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

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
    double TimeOut    = 2.0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFront = null;
    //private DcMotor LeftMiddle = null;
    private DcMotor LeftRear = null;
    private DcMotor RightFront = null;
    private DcMotor RightMiddle = null;
    private DcMotor RightRear = null;
    private Servo   servo2    = null;


    @Override
    public void runOpMode() {
//        double left;
//        double right;
        double drive;
        double turn;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
//        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LeftFront  = hardwareMap.get(DcMotor.class, NameLeftFront);     // "LF"
        //LeftMiddle = hardwareMap.get(DcMotor.class, NameLeftMiddle);
        LeftRear = hardwareMap.get(DcMotor.class, NameLeftRear);        // "LR"
        RightFront = hardwareMap.get(DcMotor.class, NameRightFront);    // "RF"
        //RightMiddle = hardwareMap.get(DcMotor.class, NameRightMiddle);
        RightRear = hardwareMap.get(DcMotor.class, NameRightRear);      // "RR"


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        //LeftMiddle.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);

        RightFront.setDirection(DcMotor.Direction.FORWARD);
        //RightMiddle.setDirection(DcMotor.Direction.FORWARD);
        RightRear.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.right_stick_y*0.5;
            turn  =  gamepad1.right_stick_x*0.5;

            // Combine drive and turn for blended motion.
            leftPower  = drive + turn;
            rightPower = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0)
            {
                leftPower /= max;
                rightPower /= max;
            }

//            // Output the safe vales to the motor drives.
//            robot.leftDrive.setPower(left);
//            robot.rightDrive.setPower(right);
//
//            // Use gamepad left & right Bumpers to open and close the claw
//            if (gamepad1.right_bumper)
//                clawOffset += CLAW_SPEED;
//            else if (gamepad1.left_bumper)
//                clawOffset -= CLAW_SPEED;
//
//            // Move both servos to new position.  Assume servos are mirror image of each other.
//            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
//            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);
//
//            // Use gamepad buttons to move arm up (Y) and down (A)
//            if (gamepad1.y)
//                robot.leftArm.setPower(robot.ARM_UP_POWER);
//            else if (gamepad1.a)
//                robot.leftArm.setPower(robot.ARM_DOWN_POWER);
//            else
//                robot.leftArm.setPower(0.0);
//
//            // Send telemetry message to signify robot running;
//            telemetry.addData("claw",  "Offset = %.2f", clawOffset);

            // Send calculated power to wheels
            LeftFront.setPower(leftPower);
            //LeftMiddle.setPower(leftPower);
            LeftRear.setPower(leftPower);
            RightFront.setPower(rightPower);
            //RightMiddle.setPower(rightPower);
            RightRear.setPower(rightPower);

            if(runtime.time() > TimeOut)
            {
                leftPower = 0;
                rightPower = 0;

            }

            telemetry.addData("left",  "%.2f", leftPower);
            telemetry.addData("right", "%.2f", rightPower);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
