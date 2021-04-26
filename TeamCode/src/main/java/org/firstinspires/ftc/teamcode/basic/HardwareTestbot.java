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

package org.firstinspires.ftc.teamcode.basic;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class HardwareTestbot
{
    /* Public OpMode members. */
    public DcMotorEx lf  = null;
    public DcMotorEx  lr  = null;
    public DcMotorEx  rf  = null;
    public DcMotorEx  rr  = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTestbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lf = hwMap.get(DcMotorEx.class, "lf");
        rf = hwMap.get(DcMotorEx.class, "rf");
        lr = hwMap.get(DcMotorEx.class, "lr");
        rr = hwMap.get(DcMotorEx.class, "rr");
        lr.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);

        // Set all motors to run without encoders.
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    /**
     *
     * DRIVE FUNCTIONS
     *
     */


    //sets power to chassis motors
    public void drive(double lfPower, double rfPower, double lrPower, double rrPower){
        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lr.setPower(lrPower);
        rr.setPower(rrPower);

    }


    //Drive forward a certain distance
    public void EncoderDrive(int distance) {

        stopAndResetEncoders();

        setRuntoPosition();

        int startPosition = lf.getCurrentPosition();
        int targetValue = startPosition + distance;

        double maxDistance = distance + 765;
        double minSpeed = 0.2;
        double fullSpeed = 1;
        double distSpeed = fullSpeed-minSpeed;

        double scalar = distSpeed / Math.pow(maxDistance, 2); //1/5100d for linear 4/83436125d for expon at max of 4085 or 1 / 18336125d for no reason

        lf.setTargetPosition(targetValue);
        lr.setTargetPosition(targetValue);
        rf.setTargetPosition(targetValue);
        rr.setTargetPosition(targetValue);

        while (rr.isBusy()) {

            double distRemaining = targetValue - rr.getCurrentPosition();

            double ex_Var_Speed = Math.abs(scalar * Math.pow(distRemaining, 2)) + minSpeed;

            lf.setPower(ex_Var_Speed);
            lr.setPower(ex_Var_Speed);
            rr.setPower(ex_Var_Speed);
            rf.setPower(ex_Var_Speed);

        }

        stopAndResetEncoders();

    }

    //Drive the robot in any direction defined by unit vector components
    public void mecDrive(float x_unit_vector, float y_unit_vector, float turn, float max_power_level) {
        //limit drive power to the max_power_level value
        x_unit_vector = Range.clip(x_unit_vector, -max_power_level, max_power_level);
        y_unit_vector = Range.clip(y_unit_vector, -max_power_level, max_power_level);
        turn = Range.clip(turn, -max_power_level, max_power_level);

        double robotAngle;
        double r = Math.hypot(-x_unit_vector, y_unit_vector);

        robotAngle = Math.atan2(-y_unit_vector, x_unit_vector) - Math.PI / 4;

        double turn_power = turn;
        final double v1 = r * Math.cos(robotAngle) + turn_power;
        final double v2 = r * Math.sin(robotAngle) - turn_power;
        final double v3 = r * Math.sin(robotAngle) + turn_power;
        final double v4 = r * Math.cos(robotAngle) - turn_power;

        drive(v1, v3, v2, v4);
    }









    /**
     * ENCODER FUNCTIONS
     */

    public void setRunWithoutEncoder() {
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopAndResetEncoders() {
        lf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
        rf.setPower(0);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setRuntoPosition() {
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setRunUsingEncoders() {
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }







    /**
     *
     * MISC FUNCTIONS
     *
     */

    //Trims double to range of -1 to 1
    public double rangeclip(double v) {
        return Range.clip(v, -1, 1);
    }

    //Checks to see if chassis is moving
    public boolean isChassisMoving() {
        return (lf.isBusy() && lr.isBusy() && rf.isBusy() && rr.isBusy());
    }

    //Shuts down loop for specified amount of time. Don't use it unless you want to make things suck
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.pu
        period.reset();
    }


}


