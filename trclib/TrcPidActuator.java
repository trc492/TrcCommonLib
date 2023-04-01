/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package TrcCommonLib.trclib;

import java.util.Arrays;
import java.util.Locale;

import TrcCommonLib.trclib.TrcPidController.PowerCompensation;

/**
 * This class implements a platform independent PID controlled actuator extending TrcPidMotor. It consists of a motor,
 * an encoder to keep track of its position, a lower limit switch to detect the zero position and a PID controller
 * allowing accurate movement to a set position. It provides methods to allow a joystick to control the actuator
 * to extend/retract or rotate within its limited range of movement and will slow down and finally stop when lower or
 * upper limit has been reached. It also provides methods to move the actuator to a specified position and hold it
 * there under load if necessary.
 * The PID controlled actuator class supports both linear and non-linear actuators. Elevator is an example of linear
 * actuators. Rotational arm is an example of non-linear actuators where raising and lowering it against gravity
 * presents non-linear load as the arm angle changes. PID control is good at controlling load with linear relationship.
 * Therefore, PID control will yield terrible result in non-linear situation. However, if we add a compensation factor
 * to linearize the load, then we can still achieve good result with PID control. The power compensation factor is
 * provided by the caller and is a function of the actuator position.
 */
public class TrcPidActuator extends TrcPidMotor
{
    /**
     * This class contains all the parameters related to the motor actuator.
     */
    public static class Parameters
    {
        public double scale = 1.0, offset = 0.0;
        public double minPos = 0.0, maxPos = 1.0;
        public TrcPidController.PidParameters pidParams;
        public boolean useMotorCloseLoopControl;
        public boolean resetPosOnLowerLimit = false;
        public double calPower = -0.25;
        public double stallMinPower = 0.0;
        public double stallTolerance = 0.0;
        public double stallTimeout = 0.0;
        public double resetTimeout = 0.0;
        public PowerCompensation powerCompensation = null;
        public double presetTolerance = 2.0;
        public double[] posPresets = null;

        /**
         * This method returns the string format of the PID actuator parameters.
         *
         * @return string format of the parameters.
         */
        @Override
        public String toString()
        {
            return String .format(
                Locale.US,
                "scale=%.1f, offset=%.1f, rangePos=(%.1f, %.1f), pidParams=%s, useMotorPid=%s, calPower=%.1f, " +
                "stallMinPower=%.1f, stallTolerance=%.1f, stallTimeout=%.1f, resetTimeout=%.1f, " +
                "presetTolerance=%.2f, posPresets=%s",
                scale, offset, minPos, maxPos, pidParams, useMotorCloseLoopControl, calPower, stallMinPower,
                stallTolerance, stallTimeout, resetTimeout, presetTolerance, Arrays.toString(posPresets));
        }   //toString

        /**
         * This method sets the scale and offset of the motor actuator. It allows the actuator to report real world
         * position units such as inches or degrees instead of sensor units.
         *
         * @param scale specifies the scale multiplier to convert position sensor unit to real world unit.
         * @param offset specifies the offset value to add to the scaled real world unit.
         * @return this parameter object.
         */
        public Parameters setScaleAndOffset(double scale, double offset)
        {
            this.scale = scale;
            this.offset = offset;
            return this;
        }   //setScaleAndOffset

        /**
         * This method sets the position range limits of the motor actuator.
         *
         * @param minPos specifies the minimum position of the actuator in scaled unit.
         * @param maxPos specifies the maximum position of the actuator in scaled unit.
         * @return this parameter object.
         */
        public Parameters setPosRange(double minPos, double maxPos)
        {
            this.minPos = minPos;
            this.maxPos = maxPos;
            return this;
        }   //setPosRange

        /**
         * This method sets the PID parameters of the PID controller used for PID controlling the motor actuator.
         *
         * @param pidParams specifies the PID parameters.
         * @return this parameter object.
         */
        public Parameters setPidParams(TrcPidController.PidParameters pidParams)
        {
            this.pidParams = pidParams;
            return this;
        }   //setPidParams

        /**
         * This method sets the PID parameters of the PID controller used for PID controlling the motor actuator.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param kF specifies the Feed Forward constant.
         * @param iZone specifies the integral zone.
         * @param tolerance specifies the tolerance.
         * @return this parameter object.
         */
        public Parameters setPidParams(double kP, double kI, double kD, double kF, double iZone, double tolerance)
        {
            // TrcPidMotor is providing the PidInput method.
            this.pidParams = new TrcPidController.PidParameters(kP, kI, kD, kF, iZone, tolerance, null);
            return this;
        }   //setPidParams

        /**
         * This method sets the PID parameters of the PID controller used for PID controlling the motor actuator.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param tolerance specifies the tolerance.
         * @return this parameter object.
         */
        public Parameters setPidParams(double kP, double kI, double kD, double tolerance)
        {
            return setPidParams(kP, kI, kD, 0.0, 0.0, tolerance);
        }   //setPidParams

        /**
         * This method sets the power to be used to zero calibrate the motor actuator.
         *
         * @param calPower specifies the motor power to use for zero calibration.
         * @return this parameter object.
         */
        public Parameters setZeroCalibratePower(double calPower)
        {
            this.calPower = calPower;
            return this;
        }   //setZeroCalibratePower

        /**
         * This method enables/disables the lower limit switch triggering a position reset.
         *
         * @param enabled specifies true to enable the lower limit switch trigger to reset position.
         * @return this parameter object.
         */
        public Parameters resetPositionOnLowerLimit(boolean enabled)
        {
            this.resetPosOnLowerLimit = enabled;
            return this;
        }   //resetPositionOnLowerLimit

        /**
         * This method sets the stall protection parameters of the motor actuator.
         *
         * @param stallMinPower specifies the minimum power applied to the motor before stall detection will kick in.
         * @param stallTolerance specifies the movement tolerance within which is still considered stalled.
         * @param stallTimeout specifies the minimum time the motor has to stall to trigger the stalled condition.
         * @param resetTimeout specifies the minimum time has to pass with no power applied to the motor to reset
         *                     stalled condition.
         * @return this parameter object.
         */
        public Parameters setStallProtectionParams(
            double stallMinPower, double stallTolerance, double stallTimeout, double resetTimeout)
        {
            this.stallMinPower = stallMinPower;
            this.stallTolerance = stallTolerance;
            this.stallTimeout = stallTimeout;
            this.resetTimeout = resetTimeout;
            return this;
        }   //setStallProtectionParams

        /**
         * This method sets the power compensation callback method. When specified, the power compensation method will
         * be called periodically to calculate the motor power to compensation for gravity.
         *
         * @param powerCompensation specifies the motor power compensation method.
         * @return this parameter object.
         */
        public Parameters setPowerCompensation(PowerCompensation powerCompensation)
        {
            this.powerCompensation = powerCompensation;
            return this;
        }   //setPowerCompensation

        /**
         * This method sets an array of preset positions for the motor actuator.
         *
         * @param tolerance specifies the preset tolerance.
         * @param posPresets specifies an array of preset positions in scaled unit.
         * @return this parameter object.
         */
        public Parameters setPosPresets(double tolerance, double... posPresets)
        {
            this.presetTolerance = tolerance;
            this.posPresets = posPresets;
            return this;
        }   //setPosPresets

    }   //class Parameters

    private final Parameters params;
    private final TrcDigitalInput lowerLimitSwitch;
    private final TrcDigitalInput upperLimitSwitch;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor1 specifies the primary motor in the actuator.
     * @param motor2 specifies the secondary motor in the actuator.
     * @param syncGain specifies the sync gain between the primary and secondary motors (i.e. Kp).
     * @param lowerLimitSwitch specifies the optional lower limit switch. Required only for auto zero calibration
     *        whenever it is active.
     * @param upperLimitSwitch specifies the optional upper limit switch.
     * @param params specifies the parameters for the PID actuator.
     */
    public TrcPidActuator(
        String instanceName, TrcMotor motor1, TrcMotor motor2, double syncGain, TrcDigitalInput lowerLimitSwitch,
        TrcDigitalInput upperLimitSwitch, Parameters params)
    {
        super(instanceName, motor1, motor2, syncGain, params.pidParams, params.useMotorCloseLoopControl,
              lowerLimitSwitch, params.calPower);
        this.lowerLimitSwitch = lowerLimitSwitch;
        this.upperLimitSwitch = upperLimitSwitch;
        this.params = params;

        if (lowerLimitSwitch != null && params.resetPosOnLowerLimit)
        {
            motor1.resetPositionOnDigitalInput(lowerLimitSwitch);
        }

        setPositionScaleAndOffset(params.scale, params.offset);

        if (params.stallMinPower != 0.0)
        {
            setStallProtection(params.stallMinPower, params.stallTolerance, params.stallTimeout, params.resetTimeout);
        }

        TrcPidController pidCtrl = getPidController();
        if (pidCtrl != null)
        {
            pidCtrl.setAbsoluteSetPoint(true);
            if (params.powerCompensation != null)
            {
                pidCtrl.getPidParameters().setPowerCompensation(params.powerCompensation);
            }
        }
    }   //TrcPidActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor1 specifies the primary motor in the actuator.
     * @param motor2 specifies the secondary motor in the actuator.
     * @param lowerLimitSwitch specifies the optional lower limit switch. Required only for auto zero calibration
     *        whenever it is active.
     * @param upperLimitSwitch specifies the optional upper limit switch.
     * @param params specifies the parameters for the PID actuator.
     */
    public TrcPidActuator(
        String instanceName, TrcMotor motor1, TrcMotor motor2, TrcDigitalInput lowerLimitSwitch,
        TrcDigitalInput upperLimitSwitch, Parameters params)
    {
        this(instanceName, motor1, motor2, 0.0, lowerLimitSwitch, upperLimitSwitch, params);
    }   //TrcPidActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param motor specifies the motor in the actuator.
     * @param lowerLimitSwitch specifies the optional lower limit switch. Required only for auto zero calibration
     *        whenever it is active.
     * @param upperLimitSwitch specifies the optional upper limit switch.
     * @param params specifies the parameters for the PID actuator.
     */
    public TrcPidActuator(
        String instanceName, TrcMotor motor, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch,
        Parameters params)
    {
        this(instanceName, motor, null, 0.0, lowerLimitSwitch, upperLimitSwitch, params);
    }   //TrcPidActuator

    /**
     * This method sets the position range limits of the motor actuator.
     *
     * @param minPos specifies the minimum position of the actuator in scaled unit.
     * @param maxPos specifies the maximum position of the actuator in scaled unit.
     * @return this parameter object.
     */
    public void setPositionRange(double minPos, double maxPos)
    {
        params.setPosRange(minPos, maxPos);
    }   //setPositionRange

    /**
     * This method enables/disables lower limit switch to trigger a position reset.
     *
     * @param enabled specifies true to enable lower limit switch to trigger a position reset, false to disable.
     */
    public void setResetPosOnLowerLimitEnabled(boolean enabled)
    {
        params.resetPositionOnLowerLimit(enabled);
    }   //setResetPosOnLowerLimitEnabled

    /**
     * This method checks if the lower limit switch is activated.
     *
     * @return true if the lower limit switch is activated, false otherwise.
     */
    public boolean isLowerLimitSwitchActive()
    {
        return lowerLimitSwitch != null && lowerLimitSwitch.isActive();
    }   //isLowerLimitSwitchActive

    /**
     * This method checks if the upper limit switch is activated.
     *
     * @return true if the upper limit switch is activated, false otherwise.
     */
    public boolean isUpperLimitSwitchActive()
    {
        return upperLimitSwitch != null && upperLimitSwitch.isActive();
    }   //isUpperLimitSwitchActive

    /**
     * This method runs the actuator with the specified power. Note that this method does not do PID control. To do
     * PID control, use setPidPower.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param power specifies the power to run the actuator.
     */
    @Override
    public void setPower(String owner, double power)
    {
        // Cancel previous PID operation such as setTarget if any.
        cancel();
        super.setPower(owner, power);
    }   //setPower

    /**
     * This method runs the actuator with the specified power. Note that this method does not do PID control. To do
     * PID control, use setPidPower.
     *
     * @param power specifies the power to run the actuator.
     */
    @Override
    public void setPower(double power)
    {
        setPower(null, power);
    }   //setPower

    /**
     * This method runs the actuator with the specified power. It will hold the current position even if power is zero.
     * Note that if position range is not set, it will throw a RuntimeException.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param power specifies the power to run the actuator.
     * @param hold specifies true to hold position when power is zero, false otherwise.
     * @throws RuntimeException if position range is not set.
     */
    public void setPidPower(String owner, double power, boolean hold)
    {
        if (params.minPos == 0.0 && params.maxPos == 0.0)
        {
            throw new RuntimeException("setPidPower requires position range to be set.");
        }

        setPowerWithinPosRange(owner, power, params.minPos, params.maxPos, hold);
    }   //setPidPower

    /**
     * This method runs the actuator with the specified power. It will hold the current position even if power is zero.
     * Note that if position range is not set, PID control will be disabled.
     *
     * @param power specifies the power to run the actuator.
     * @param hold specifies true to hold position when power is zero, false otherwise.
     */
    public void setPidPower(double power, boolean hold)
    {
        setPidPower(null, power, hold);
    }   //setPower

    /**
     * This method runs the actuator with the specified power. It will hold the current position even if power is zero.
     * Note that if position range is not set, PID control will be disabled.
     *
     * @param power specifies the power to run the actuator.
     */
    public void setPidPower(double power)
    {
        setPidPower(null, power, true);
    }   //setPower

    /**
     * This method checks if the preset index is within the preset table.
     *
     * @param index specifies the preset table index to check.
     * @return true if there is a preset table and the index is within the table.
     */
    public boolean validatePresetIndex(int index)
    {
        return params.posPresets != null && index >= 0 && index < params.posPresets.length;
    }   //validatePresetIndex

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the subsystem.
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param presetIndex specifies the index to the preset position array.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param powerLimit specifies the maximum power limit.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setPresetPosition(
        String owner, double delay, int presetIndex, boolean holdTarget, double powerLimit, TrcEvent event,
        double timeout)
    {
        if (validatePresetIndex(presetIndex))
        {
            setPosition(owner, delay, params.posPresets[presetIndex], holdTarget, powerLimit, event, timeout);
        }
    }   //setPresetPosition

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param preset specifies the index to the preset position array.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param powerLimit specifies the maximum power limit.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setPresetPosition(
        double delay, int preset, boolean holdTarget, double powerLimit, TrcEvent event, double timeout)
    {
        setPresetPosition(null, delay, preset, holdTarget, powerLimit, event, timeout);
    }   //setPresetPosition

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param preset specifies the index to the preset position array.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param powerLimit specifies the maximum power limit.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setPresetPosition(int preset, boolean holdTarget, double powerLimit, TrcEvent event, double timeout)
    {
        setPresetPosition(null, 0.0, preset, holdTarget, powerLimit, event, timeout);
    }   //setPresetPosition

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param preset specifies the index to the preset position array.
     * @param powerLimit specifies the maximum power limit.
     * @param event specifies the event to signal when done, can be null if not provided.
     */
    public void setPresetPosition(int preset, double powerLimit, TrcEvent event)
    {
        setPresetPosition(null, 0.0, preset, true, powerLimit, event, 0.0);
    }   //setPresetPosition

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param preset specifies the index to the preset position array.
     * @param event specifies the event to signal when done, can be null if not provided.
     */
    public void setPresetPosition(int preset, TrcEvent event)
    {
        setPresetPosition(null, 0.0, preset, true, 1.0, event, 0.0);
    }   //setPresetPosition

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param preset specifies the index to the preset position array.
     * @param powerLimit specifies the maximum power limit.
     */
    public void setPresetPosition(double delay, int preset, double powerLimit)
    {
        setPresetPosition(null, delay, preset, true, powerLimit, null, 0.0);
    }   //setPresetPosition

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param preset specifies the index to the preset position array.
     */
    public void setPresetPosition(double delay, int preset)
    {
        setPresetPosition(null, delay, preset, true, 1.0, null, 0.0);
    }   //setPresetPosition

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param preset specifies the index to the preset position array.
     * @param powerLimit specifies the maximum power limit.
     */
    public void setPresetPosition(int preset, double powerLimit)
    {
        setPresetPosition(null, 0.0, preset, true, powerLimit, null, 0.0);
    }   //setPresetPosition

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param preset specifies the index to the preset position array.
     */
    public void setPresetPosition(int preset)
    {
        setPresetPosition(null, 0.0, preset, true, 1.0, null, 0.0);
    }   //setPresetPosition

    /**
     * This method determines the next preset index up from the current position.
     *
     * @return next preset index up, -1 if there is no preset table.
     */
    public int nextPresetIndexUp()
    {
        int index = -1;

        if (params.posPresets != null)
        {
            double currPos = getPosition();

            for (int i = 0; i < params.posPresets.length; i++)
            {
                if (params.posPresets[i] > currPos)
                {
                    index = i;
                    if (Math.abs(currPos - params.posPresets[i]) <= params.presetTolerance)
                    {
                        index++;
                    }
                    break;
                }
            }

            if (index == -1)
            {
                index = params.posPresets.length - 1;
            }
        }

        return index;
    }   //nextPresetIndexUp

    /**
     * This method determines the next preset index down from the current position.
     *
     * @return next preset index down, -1 if there is no preset table.
     */
    public int nextPresetIndexDown()
    {
        int index = -1;

        if (params.posPresets != null)
        {
            double currPos = getPosition();

            for (int i = params.posPresets.length - 1; i >= 0; i--)
            {
                if (params.posPresets[i] < currPos)
                {
                    index = i;
                    if (Math.abs(currPos - params.posPresets[i]) <= params.presetTolerance)
                    {
                        index--;
                    }
                    break;
                }
            }

            if (index == -1)
            {
                index = 0;
            }
        }

        return index;
    }   //nextPresetIndexDown

    /**
     * This method sets the actuator to the next preset position up or down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is coompleted, can be null if no ownership
     *        is required.
     * @param presetUp specifies true to move to next preset up, false to move to next preset down.
     * @param powerLimit specifies the maximum power limit.
     */
    private void setNextPresetPosition(String owner, boolean presetUp, double powerLimit)
    {
        int index = presetUp? nextPresetIndexUp(): nextPresetIndexDown();

        if (index != -1)
        {
            setPresetPosition(owner, 0.0, index, true, powerLimit, null, 0.0);
        }
    }   //setNextPresetPosition

    /**
     * This method sets the actuator to the next preset position up from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is coompleted, can be null if no ownership
     *        is required.
     * @param powerLimit specifies the maximum power limit.
     */
    public void presetPositionUp(String owner, double powerLimit)
    {
        setNextPresetPosition(owner, true, powerLimit);
    }   //presetPositionUp

    /**
     * This method sets the actuator to the next preset position down from the current position.
     *
     * @param owner specifies the owner ID that will acquire ownership before setting the preset position and will
     *        automatically release ownership when the actuator movement is coompleted, can be null if no ownership
     *        is required.
     * @param powerLimit specifies the maximum power limit.
     */
    public void presetPositionDown(String owner, double powerLimit)
    {
        setNextPresetPosition(owner, false, powerLimit);
    }   //presetPositionDown

}   //class TrcPidActuator
