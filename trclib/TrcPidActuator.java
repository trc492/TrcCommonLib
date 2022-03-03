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
        public double minPos = 0.0, maxPos = 1.0;
        public double scale = 1.0, offset = 0.0;
        public TrcPidController.PidParameters pidParams;
        public double calPower = 0.3;
        public double stallMinPower = 0.0;
        public double stallTolerance = 0.0;
        public double stallTimeout = 0.0;
        public double resetTimeout = 0.0;
        public double[] posPresets = null;
        public PowerCompensation powerCompensation = null;

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
         * This method sets the scale and offset of the motor actuator. It allows the actuator to report real world
         * position units such as inches or degrees instead of sensor units.
         *
         * @param scale specifies the scale multiplier to convert position sensor unit to real world unit.
         * @param offset specifies the offset value to add to the scaled real world unit.
         * @return this parameter object.
         */
        public Parameters setScaleOffset(double scale, double offset)
        {
            this.scale = scale;
            this.offset = offset;
            return this;
        }   //setScaleOffset

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
         * @param tolerance specifies the tolerance.
         * @return this parameter object.
         */
        public Parameters setPidParams(double kP, double kI, double kD, double tolerance)
        {
            this.pidParams = new TrcPidController.PidParameters(kP, kI, kD, tolerance);
            return this;
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
         * This method sets an array of preset positions for the motor actuator.
         *
         * @param posPresets specifies an array of preset positions in scaled unit.
         * @return this parameter object.
         */
        public Parameters setPosPresets(double... posPresets)
        {
            this.posPresets = posPresets;
            return this;
        }   //setPosPresets

        /**
         * This method sets the power compensation callback method. When specified, the power compensation method will
         * be called periodically to calculate the motor power to compensation for gravity.
         *
         * @param powerCompensation specifies the motor power compensation method.
         * @return this parameter object.
         */
        public Parameters setPowerCompensation(TrcPidMotor.PowerCompensation powerCompensation)
        {
            this.powerCompensation = powerCompensation;
            return this;
        }   //setPowerCompensation

    }   //class Parameters

    private final Parameters params;
    private final TrcDigitalInput lowerLimitSwitch;
    private final TrcDigitalInput upperLimitSwitch;
    private boolean manualOverride = false;
    private int positionLevel = 0;

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
        super(instanceName, motor1, motor2, syncGain, params.pidParams, lowerLimitSwitch != null, params.calPower,
              params.powerCompensation);
        this.lowerLimitSwitch = lowerLimitSwitch;
        this.upperLimitSwitch = upperLimitSwitch;
        this.params = params;

        if (lowerLimitSwitch != null)
        {
            motor1.resetPositionOnDigitalInput(lowerLimitSwitch);
        }

        setPositionScale(params.scale, params.offset);

        if (params.stallMinPower != 0.0)
        {
            setStallProtection(params.stallMinPower, params.stallTolerance, params.stallTimeout, params.resetTimeout);
        }

        super.getPidController().setAbsoluteSetPoint(true);
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
     * This method returns the state of manual override.
     *
     * @return true if manual override is ON, false otherwise.
     */
    public boolean isManualOverride()
    {
        final String funcName = "isManualOverride";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", manualOverride);
        }

        return manualOverride;
    }   //isManualOverride

    /**
     * This method sets manual override mode. This is useful to override PID control of the actuator in situations
     * where the encoder is not zero calibrated or malfunctioning. Note that this only overrides the encoder but not
     * the limit switch. So if the lower limit switch is engaged, the actuator will not retract even though manual
     * override is true.
     *
     * @param manualOverride specifies true for manual override, false otherwise.
     */
    public void setManualOverride(boolean manualOverride)
    {
        final String funcName = "setManualOverride";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "manualOverrid=%s", Boolean.toString(manualOverride));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.manualOverride = manualOverride;
    }   //setManualOverride

    /**
     * This method runs the actuator with the specified power. It will hold the current position even if power is zero.
     * Note that if position range is not set, PID control will be disabled.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param power specifies the power to run the actuator.
     * @param hold specifies true to hold position when power is zero, false otherwise.
     */
    public void setPower(String owner, double power, boolean hold)
    {
        final String funcName = "setPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "power=%s", power);
        }

        if (manualOverride || params.minPos == 0.0 && params.maxPos == 0.0)
        {
            cancel();
            super.setPower(owner, power);
        }
        else
        {
            setPowerWithinPosRange(owner, power, params.minPos, params.maxPos, hold);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPower

    /**
     * This method runs the actuator with the specified power. It will hold the current position even if power is zero.
     * Note that if position range is not set, PID control will be disabled.
     *
     * @param power specifies the power to run the actuator.
     * @param hold specifies true to hold position when power is zero, false otherwise.
     */
    public void setPower(double power, boolean hold)
    {
        setPower(null, power, hold);
    }   //setPower

    /**
     * This method runs the actuator with the specified power. It will hold the current position even if power is zero.
     * Note that if position range is not set, PID control will be disabled.
     *
     * @param power specifies the power to run the actuator.
     */
    @Override
    public void setPower(double power)
    {
        setPower(null, power, false);
    }   //setPower

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param level specifies the index to the preset position array.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param callback specifies the callback handler to notify when done, can be null if not provided.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setLevel(
        String owner, double delay, int level, boolean holdTarget, TrcEvent event, TrcNotifier.Receiver callback,
        double timeout)
    {
        if (validateOwnership(owner))
        {
            if (params.posPresets != null)
            {
                if (level < 0)
                {
                    positionLevel = 0;
                }
                else if (level >= params.posPresets.length)
                {
                    positionLevel = params.posPresets.length - 1;
                }
                else
                {
                    positionLevel = level;
                }

                setTarget(delay, params.posPresets[positionLevel], holdTarget, event, callback, timeout);
            }
        }
    }   //setLevel

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param level specifies the index to the preset position array.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param callback specifies the callback handler to notify when done, can be null if not provided.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setLevel(
        double delay, int level, boolean holdTarget, TrcEvent event, TrcNotifier.Receiver callback, double timeout)
    {
        setLevel(null, delay, level, holdTarget, event, callback, timeout);
    }   //setLevel

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param level specifies the index to the preset position array.
     * @param holdTarget specifies true to hold target after PID operation is completed.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param callback specifies the callback handler to notify when done, can be null if not provided.
     * @param timeout specifies a timeout value in seconds. If the operation is not completed without the specified
     *                timeout, the operation will be canceled and the event will be signaled. If no timeout is
     *                specified, it should be set to zero.
     */
    public void setLevel(int level, boolean holdTarget, TrcEvent event, TrcNotifier.Receiver callback, double timeout)
    {
        setLevel(null, 0.0, level, holdTarget, event, callback, timeout);
    }   //setLevel

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param level specifies the index to the preset position array.
     * @param event specifies the event to signal when done, can be null if not provided.
     * @param callback specifies the callback handler to notify when done, can be null if not provided.
     */
    public void setLevel(int level, TrcEvent event, TrcNotifier.Receiver callback)
    {
        setLevel(null, 0.0, level, true, event, callback, 0.0);
    }   //setLevel

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param delay specifies delay time in seconds before setting position, can be zero if no delay.
     * @param level specifies the index to the preset position array.
     */
    public void setLevel(double delay, int level)
    {
        setLevel(null, delay, level, true, null, null, 0.0);
    }   //setLevel

    /**
     * This method sets the actuator to the specified preset position.
     *
     * @param level specifies the index to the preset position array.
     */
    public void setLevel(int level)
    {
        setLevel(null, 0.0, level, true, null, null, 0.0);
    }   //setLevel

    /**
     * This method sets the actuator to the next preset position up.
     */
    public void levelUp()
    {
        setLevel(positionLevel + 1);
    }   //levelUp

    /**
     * This method sets the actuator to the next preset position down.
     */
    public void levelDown()
    {
        setLevel(positionLevel - 1);
    }   //levelDown

    /**
     * This method returns the last preset level that is set to the actuator.
     *
     * @return last preset level set.
     */
    public int getLevel()
    {
        return positionLevel;
    }   //getLevel

}   //class TrcPidActuator
