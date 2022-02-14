/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.ArrayList;

import TrcCommonLib.trclib.TrcTaskMgr.TaskType;

/**
 * This class implements a platform independent motor controller. Typically, this class is extended by a platform
 * dependent motor controller class. Not all motor controllers are created equal. Some have more features than the
 * others. This class attempts to emulate some of the features in software. If the platform dependent motor controller
 * supports some features in hardware it should override the corresponding methods and call the hardware directly.
 * For some features that there is no software emulation, this class will throw an UnsupportedOperationException.
 * If the motor controller hardware support these features, the platform dependent class should override these methods
 * to provide the support in hardware.
 */
public abstract class TrcMotor implements TrcOdometrySensor, TrcExclusiveSubsystem
{
    protected static final String moduleName = "TrcMotor";
    protected static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When not enabled,
     * (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor will
     * stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     * @throws UnsupportedOperationException if hardware does not support it.
     */
    public abstract void setBrakeModeEnabled(boolean enabled);

    /**
     * This method sets the raw motor power. It is called by the Velocity Control task. If the subclass is
     * implementing its own native velocity control, it does not really need to do anything for this method.
     * But for completeness, it can just set the raw motor power in the motor controller.
     * Note: Do not call this method to set motor power, call set() instead. This method is intended to be called
     * internally by TrcMotor only.
     *
     * @param value specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    public abstract void setMotorPower(double value);

    /**
     * This method gets the last set power.
     *
     * @return the last setMotorPower value.
     */
    public abstract double getMotorPower();

    /**
     * This method resets the motor position sensor, typically an encoder.
     * Note: Do not call this method to reset motor position, call resetPosition() instead. This method is intended
     * to be called internally by TrcMotor only.
     *
     * @throws UnsupportedOperationException if hardware does not support it.
     */
    public abstract void resetMotorPosition();

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     * Note: Do not call this method to get motor position, call getPosition() instead. This method is intended
     * to be called internally by TrcMotor only.
     *
     * @return current motor position in raw sensor units.
     * @throws UnsupportedOperationException if hardware does not support it.
     */
    public abstract double getMotorPosition();

    /**
     * This method returns the motor velocity from the platform dependent motor hardware. If the hardware does
     * not support velocity info, it should throw an UnsupportedOperationException.
     * Note: Do not call this method to get motor velocity, call getVelocity() instead. This method is intended
     * to be called internally by TrcMotor only.
     *
     * @return current motor velocity in raw sensor units per sec.
     * @throws UnsupportedOperationException if hardware does not support it.
     */
    public abstract double getMotorVelocity();

    /**
     * This method returns the state of the reverse limit switch.
     * Note: Do not call this method to get limit switch state, call isLowerLimitSwitchActive() or
     * isUpperLimitSwitchActive() instead. This method is intended to be called internally by TrcMotor only.
     *
     * @return true if reverse limit switch is active, false otherwise.
     * @throws UnsupportedOperationException if hardware does not support it.
     */
    public abstract boolean isRevLimitSwitchActive();

    /**
     * This method returns the state of the forward limit switch.
     * Note: Do not call this method to get limit switch state, call isLowerLimitSwitchActive() or
     * isUpperLimitSwitchActive() instead. This method is intended to be called internally by TrcMotor only.
     *
     * @return true if forward limit switch is active, false otherwise.
     * @throws UnsupportedOperationException if hardware does not support it.
     */
    public abstract boolean isFwdLimitSwitchActive();

    //
    // Global objects.
    //
    private static final ArrayList<TrcMotor> odometryMotors = new ArrayList<>();
    protected static TrcElapsedTimer motorGetPosElapsedTimer;
    protected static TrcElapsedTimer motorSetElapsedTimer;
    private ArrayList<TrcMotor> followingMotorsList = new ArrayList<>();

    private final String instanceName;
    private final TrcOdometrySensor.Odometry odometry;
    private static TrcTaskMgr.TaskObject odometryTaskObj;
    private static TrcTaskMgr.TaskObject cleanupTaskObj;
    private final TrcTaskMgr.TaskObject velocityCtrlTaskObj;
    private final TrcTimer timer;

    private TrcEvent notifyEvent;
    private TrcDigitalInputTrigger digitalTrigger;
    private TrcSensorTrigger.DigitalTriggerHandler digitalTriggerHandler;

    protected boolean calibrating = false;

    private double motorDirSign = 1.0;
    private double posSensorSign = 1.0;
    private double zeroPosition = 0.0;
    private boolean limitSwitchesSwapped = false;
    private boolean softLowerLimitEnabled = false;
    private boolean softUpperLimitEnabled = false;
    private double softLowerLimit = 0.0;
    private double softUpperLimit = 0.0;

    private boolean odometryEnabled = false;

    protected double maxMotorVelocity = 0.0;
    private TrcPidController velocityPidCtrl = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcMotor(final String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ? globalTracer :
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.odometry = new TrcOdometrySensor.Odometry(this);

        if (odometryTaskObj == null)
        {
            //
            // Odometry task is a singleton that manages odometry of all motors.
            // This will be a STANDALONE_TASK so that it won't degrade the INPUT_TASK with long delay waiting for the
            // hardware. If we create individual task for each motor, moving them to STANDALONE_TASK will create too
            // many threads.
            //
            odometryTaskObj = TrcTaskMgr.createTask(moduleName + ".odometryTask", TrcMotor::odometryTask);
            cleanupTaskObj = TrcTaskMgr.createTask(instanceName + ".cleanupTask", this::cleanupTask);
            cleanupTaskObj.registerTask(TaskType.STOP_TASK);
        }
        velocityCtrlTaskObj = TrcTaskMgr.createTask(instanceName + ".velCtrlTask", this::velocityCtrlTask);
        timer = new TrcTimer("motorTimer." + instanceName);
    }   //TrcMotor

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    //
    // Platform-dependent subclasses should override these methods if they can be supported in hardware.
    //

    /**
     * This method inverts the motor direction.
     * Note: Should be overriden by platform-dependent subclasses if this is supported in hardware.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    public void setInverted(boolean inverted)
    {
        motorDirSign = inverted? -1.0: 1.0;
    }   //setInverted

    /**
     * This method returns the state of the motor controller direction.
     * Note: Should be overriden by platform-dependent subclasses if this is supported in hardware.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    public boolean isInverted()
    {
        return motorDirSign == -1.0;
    }   //isInverted

    /**
     * This method sets this motor to follow another motor. If the subclass is not capable of following another motor,
     * this method throws an UnsupportedOperationException.
     *
     * @throws UnsupportedOperationException if hardware does not support it.
     */
    public void followMotor(TrcMotor motor)
    {
        throw new UnsupportedOperationException("followMotor is not supported!");
    }   //followMotor

    /**
     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
     * situation.
     * Note: Should be overriden by platform-dependent subclasses if this is supported in hardware.
     *
     * @param inverted specifies true to invert position sensor direction, false otherwise.
     */
    public void setPositionSensorInverted(boolean inverted)
    {
        posSensorSign = inverted? -1.0: 1.0;
    }   //setPositionSensorInverted

    /**
     * This method returns the state of the position sensor direction.
     * Note: Should be overriden by platform-dependent subclasses if this is supported in hardware.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    public boolean isPositionSensorInverted()
    {
        return posSensorSign == -1.0;
    }   //isPositionSensorInverted

    /**
     * This method checks if the motor controller is connected to the robot. Note that this does NOT guarantee the
     * connection status of the motor to the motor controller. If detecting the motor presence is impossible (i.e. the
     * motor controller is connected via PWM) this method will always return true.
     *
     * @return True if the motor is connected or if it's impossible to know, false otherwise.
     */
    public boolean isConnected()
    {
        return true;
    }   //isConnected

    //
    // TrcMotor APIs.
    //

    /**
     * This method adds the given motor to the list that will follow this motor.
     *
     * @param motor specifies the motor that will follow this motor.
     */
    public void addFollowingMotor(TrcMotor motor)
    {
        followingMotorsList.add(motor);
    }   //setFollowingMotor

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *              ownership aware.
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     */
    public void set(String owner, double value)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "owner=%s,value=%f", owner, value);
        }

        if (validateOwnership(owner))
        {
            value = TrcUtil.clipRange(value);

            if ((softLowerLimitEnabled || softUpperLimitEnabled) && value != 0.0)
            {
                double currPos = getPosition();

                if (softLowerLimitEnabled && value < 0.0 && currPos <= softLowerLimit ||
                    softUpperLimitEnabled && value > 0.0 && currPos >= softUpperLimit)
                {
                    value = 0.0;
                }
            }

            calibrating = false;

            if (motorSetElapsedTimer != null) motorSetElapsedTimer.recordStartTime();
            setMotorPower(value);
            if (followingMotorsList != null)
            {
                for (TrcMotor motor: followingMotorsList)
                {
                    motor.setMotorPower(value);
                }
            }
            // if (velocityPidCtrl != null)
            // {
            //     // TO-DO: rethink velocity control mode. Leverage hardware if available.
            //     velocityPidCtrl.setTarget(value);
            // }
            // else
            // {
            //     setMotorPower(value);
            // }
            if (motorSetElapsedTimer != null) motorSetElapsedTimer.recordEndTime();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (value=%f)", value);
        }
    }   //set

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode.
     *
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     */
    public void set(double value)
    {
        set(null, value);
    }   //set

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.The value can be power or velocity percentage depending on whether the motor controller is in
     * power mode or velocity mode.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *              ownership aware.
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param time specifies the time period in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void set(String owner, double value, double time, TrcEvent event)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API, "owner=%s,value=%f,time=%.3f,event=%s",
                owner, value, time, event);
        }

        if (validateOwnership(owner))
        {
            timer.cancel();     // cancel old unexpired timer if any.
            if (value != 0.0 && time > 0.0)
            {
                notifyEvent = event;
                timer.set(time, this::notify);
            }
            set(null, value);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //set

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.The value can be power or velocity percentage depending on whether the motor controller is in
     * power mode or velocity mode.
     *
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param time specifies the time period in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void set(double value, double time, TrcEvent event)
    {
        set(null, value, time, event);
    }   //set

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.The value can be power or velocity percentage depending on whether the motor controller is in
     * power mode or velocity mode.
     *
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param time specifies the time period in seconds to have power set.
     */
    public void set(double value, double time)
    {
        set(null, value, time, null);
    }   //set

    /**
     * This method is called when the motor power set timer has expired. It will turn the motor off.
     *
     * @param context specifies the timer object (not used).
     */
    private void notify(Object context)
    {
        set(0.0);
        if (notifyEvent != null)
        {
            notifyEvent.signal();
            notifyEvent = null;
        }
    }   //notify

    /**
     * This method resets the motor position sensor, typically an encoder. This method emulates a reset for a
     * potentiometer by doing a soft reset.
     *
     * @param hardware specifies true for resetting hardware position, false for resetting software position.
     */
    public void resetPosition(boolean hardware)
    {
        final String funcName = "resetPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "hardware=%s", hardware);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (hardware)
        {
            // Call platform-dependent subclass to reset the position sensor hardware.
            resetMotorPosition();
            zeroPosition = 0.0;
        }
        else
        {
            // Call platform-dependent subclass to read current position as the zero position.
            zeroPosition = getMotorPosition();
        }
    }   //resetPosition

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position in sensor units.
     */
    public double getPosition()
    {
        final String funcName = "getPosition";
        double currPos;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (odometryEnabled)
        {
            synchronized (odometry)
            {
                // Don't read from motor hardware directly, get it from odometry instead.
                currPos = odometry.currPos;
            }
        }
        else
        {
            currPos = (getMotorPosition() - zeroPosition)*posSensorSign;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.0f", currPos);
        }

        return currPos;
    }   //getPosition

    /**
     * This method returns the velocity of the motor rotation. It keeps track of the rotation velocity by using a
     * periodic task to monitor the position sensor value. If the motor controller has hardware monitoring velocity,
     * it can override this method and access the hardware instead. However, accessing hardware may impact
     * performance because it may involve initiating USB/CAN/I2C bus cycles. Therefore, it may be beneficial to
     * just let the the periodic task calculate the velocity here.
     *
     * @return motor velocity in sensor units per second.
     */
    public double getVelocity()
    {
        final String funcName = "getVelocity";
        final double velocity;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (odometryEnabled)
        {
            synchronized (odometry)
            {
                // Don't read from motor hardware directly, get it from odometry instead.
                velocity = odometry.velocity;
            }
        }
        else
        {
            velocity = getMotorVelocity()*posSensorSign;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.0f", velocity);
        }

        return velocity;
    }   //getVelocity

    /**
     * This method swaps the forward and reverse limit switches. By default, the lower limit switch is associated
     * with the reverse limit switch and the upper limit switch is associated with the forward limit switch. This
     * method will swap the association.
     *
     * @param swapped specifies true to swap the limit switches, false otherwise.
     */
    public void setLimitSwitchesSwapped(boolean swapped)
    {
        final String funcName = "setLimitSwitchesSwapped";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "swapped=%s", swapped);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        limitSwitchesSwapped = swapped;
    }   //setLimitSwitchesSwapped

    /**
     * This method returns the state of the lower limit switch.
     *
     * @return true if lower limit switch is active, false otherwise.
     */
    public boolean isLowerLimitSwitchActive()
    {
        final String funcName = "isLowerLimitSwitchActive";
        boolean isActive = limitSwitchesSwapped? isFwdLimitSwitchActive(): isRevLimitSwitchActive();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", isActive);
        }

        return isActive;
    }   //isLowerLimitSwitchActive

    /**
     * This method returns the state of the upper limit switch.
     *
     * @return true if upper limit switch is active, false otherwise.
     */
    public boolean isUpperLimitSwitchActive()
    {
        final String funcName = "isUpperLimitSwitchActive";
        boolean isActive = limitSwitchesSwapped? isRevLimitSwitchActive(): isFwdLimitSwitchActive();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", isActive);
        }

        return isActive;
    }   //isUpperLimitSwitchActive

    /**
     * This method enables/disables soft limit switches. This is intended to be called as part of motor
     * initialization. Therefore, it is not designed to be ownership-aware.
     *
     * @param lowerLimitEnabled specifies true to enable lower soft limit switch, false otherwise.
     * @param upperLimitEnabled specifies true to enable upper soft limit switch, false otherwise.
     */
    public void setSoftLimitEnabled(boolean lowerLimitEnabled, boolean upperLimitEnabled)
    {
        final String funcName = "setSoftLimitEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API, "lowerEnabled=%s,upperEnabled=%s",
                lowerLimitEnabled, upperLimitEnabled);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        softLowerLimitEnabled = lowerLimitEnabled;
        softUpperLimitEnabled = upperLimitEnabled;
    }   //setSoftLimitEnabled

    /**
     * This method sets the lower soft limit. This is intended to be called as part of motor initialization.
     * Therefore, it is not designed to be ownership-aware.
     *
     * @param lowerLimit specifies the position of the lower limit.
     */
    public void setSoftLowerLimit(double lowerLimit)
    {
        final String funcName = "setSoftLowerLimit";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "lowerLimit=%.1f", lowerLimit);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        softLowerLimit = lowerLimit;
    }   //setSoftLowerLimit

    /**
     * This method sets the upper soft limit. This is intended to be called as part of motor initialization.
     * Therefore, it is not designed to be ownership-aware.
     *
     * @param upperLimit specifies the position of the upper limit.
     */
    public void setSoftUpperLimit(double upperLimit)
    {
        final String funcName = "setSoftUpperLimit";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "upperLimit=%.1f", upperLimit);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        softUpperLimit = upperLimit;
    }   //setSoftUpperLimit

    /**
     * This method sets the lower and upper soft limits. This is intended to be called as part of motor initialization.
     * Therefore, it is not designed to be ownership-aware.
     *
     * @param lowerLimit specifies the position of the lower limit.
     * @param upperLimit specifies the position of the upper limit.
     */
    public void setSoftLimits(double lowerLimit, double upperLimit)
    {
        setSoftLowerLimit(lowerLimit);
        setSoftUpperLimit(upperLimit);
    }   //setSoftLimits

    /**
     * This method creates a digital trigger on the given digital input sensor. It resets the position sensor
     * reading when the digital input is triggered. This is intended to be called as part of motor initialization.
     * Therefore, it is not designed to be ownership-aware.
     *
     * @param digitalInput   specifies the digital input sensor that will trigger a position reset.
     * @param triggerHandler specifies an event callback if the trigger occurred, null if none specified.
     */
    public void resetPositionOnDigitalInput(
        TrcDigitalInput digitalInput, TrcSensorTrigger.DigitalTriggerHandler triggerHandler)
    {
        final String funcName = "resetPositionOnDigitalInput";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "digitalInput=%s", digitalInput);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        digitalTrigger = new TrcDigitalInputTrigger(instanceName, digitalInput, this::triggerEvent);
        digitalTriggerHandler = triggerHandler;
        digitalTrigger.setEnabled(true);
    }   //resetPositionOnDigitalInput

    /**
     * This method creates a digital trigger on the given digital input sensor. It resets the position sensor
     * reading when the digital input is triggered.
     *
     * @param digitalInput specifies the digital input sensor that will trigger a position reset.
     */
    public void resetPositionOnDigitalInput(TrcDigitalInput digitalInput)
    {
        resetPositionOnDigitalInput(digitalInput, null);
    }   //resetPositionOnDigitalInput

    /**
     * This method is called when the digital input device has changed state.
     *
     * @param active specifies true if the digital device state is active, false otherwise.
     */
    private void triggerEvent(boolean active)
    {
        final String funcName = "triggerEvent";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.CALLBK, "trigger=%s,active=%s", digitalTrigger, active);
        }

        globalTracer.traceInfo(
            "triggerEvent", "TrcMotor encoder reset! motor=%s,pos=%.2f", instanceName, getMotorPosition());

        if (calibrating)
        {
            //
            // set(0.0) will turn off calibration mode.
            //
            set(0.0);
            calibrating = false;
        }

        resetPosition(false);

        if (digitalTriggerHandler != null)
        {
            digitalTriggerHandler.digitalTriggerEvent(active);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }
    }   //triggerEvent

    /**
     * This method performs a zero calibration on the motor by slowly turning in reverse. When the lower limit switch
     * is triggered, it stops the motor and resets the motor position.
     *
     * @param calibratePower specifies the motor power to perform zero calibration (must be positive value).
     */
    public void zeroCalibrate(double calibratePower)
    {
        //
        // Only do this if there is a digital trigger.
        //
        if (digitalTrigger != null && digitalTrigger.isEnabled())
        {
            set(-Math.abs(calibratePower));
            calibrating = true;
        }
    }   //zeroCalibrate

    //
    // Odometry.
    //

    /**
     * This method clears the list of motors that register for odometry monitoring. This method should only be called
     * by the task scheduler.
     *
     * @param removeOdometryTask specifies true to also remove the odometry task object, false to leave it alone.
     *                           This is mainly for FTC, FRC should always set this to false.
     */
    public static void clearOdometryMotorsList(boolean removeOdometryTask)
    {
        synchronized (odometryMotors)
        {
            if (odometryMotors.size() > 0)
            {
                odometryMotors.clear();
                odometryTaskObj.unregisterTask(TaskType.INPUT_TASK);
            }
            //
            // We must clear the task object because FTC opmode stuck around even after it has ended. So the task
            // object would have a stale odometryTask if we run the opmode again.
            //
            if (removeOdometryTask)
            {
                odometryTaskObj = null;
            }
        }
    }   //clearOdometryMotorsList

    /**
     * This method returns the number of motors in the list registered for odometry monitoring.
     *
     * @return number of motors in the list.
     */
    public static int getNumOdometryMotors()
    {
        int numMotors;

        synchronized (odometryMotors)
        {
            numMotors = odometryMotors.size();
        }

        return numMotors;
    }   //getNumOdometryMotors

    /**
     * This method enables/disables the task that monitors the motor odometry. Since odometry task takes up CPU cycle,
     * it should not be enabled if the user doesn't need motor odometry info.
     *
     * @param enabled specifies true to enable odometry task, disable otherwise.
     */
    public void setOdometryEnabled(boolean enabled)
    {
        final String funcName = "setOdometryEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", enabled);
        }

        if (enabled)
        {
            resetOdometry(false);
            synchronized (odometryMotors)
            {
                //
                // Add only if this motor is not already on the list.
                //
                if (!odometryMotors.contains(this))
                {
                    odometryMotors.add(this);
                    if (odometryMotors.size() == 1)
                    {
                        //
                        // We are the first one on the list, start the task.
                        //
                        odometryTaskObj.registerTask(TaskType.STANDALONE_TASK);
                    }
                }
            }
        }
        else
        {
            synchronized (odometryMotors)
            {
                odometryMotors.remove(this);
                if (odometryMotors.isEmpty())
                {
                    //
                    // We were the only one on the list, stop the task.
                    //
                    odometryTaskObj.unregisterTask(TaskType.STANDALONE_TASK);
                }
            }
        }
        odometryEnabled = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setOdometryEnabled

    /**
     * This method checks if the odometry of this motor is enabled.
     *
     * @return true if odometry of this motor is enabled, false if disabled.
     */
    public boolean isOdometryEnabled()
    {
        return odometryEnabled;
    }   //isOdometryEnabled

    //
    // Implements TrcOdometrySensor interfaces.
    //

    /**
     * This method resets the odometry data and sensor.
     *
     * @param resetHardware specifies true to do a hardware reset, false to do a software reset. Hardware reset may
     *                      require some time to complete and will block this method from returning until finish.
     */
    @Override
    public void resetOdometry(boolean resetHardware)
    {
        synchronized (odometry)
        {
            resetPosition(resetHardware);
            odometry.prevTimestamp = odometry.currTimestamp = TrcUtil.getCurrentTime();
            odometry.prevPos = odometry.currPos = 0.0;
            odometry.velocity = 0.0;
        }
    }   //resetOdometry

    /**
     * This method returns a copy of the odometry data of the specified axis. It must be a copy so it won't change while
     * the caller is accessing the data fields.
     *
     * @param axisIndex specifies the axis index if it is a multi-axes sensor, 0 if it is a single axis sensor (not used).
     * @return a copy of the odometry data of the specified axis.
     */
    @Override
    public Odometry getOdometry(int axisIndex)
    {
        synchronized (odometry)
        {
            if (!odometryEnabled)
            {
                throw new RuntimeException("Motor odometry is not enabled.");
            }

            return odometry.clone();
        }
    }   //getOdometry

    /**
     * This method is called periodically to update motor odometry data. Odometry data includes position and velocity
     * data. By using this task to update odometry at a periodic rate, it allows robot code to obtain odometry data
     * from the cached data maintained by this task instead of repeatedly reading it directly from the motor
     * controller which may impact performance because it may involve initiating USB/CAN/I2C bus cycles. So even
     * though some motor controller hardware may keep track of its own velocity, it may be beneficial to just let the
     * odometry task to calculate it.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is running.
     */
    private static void odometryTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = moduleName + ".odometryTask";

        if (debugEnabled)
        {
            globalTracer.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        synchronized (odometryMotors)
        {
            for (TrcMotor motor : odometryMotors)
            {
                synchronized (motor.odometry)
                {
                    motor.odometry.prevTimestamp = motor.odometry.currTimestamp;
                    motor.odometry.prevPos = motor.odometry.currPos;
                    motor.odometry.currTimestamp = TrcUtil.getCurrentTime();

                    if (motorGetPosElapsedTimer != null) motorGetPosElapsedTimer.recordStartTime();
                    motor.odometry.currPos = (motor.getMotorPosition() - motor.zeroPosition)*motor.posSensorSign;
                    if (motorGetPosElapsedTimer != null) motorGetPosElapsedTimer.recordEndTime();

                    double low = Math.abs(motor.odometry.prevPos);
                    double high = Math.abs(motor.odometry.currPos);

                    if (low > high)
                    {
                        double temp = high;
                        high = low;
                        low = temp;
                    }

                    // To be spurious, motor must jump 10000+ units, and change by 8+ orders of magnitude
                    // log10(high)-log10(low) gives change in order of magnitude
                    // use log rules, equal to log10(high/low) >= 8
                    // change of base, log2(high/low)/log2(10) >= 8
                    // log2(high/low) >= 26.6ish
                    // Math.getExponent() is equal to floor(log2())
                    if (high - low > 10000)
                    {
                        low = Math.max(low, 1);
                        if (Math.getExponent(high / low) >= 27)
                        {
                            globalTracer.traceWarn(
                                funcName, "WARNING: Spurious encoder detected on motor %s! odometry=%s", motor, motor.odometry);
                            // Throw away spurious data and use previous data instead.
                            motor.odometry.currPos = motor.odometry.prevPos;
                        }
                    }

                    try
                    {
                        motor.odometry.velocity = motor.getMotorVelocity()*motor.posSensorSign;
                    }
                    catch (UnsupportedOperationException e)
                    {
                        //
                        // It doesn't support velocity data so calculate it ourselves.
                        //
                        double timeDelta = motor.odometry.currTimestamp - motor.odometry.prevTimestamp;
                        motor.odometry.velocity =
                            timeDelta == 0.0 ? 0.0 : (motor.odometry.currPos - motor.odometry.prevPos) / timeDelta;
                    }

                    if (debugEnabled)
                    {
                        globalTracer.traceInfo(funcName, "Odometry: %s=(%s)", motor, motor.odometry);
                    }
                }
            }
        }

        if (debugEnabled)
        {
            globalTracer.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //odometryTask

    /**
     * This method is called before the runMode is about to stop so we can disable odometry.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is running.
     */
    private void cleanupTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "cleanupTask";

        if (debugEnabled)
        {
            globalTracer.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        clearOdometryMotorsList(false);

        if (debugEnabled)
        {
            globalTracer.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //cleanupTask

    //
    // Velocity control mode.
    //

    /**
     * This method sets the motor controller to velocity mode with the specified maximum velocity.
     *
     * @param maxVelocity     specifies the maximum velocity the motor can run, in sensor units per second.
     * @param pidCoefficients specifies the PID coefficients to use to compute a desired torque value for the motor.
     *                        E.g. these coefficients go from velocity error percent to desired stall torque percent.
     */
    public synchronized void enableVelocityMode(double maxVelocity, TrcPidController.PidCoefficients pidCoefficients)
    {
        final String funcName = "enableVelocityMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "maxVel=%f,pidCoef=%s", maxVelocity,
                pidCoefficients.toString());
        }

        if (pidCoefficients == null)
        {
            throw new IllegalArgumentException("PidCoefficient must not be null.");
        }

        if (!odometryEnabled)
        {
            throw new RuntimeException("Motor odometry must be enabled to use velocity mode.");
        }

        this.maxMotorVelocity = maxVelocity;
        velocityPidCtrl = new TrcPidController(instanceName + ".velocityCtrl", pidCoefficients, 1.0,
            this::getNormalizedVelocity);
        velocityPidCtrl.setAbsoluteSetPoint(true);

        velocityCtrlTaskObj.registerTask(TaskType.OUTPUT_TASK);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //enableVelocityMode

    /**
     * This method disables velocity mode returning it to power mode.
     */
    public synchronized void disableVelocityMode()
    {
        final String funcName = "disableVelocityMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (velocityPidCtrl != null)
        {
            velocityCtrlTaskObj.unregisterTask(TaskType.OUTPUT_TASK);
            velocityPidCtrl = null;
        }
    }   //disableVelocityMode

    /**
     * This method returns the motor velocity normalized to the range of -1.0 to 1.0, essentially a percentage of the
     * maximum motor velocity.
     *
     * @return normalized motor velocity.
     */
    private double getNormalizedVelocity()
    {
        final String funcName = "getNormalizedVelocity";
        double normalizedVel = maxMotorVelocity != 0.0 ? getVelocity() / maxMotorVelocity : 0.0;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", normalizedVel);
        }

        return normalizedVel;
    }   //getNormalizedVelocity

    /**
     * This method overrides the motorSpeedTask in TrcMotor which is called periodically to calculate he speed of
     * the motor. In addition to calculate the motor speed, it also calculates and sets the motor power required
     * to maintain the set speed if speed control mode is enabled.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is running.
     */
    private synchronized void velocityCtrlTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "velocityCtrlTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (velocityPidCtrl != null)
        {
            double desiredStallTorquePercentage = velocityPidCtrl.getOutput();
            double motorPower = transformTorqueToMotorPower(desiredStallTorquePercentage);

            setMotorPower(motorPower);
            if (debugEnabled)
            {
                dbgTrace.traceInfo(instanceName,
                    "targetSpeed=%.2f, currSpeed=%.2f, desiredStallTorque=%.2f, motorPower=%.2f",
                    velocityPidCtrl.getTarget(), getVelocity(), desiredStallTorquePercentage, motorPower);
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //velocityCtrlTask

    /**
     * Transforms the desired percentage of motor stall torque to the motor duty cycle (aka power)
     * that would give us that amount of torque at the current motor speed.
     *
     * @param desiredStallTorquePercentage specifies the desired percentage of motor torque to receive in percent of
     *                                     motor stall torque.
     * @return power percentage to apply to the motor to generate the desired torque (to the best ability of the motor).
     */
    private double transformTorqueToMotorPower(double desiredStallTorquePercentage)
    {
        final String funcName = "transformTorqueToMotorPower";
        double power;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "torque=%f", desiredStallTorquePercentage);
        }
        //
        // Leverage motor curve information to linearize torque output across varying RPM
        // as best we can. We know that max torque is available at 0 RPM and zero torque is
        // available at max RPM - use that relationship to proportionately boost voltage output
        // as motor speed increases.
        //
        final double currSpeedSensorUnitPerSec = Math.abs(getVelocity());
        final double currNormalizedSpeed = currSpeedSensorUnitPerSec / maxMotorVelocity;

        // Max torque percentage declines proportionally to motor speed.
        final double percentMaxTorqueAvailable = 1 - currNormalizedSpeed;

        if (percentMaxTorqueAvailable > 0)
        {
            power = desiredStallTorquePercentage / percentMaxTorqueAvailable;
        }
        else
        {
            // When we exceed max motor speed (and the correction factor is undefined), apply 100% voltage.
            power = Math.signum(desiredStallTorquePercentage);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%f", power);
        }

        return power;
    }   //transformTorqueToMotorPower

    //
    // Performance monitoring.
    //

    /**
     * This method enables/disables the elapsed timers for performance monitoring.
     *
     * @param enabled specifies true to enable elapsed timers, false to disable.
     */
    public static void setElapsedTimerEnabled(boolean enabled)
    {
        if (enabled)
        {
            if (motorGetPosElapsedTimer == null)
            {
                motorGetPosElapsedTimer = new TrcElapsedTimer(moduleName + ".getPos", 2.0);
            }

            if (motorSetElapsedTimer == null)
            {
                motorSetElapsedTimer = new TrcElapsedTimer(moduleName + ".set", 2.0);
            }
        }
        else
        {
            motorGetPosElapsedTimer = null;
            motorSetElapsedTimer = null;
        }
    }   //setElapsedTimerEnabled

    /**
     * This method prints the elapsed time info using the given tracer.
     *
     * @param tracer specifies the tracer to use for printing elapsed time info.
     */
    public static void printElapsedTime(TrcDbgTrace tracer)
    {
        if (motorGetPosElapsedTimer != null)
        {
            motorGetPosElapsedTimer.printElapsedTime(tracer);
        }

        if (motorSetElapsedTimer != null)
        {
            motorSetElapsedTimer.printElapsedTime(tracer);
        }
    }   //printElapsedTime

}   //class TrcMotor
