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
import java.util.concurrent.atomic.AtomicBoolean;

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
public abstract class TrcMotor implements TrcMotorController, TrcOdometrySensor, TrcExclusiveSubsystem
{
    private static final String moduleName = "TrcMotor";
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;

    public enum TriggerMode
    {
        OnActive,
        OnInactive,
        OnBoth
    }   //enum TriggerMode

    private enum MotorState
    {
        done,
        doDelay,
        doDuration
    }   //enum MotorState

    //
    // Global objects.
    //
    private static final ArrayList<TrcMotor> odometryMotors = new ArrayList<>();

    private static TrcElapsedTimer motorGetPosElapsedTimer;
    private static TrcElapsedTimer motorSetElapsedTimer;
    private final ArrayList<TrcMotor> followingMotorsList = new ArrayList<>();

    protected final String instanceName;
    private final TrcOdometrySensor.Odometry odometry;
    private final TrcTimer timer;
    private MotorState motorState = MotorState.done;
    private double motorValue;
    private double duration;
    private TrcEvent notifyEvent;
    private static TrcTaskMgr.TaskObject odometryTaskObj;
    private final TrcTaskMgr.TaskObject velocityCtrlTaskObj;
    private boolean velocityControlEnabled = false;

    private TrcTriggerDigitalInput digitalTrigger;
    private TriggerMode triggerMode;
    private TrcEvent triggerCallbackEvent;
    private AtomicBoolean triggerCallbackContext;

    private boolean calibrating = false;

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
        this.instanceName = instanceName;
        this.odometry = new TrcOdometrySensor.Odometry(this);
        timer = new TrcTimer(instanceName);

        if (odometryTaskObj == null)
        {
            //
            // Odometry task is a singleton that manages odometry of all motors.
            // This will be a STANDALONE_TASK so that it won't degrade the INPUT_TASK with long delay waiting for the
            // hardware. If we create individual task for each motor, moving them to STANDALONE_TASK will create too
            // many threads.
            //
            odometryTaskObj = TrcTaskMgr.createTask(moduleName + ".odometryTask", TrcMotor::odometryTask);
            TrcTaskMgr.TaskObject cleanupTaskObj = TrcTaskMgr.createTask(
                instanceName + ".cleanupTask", this::cleanupTask);
            cleanupTaskObj.registerTask(TaskType.STOP_TASK);
        }
        velocityCtrlTaskObj = TrcTaskMgr.createTask(instanceName + ".velCtrlTask", this::velocityCtrlTask);
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

    /**
     * This method adds the given motor to the list that will follow this motor. It should only be called by the
     * given motor to add it to the follower list of the motor it wants to follow.
     *
     * @param motor specifies the motor that will follow this motor.
     */
    public void addFollowingMotor(TrcMotor motor)
    {
        if (!followingMotorsList.contains(motor))
        {
            followingMotorsList.add(motor);
        }
    }   //addFollowingMotor

    /**
     * This method calls the motor subclass to set the motor output value. The value can be power or velocity
     * percentage depending on whether the motor controller is in power mode or velocity mode.
     *
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     */
    private void setMotorValue(double value)
    {
        if (motorSetElapsedTimer != null) motorSetElapsedTimer.recordStartTime();
        //
        // If subclass supports velocity control, it would have overridden enableVelocityMode so that velocityPidCtrl
        // won't be created. In that case, we will do software velocity control using software PID controller.
        //
        if (velocityPidCtrl != null)
        {
            velocityPidCtrl.setTarget(value);
        }
        else if (velocityControlEnabled)
        {
            // Normalize motor velocity to percentage max velocity in the range of -1.0 to 1.0.
            value = TrcUtil.clipRange(value/maxMotorVelocity);
            setMotorVelocity(value);
            for (TrcMotor motor: followingMotorsList)
            {
                motor.setMotorVelocity(value);
            }
        }
        else
        {
            setMotorPower(value);
            for (TrcMotor motor: followingMotorsList)
            {
                motor.setMotorPower(value);
            }
        }

        if (motorSetElapsedTimer != null) motorSetElapsedTimer.recordEndTime();
    }   //setMotorValue

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode. Optionally, you can specify a delay before running
     * the motor and a duration for which the motor will be turned off afterwards.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *              ownership aware.
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay.
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *                 turning off.
     * @param event specifies the event to signal when the motor operation is completed
     */
    public void set(String owner, double delay, double value, double duration, TrcEvent event)
    {
        final String funcName = "set";

        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "%s.%s: owner=%s,delay=%.3f,value=%f,duration=%.3f",
                moduleName, instanceName, owner, delay, value, duration);
        }

        if (validateOwnership(owner))
        {
            // Apply soft position limits if any.
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
            // Cancel the timer from the previous operation if there is one.
            timer.cancel();
            this.motorValue = value;
            if (delay > 0.0)
            {
                // The motor may be spinning, let's stop it.
                stopMotor();
                motorState = MotorState.doDelay;
                this.duration = duration;
                this.notifyEvent = event;
                timer.set(delay, this::motorEventHandler);
            }
            else
            {
                setMotorValue(value);
                if (duration > 0.0)
                {
                    motorState = MotorState.doDuration;
                    this.duration = duration;
                    this.notifyEvent = event;
                    timer.set(duration, this::motorEventHandler);
                }
                else
                {
                    motorState = MotorState.done;
                }
            }
        }
    }   //set

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode.
     *
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *                 turning off.
     * @param event specifies the event to signal when the motor operation is completed
     */
    public void set(double delay, double value, double duration, TrcEvent event)
    {
        set(null, delay, value, duration, event);
    }   //set

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode.
     *
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *                 turning off.
     * @param event specifies the event to signal when the motor operation is completed
     */
    public void set(double value, double duration, TrcEvent event)
    {
        set(null, 0.0, value, duration, event);
    }   //set

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode.
     *
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *                 turning off.
     */
    public void set(double delay, double value, double duration)
    {
        set(null, delay, value, duration, null);
    }   //set

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode.
     *
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *                 turning off.
     */
    public void set(double value, double duration)
    {
        set(null, 0.0, value, duration, null);
    }   //set

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode.
     *
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     */
    public void set(double value)
    {
        set(null, 0.0, value, 0.0, null);
    }   //set

    /**
     * This method is called when the motor power set timer has expired. It will turn the motor off.
     *
     * @param context specifies the timer object (not used).
     */
    private void motorEventHandler(Object context)
    {
        if (motorState == MotorState.doDelay)
        {
            // The delay timere has expired, so set the motor power now.
            setMotorValue(motorValue);
            if (duration > 0.0)
            {
                // We have set a duration, so set up a timer for it.
                motorState = MotorState.doDuration;
                timer.set(duration, this::motorEventHandler);
            }
            else
            {
                // We were setting the motor power indefinitely, so we are done.
                motorState = MotorState.done;
            }
        }
        else if (motorState == MotorState.doDuration)
        {
            // The duration timer has expired, turn everything off and clean up.
            duration = 0.0;
            setMotorValue(0.0);
            motorState = MotorState.done;
        }

        if (motorState == MotorState.done && notifyEvent != null)
        {
            notifyEvent.signal();
            notifyEvent = null;
        }
    }   //motorEventHandler

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
            globalTracer.traceInfo(funcName, "%s.%s: hardware=%s", moduleName, instanceName, hardware);
        }

        if (hardware)
        {
            // Call platform-dependent subclass to reset the position sensor hardware.
            resetMotorPosition();
        }

        // Call platform-dependent subclass to read current position as the zero position.
        // Note: the above resetMotorPosition call may have timed out and not resetting the hardware. We will still
        // do soft reset as a safety measure.
        zeroPosition = getMotorPosition();
    }   //resetPosition

    /**
     * This method resets the motor position sensor, typically an encoder. This method emulates a reset for a
     * potentiometer by doing a soft reset.
     */
    public void resetPosition()
    {
        resetPosition(false);
    }   //resetPosition

    /**
     * This method returns the motor position by reading the position sensor.
     *
     * @return current motor position in sensor units.
     */
    public double getPosition()
    {
        final String funcName = "getPosition";
        double currPos;

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
            currPos = getMotorPosition() - zeroPosition;
        }

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "%s.%s: pos=%.0f", moduleName, instanceName, currPos);
        }

        return currPos;
    }   //getPosition

    /**
     * This method returns the velocity of the motor rotation. Velocity could either be obtained by calling the motor
     * hardware if it supports it or using a periodic task to monitor the position sensor value. However, accessing
     * hardware may impact performance because it may involve initiating USB/CAN/I2C bus cycles. Therefore, it may be
     * beneficial to just let the the odometry task calculate the velocity here.
     *
     * @return motor velocity in sensor units per second.
     */
    public double getVelocity()
    {
        final String funcName = "getVelocity";
        final double velocity;

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
            velocity = getMotorVelocity();
        }

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "%s.%s: vel=%.0f", moduleName, instanceName, velocity);
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
            globalTracer.traceInfo(funcName, "%s.%s: swapped=%s", moduleName, instanceName, swapped);
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
            globalTracer.traceInfo(funcName, "%s.%s: lowerLimitActive=%s", moduleName, instanceName, isActive);
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
            globalTracer.traceInfo(funcName, "%s.%s: upperLimitActive=%s", moduleName, instanceName, isActive);
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
            globalTracer.traceInfo(
                funcName, "%s.%s: lowerEnabled=%s,upperEnabled=%s",
                moduleName, instanceName, lowerLimitEnabled, upperLimitEnabled);
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
            globalTracer.traceInfo(funcName, "%s.%s: lowerLimit=%.1f", moduleName, instanceName, lowerLimit);
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
            globalTracer.traceInfo(funcName, "%s.%s: upperLimit=%.1f", moduleName, instanceName, upperLimit);
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
     * @param triggerMode specifies the trigger mode.
     * @param triggerCallback specifies an event callback if the trigger occurred, null if none specified.
     */
    public void resetPositionOnDigitalInput(
        TrcDigitalInput digitalInput, TriggerMode triggerMode, TrcEvent.Callback triggerCallback)
    {
        final String funcName = "resetPositionOnDigitalInput";

        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "%s.%s: digitalInput=%s,triggerMode=%s,callback=%s",
                moduleName, instanceName, digitalInput, triggerMode, triggerCallback != null);
        }

        digitalTrigger = new TrcTriggerDigitalInput(instanceName + ".digitalTrigger", digitalInput, this::triggerCallback);
        this.triggerMode = triggerMode;

        if (triggerCallback != null)
        {
            triggerCallbackEvent = new TrcEvent(instanceName + ".callbackEvent");
            triggerCallbackContext = new AtomicBoolean();
            triggerCallbackEvent.setCallback(triggerCallback, triggerCallbackContext);
        }
        digitalTrigger.setEnabled(true);
    }   //resetPositionOnDigitalInput

    /**
     * This method creates a digital trigger on the given digital input sensor. It resets the position sensor
     * reading when the digital input is triggered. This is intended to be called as part of motor initialization.
     * Therefore, it is not designed to be ownership-aware.
     *
     * @param digitalInput   specifies the digital input sensor that will trigger a position reset.
     * @param triggerMode specifies the trigger mode.
     */
    public void resetPositionOnDigitalInput(TrcDigitalInput digitalInput, TriggerMode triggerMode)
    {
        resetPositionOnDigitalInput(digitalInput, triggerMode, null);
    }   //resetPositionOnDigitalInput

    /**
     * This method creates a digital trigger on the given digital input sensor. It resets the position sensor
     * reading when the digital input is triggered. This is intended to be called as part of motor initialization.
     * Therefore, it is not designed to be ownership-aware.
     *
     * @param digitalInput   specifies the digital input sensor that will trigger a position reset.
     * @param triggerCallback specifies an event callback if the trigger occurred, null if none specified.
     */
    public void resetPositionOnDigitalInput(TrcDigitalInput digitalInput, TrcEvent.Callback triggerCallback)
    {
        resetPositionOnDigitalInput(digitalInput, TriggerMode.OnActive, triggerCallback);
    }   //resetPositionOnDigitalInput

    /**
     * This method creates a digital trigger on the given digital input sensor. It resets the position sensor
     * reading when the digital input is triggered.
     *
     * @param digitalInput specifies the digital input sensor that will trigger a position reset.
     */
    public void resetPositionOnDigitalInput(TrcDigitalInput digitalInput)
    {
        resetPositionOnDigitalInput(digitalInput, TriggerMode.OnActive, null);
    }   //resetPositionOnDigitalInput

    /**
     * This method is called when the digital input device has changed state.
     *
     * @param context specifies true if the digital device state is active, false otherwise.
     */
    private void triggerCallback(Object context)
    {
        final String funcName = "triggerCallback";
        boolean active = ((AtomicBoolean) context).get();

        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "%s.%s: trigger=%s,active=%s", moduleName, instanceName, digitalTrigger, active);
        }

        if (calibrating)
        {
            //
            // set(0.0) will turn off calibration mode.
            //
            set(0.0);
            calibrating = false;
        }

        if (triggerMode == TriggerMode.OnBoth ||
            triggerMode == TriggerMode.OnActive && active ||
            triggerMode == TriggerMode.OnInactive && !active)
        {
            globalTracer.traceInfo(
                funcName, "%s.%s: reset position on digital trigger! (BeforePos=%.2f)",
                moduleName, instanceName, getMotorPosition());
            resetPosition(false);
        }

        if (triggerCallbackEvent != null)
        {
            triggerCallbackContext.set(active);
            triggerCallbackEvent.signal();
        }
    }   //triggerCallback

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
            set(calibratePower);
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
                odometryTaskObj.unregisterTask();
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
     * @param resetOdometry specifies true to reset odometry, false otherwise.
     * @param resetHardware specifies true to reset odometry hardware, false otherwise. This is only applicable when
     *        enabling odometry, not used when disabling.
     */
    public void setOdometryEnabled(boolean enabled, boolean resetOdometry, boolean resetHardware)
    {
        final String funcName = "setOdometryEnabled";

        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "%s.%s: enabled=%s,resetOd=%s,hwReset=%s",
                moduleName, instanceName, enabled, resetOdometry, resetHardware);
        }

        if (enabled)
        {
            if (resetOdometry)
            {
                resetOdometry(resetHardware);
            }

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
                    odometryTaskObj.unregisterTask();
                }
            }
        }
        odometryEnabled = enabled;
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
            odometry.prevTimestamp = odometry.currTimestamp = TrcTimer.getCurrentTime();
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
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private static void odometryTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        final String funcName = moduleName + ".odometryTask";

        synchronized (odometryMotors)
        {
            for (TrcMotor motor : odometryMotors)
            {
                synchronized (motor.odometry)
                {
                    motor.odometry.prevTimestamp = motor.odometry.currTimestamp;
                    motor.odometry.prevPos = motor.odometry.currPos;
                    motor.odometry.currTimestamp = TrcTimer.getCurrentTime();

                    if (motorGetPosElapsedTimer != null) motorGetPosElapsedTimer.recordStartTime();
                    motor.odometry.currPos = motor.getMotorPosition() - motor.zeroPosition;
                    if (motorGetPosElapsedTimer != null) motorGetPosElapsedTimer.recordEndTime();
                    //
                    // Detect spurious encoder reading.
                    //
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
                                funcName, "%s.%s: WARNING-Spurious encoder detected! odometry=%s",
                                moduleName, motor, motor.odometry);
                            // Throw away spurious data and use previous data instead.
                            motor.odometry.currPos = motor.odometry.prevPos;
                        }
                    }

                    try
                    {
                        motor.odometry.velocity = motor.getMotorVelocity();
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
                        globalTracer.traceInfo(funcName, "%s.%s: Odometry=%s", moduleName, motor, motor.odometry);
                    }
                }
            }
        }
    }   //odometryTask

    /**
     * This method is called before the runMode is about to stop so we can disable odometry.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is running.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void cleanupTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        clearOdometryMotorsList(false);
    }   //cleanupTask

    //
    // Velocity control mode.
    //

    /**
     * This method sets the motor controller to velocity mode with the specified maximum velocity.
     *
     * @param maxVelocity specifies the maximum velocity the motor can run, in sensor units per second.
     * @param pidCoeff specifies the PID coefficients for software PID control, can be null if using motor built-in
     *        close loop control in which case the PID coefficients of the motor must be set prior to enabling
     *        velocity mode.
     */
    public synchronized void enableVelocityMode(double maxVelocity, TrcPidController.PidCoefficients pidCoeff)
    {
        final String funcName = "enableVelocityMode";

        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "%s.%s: maxVel=%f,pidCoef=%s", moduleName, instanceName, maxVelocity, pidCoeff);
        }

        this.maxMotorVelocity = maxVelocity;
        velocityControlEnabled = true;
        if (pidCoeff != null)
        {
            // Using software PID control requires odometry to calculate velocity
            if (!odometryEnabled)
            {
                throw new RuntimeException("Motor odometry must be enabled to use velocity mode.");
            }
            velocityPidCtrl = new TrcPidController(
                instanceName + ".velocityPidCtrl", pidCoeff, 1.0, this::getNormalizedVelocity);
            velocityPidCtrl.setAbsoluteSetPoint(true);
            velocityCtrlTaskObj.registerTask(TaskType.OUTPUT_TASK);
        }
    }   //enableVelocityMode

    /**
     * This method disables velocity mode returning it to power mode.
     */
    public synchronized void disableVelocityMode()
    {
        if (velocityPidCtrl != null)
        {
            velocityCtrlTaskObj.unregisterTask();
            velocityPidCtrl = null;
            velocityControlEnabled = false;
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
        return maxMotorVelocity != 0.0? getVelocity() / maxMotorVelocity: 0.0;
    }   //getNormalizedVelocity

    /**
     * This method overrides the motorSpeedTask in TrcMotor which is called periodically to calculate he speed of
     * the motor. In addition to calculate the motor speed, it also calculates and sets the motor power required
     * to maintain the set speed if speed control mode is enabled.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is running.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private synchronized void velocityCtrlTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        final String funcName = "velocityCtrlTask";

        if (velocityPidCtrl != null)
        {
            double desiredStallTorquePercentage = velocityPidCtrl.getOutput();
            double motorPower = transformTorqueToMotorPower(desiredStallTorquePercentage);

            setMotorPower(motorPower);
            for (TrcMotor motor: followingMotorsList)
            {
                motor.setMotorPower(motorPower);
            }

            if (debugEnabled)
            {
                globalTracer.traceInfo(funcName,
                    "%s: targetSpeed=%.2f, currSpeed=%.2f, desiredStallTorque=%.2f, motorPower=%.2f",
                    moduleName, velocityPidCtrl.getTarget(), getVelocity(), desiredStallTorquePercentage, motorPower);
            }
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
        //
        // Leverage motor curve information to linearize torque output across varying RPM
        // as best we can. We know that max torque is available at 0 RPM and zero torque is
        // available at max RPM. Use that relationship to proportionately boost voltage output
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
            globalTracer.traceInfo(
                funcName, "%s.%s: torque=%f,power=%f", moduleName, instanceName, desiredStallTorquePercentage, power);
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
