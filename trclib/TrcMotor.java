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
 * This class implements a platform independent generic motor controller. Typically, this class is extended by a
 * platform dependent motor controller class. Not all motor controllers are created equal. Some have more features
 * than the others. This class attempts to emulate some of the features in software. If the platform dependent motor
 * controller supports some features in hardware it should override the corresponding methods and call the hardware
 * directly. For some features that there is no software emulation, this class will throw an
 * UnsupportedOperationException. If the motor controller hardware support these features, the platform dependent
 * class should override these methods to provide the support in hardware.
 */
public abstract class TrcMotor implements TrcMotorController, TrcOdometrySensor, TrcExclusiveSubsystem
{
    private static final String moduleName = "TrcMotor";
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;
    private static final boolean verbosePidInfo = false;
    private TrcDbgTrace msgTracer = null;
    private TrcRobotBattery battery = null;
    private boolean tracePidInfo = false;

    public enum ControlMode
    {
        PowerMode,
        PositionMode,
        VelocityMode
    }   //enum ControlMode

    public enum TriggerMode
    {
        OnActive,
        OnInactive,
        OnBoth
    }   //enum TriggerMode

    //
    // Global objects.
    //
    private static final ArrayList<TrcMotor> odometryMotors = new ArrayList<>();
    private static TrcTaskMgr.TaskObject odometryTaskObj;
    protected static TrcElapsedTimer motorGetPosElapsedTimer;
    protected static TrcElapsedTimer motorSetPowerElapsedTimer;
    protected static TrcElapsedTimer motorSetVelElapsedTimer;
    private final ArrayList<TrcMotor> followingMotorsList = new ArrayList<>();

    private final String instanceName;
    private final double maxMotorVelocity;
    private final TrcOdometrySensor.Odometry odometry;
    private final TrcTimer timer;
    private final TrcTaskMgr.TaskObject posCtrlTaskObj;
    private final TrcTaskMgr.TaskObject velCtrlTaskObj;
    private boolean limitSwitchesSwapped = false;
    private boolean softLowerLimitEnabled = false;
    private boolean softUpperLimitEnabled = false;
    private double softLowerLimit = 0.0;
    private double softUpperLimit = 0.0;
    private double zeroPosition = 0.0;
    private TrcPidController posPidCtrl = null;
    private TrcPidController velPidCtrl = null;
    private TrcTriggerDigitalInput digitalTrigger;
    private TriggerMode triggerMode;
    private TrcEvent.Callback triggerCallback;
    private AtomicBoolean triggerCallbackContext;
    private TrcEvent triggerCallbackEvent;
    private boolean odometryEnabled = false;
    private boolean calibrating = false;
    private double motorValue;
    private double duration;
    private TrcEvent notifyEvent;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param maxVelocity specifies the maximum velocity the motor can run, in sensor units per second, can be zero
     *        if not provided in which case velocity control mode is not available.
     */
    public TrcMotor(String instanceName, double maxVelocity)
    {
        this.instanceName = instanceName;
        this.maxMotorVelocity = maxVelocity;
        odometry = new TrcOdometrySensor.Odometry(this);
        timer = new TrcTimer(instanceName);
        posCtrlTaskObj = TrcTaskMgr.createTask(instanceName + ".posCtrlTask", this::posCtrlTask);
        velCtrlTaskObj = TrcTaskMgr.createTask(instanceName + ".velCtrlTask", this::velCtrlTask);

        if (odometryTaskObj == null)
        {
            // Odometry task is a singleton that manages odometry of all motors.
            // This will be a STANDALONE_TASK so that it won't degrade the host task with long delay waiting for the
            // hardware. If we create individual task for each motor, moving them to STANDALONE_TASK will create too
            // many threads.
            odometryTaskObj = TrcTaskMgr.createTask(moduleName + ".odometryTask", TrcMotor::odometryTask);
            TrcTaskMgr.TaskObject odometryCleanupTaskObj = TrcTaskMgr.createTask(
                instanceName + ".odometryCleanupTask", this::odometryCleanupTask);
            odometryCleanupTaskObj.registerTask(TaskType.STOP_TASK);
        }
    }   //TrcMotor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcMotor(String instanceName)
    {
        this(instanceName, 0.0);
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
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     * @param battery specifies the battery object to get battery info for the message.
     */
    public void setMsgTracer(TrcDbgTrace tracer, boolean tracePidInfo, TrcRobotBattery battery)
    {
        this.msgTracer = tracer;
        this.tracePidInfo = tracePidInfo;
        this.battery = battery;
    }   //setMsgTracer

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     */
    public void setMsgTracer(TrcDbgTrace tracer, boolean tracePidInfo)
    {
        setMsgTracer(tracer, tracePidInfo, null);
    }   //setMsgTracer

    /**
     * This method sets the message tracer for logging trace messages.
     *
     * @param tracer specifies the tracer for logging messages.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        setMsgTracer(tracer, false, null);
    }   //setMsgTracer

    /**
     * This method swaps the forward and reverse limit switches. By default, the lower limit switch is associated
     * with the reverse limit switch and the upper limit switch is associated with the forward limit switch. This
     * method will swap the association.
     *
     * @param swapped specifies true to swap the limit switches, false otherwise.
     */
    public void setLimitSwitchesSwapped(boolean swapped)
    {
        limitSwitchesSwapped = swapped;
    }   //setLimitSwitchesSwapped

    /**
     * This method returns the state of the lower limit switch.
     *
     * @return true if lower limit switch is active, false otherwise.
     */
    public boolean isLowerLimitSwitchActive()
    {
        return limitSwitchesSwapped? isFwdLimitSwitchActive(): isRevLimitSwitchActive();
    }   //isLowerLimitSwitchActive

    /**
     * This method returns the state of the upper limit switch.
     *
     * @return true if upper limit switch is active, false otherwise.
     */
    public boolean isUpperLimitSwitchActive()
    {
        return limitSwitchesSwapped? isRevLimitSwitchActive(): isFwdLimitSwitchActive();
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
     * This method resets the motor position sensor, typically an encoder. This method emulates a reset for a
     * potentiometer by doing a soft reset.
     *
     * @param hardware specifies true for resetting hardware position, false for resetting software position.
     */
    public void resetPosition(boolean hardware)
    {
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
        double currPos;

        if (odometryEnabled)
        {
            synchronized (odometry)
            {
                // Don't read from motor hardware directly, get it from odometry instead.
                // Odometry already took care of zeroPosition adjustment.
                currPos = odometry.currPos;
            }
        }
        else
        {
            currPos = getMotorPosition() - zeroPosition;
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
        final double currVel;

        if (odometryEnabled)
        {
            synchronized (odometry)
            {
                // Don't read from motor hardware directly, get it from odometry instead.
                currVel = odometry.velocity;
            }
        }
        else
        {
            currVel = getMotorVelocity();
        }

        return currVel;
    }   //getVelocity

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

        digitalTrigger = new TrcTriggerDigitalInput(instanceName + ".digitalTrigger", digitalInput);
        this.triggerMode = triggerMode;
        this.triggerCallback = triggerCallback;

        if (triggerCallback != null)
        {
            triggerCallbackContext = new AtomicBoolean();
            triggerCallbackEvent = new TrcEvent(instanceName + ".callbackEvent");
        }
        digitalTrigger.enableTrigger(this::resetTriggerCallback);
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
    private void resetTriggerCallback(Object context)
    {
        final String funcName = "resetTriggerCallback";
        boolean active = ((AtomicBoolean) context).get();

        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "%s.%s: trigger=%s,active=%s", moduleName, instanceName, digitalTrigger, active);
        }

        if (calibrating)
        {
            // set(0.0) will turn off calibration mode.
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
            triggerCallbackEvent.setCallback(triggerCallback, triggerCallbackContext);
            triggerCallbackEvent.signal();
        }
    }   //resetTriggerCallback

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
        // TODO: How about calibrate using motor stall?
        //
        if (digitalTrigger != null && digitalTrigger.isEnabled())
        {
            set(calibratePower);
            calibrating = true;
        }
    }   //zeroCalibrate

    /**
     * This method calls the motor subclass to set the motor output value. The value can be power or velocity
     * percentage depending on whether the motor controller is in power mode or velocity mode.
     *
     * @param value specifies the percentage power value (range -1 to 1) or velocity value in sensor unit (e.g.
     *        encoder counts/sec).
     */
    private void setMotorValue(double value)
    {
        ControlMode controlMode = getControlMode();

        if (controlMode == ControlMode.VelocityMode)
        {
            // Do our software velocity PID control.
            setMotorVelocity(value);
        }
        else if (controlMode == ControlMode.PowerMode)
        {
            setMotorPower(value);
        }
        // If motor subclass supports followMotor natively, followingMotorsList will be empty.
        // If not, this is software simulation of followMotor.
        synchronized (followingMotorsList)
        {
            for (TrcMotor follower : followingMotorsList)
            {
                if (controlMode == ControlMode.VelocityMode)
                {
                    follower.setMotorVelocity(value);
                }
                else if (controlMode == ControlMode.PowerMode)
                {
                    follower.setMotorPower(value);
                }
            }
        }
    }   //setMotorValue

    /**
     * This method sets the motor output value. The value can be power or velocity percentage depending on whether
     * the motor controller is in power mode or velocity mode. Optionally, you can specify a delay before running
     * the motor and a duration for which the motor will be turned off afterwards.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param delay specifies the time in seconds to delay before setting the power, 0.0 if no delay.
     * @param value specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to run the motor and turns it off afterwards, 0.0 if not
     *        turning off.
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

        if (validateOwnership(owner) && getControlMode() != ControlMode.PositionMode)
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
            this.duration = duration;
            this.notifyEvent = event;
            if (delay > 0.0)
            {
                // The motor may be spinning, let's stop it.
                stopMotor();
                timer.set(delay, this::delayExpiredCallback);
            }
            else
            {
                delayExpiredCallback(null);
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
     *        turning off.
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
     *        turning off.
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
     *        turning off.
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
     *        turning off.
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
     * This method is called when set motor power delay timer has expired. It will set the specified motor power.
     *
     * @param context specifies the timer object (not used).
     */
    private void delayExpiredCallback(Object context)
    {
        // Delay timer has expired, set the motor power now.
        setMotorValue(motorValue);
        if (duration > 0.0)
        {
            // We have set a duration, set up a timer for it.
            timer.set(duration, this::durationExpiredCallback);
        }
        else if (notifyEvent != null)
        {
            notifyEvent.signal();
            notifyEvent = null;
        }
    }   //delayExpiredCallback

    /**
     * This method is called when set motor power duration timer has expired. It will turn the motor off.
     *
     * @param context specifies the timer object (not used).
     */
    private void durationExpiredCallback(Object context)
    {
        // The duration timer has expired, turn everything off and clean up.
        duration = 0.0;
        motorValue = 0.0;
        setMotorValue(motorValue);
        if (notifyEvent != null)
        {
            notifyEvent.signal();
            notifyEvent = null;
        }
    }   //durationExpiredCallback

    /**
     * This method adds the given motor to the list that will follow this motor. It should only be called by the
     * given motor to add it to the follower list of the motor it wants to follow.
     *
     * @param motor specifies the motor that will follow this motor.
     */
    private void addFollowingMotor(TrcMotor motor)
    {
        synchronized (followingMotorsList)
        {
            if (!followingMotorsList.contains(motor))
            {
                followingMotorsList.add(motor);
            }
        }
    }   //addFollowingMotor

    //
    // Implements TrcMotorController interface, simulate in software if necessary.
    //

    /**
     * This method sets this motor to follow another motor.
     *
     * @param motor specifies the motor to follow.
     */
    @Override
    public void followMotor(TrcMotor motor)
    {
        motor.addFollowingMotor(this);
    }   //followMotor

    /**
     * This method sets the PID coefficients for the software position PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setPositionPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        if (posPidCtrl != null)
        {
            // We are using software PID Control.
            posPidCtrl.setPidCoefficients(pidCoeff);
        }
        else
        {
            throw new IllegalStateException("Position control mode is not enabled.");
        }
    }   //setPositionPidCoefficients

    /**
     * This method returns the PID coefficients of the motor's position PID controller.
     *
     * @return PID coefficients of the motor's position PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getPositionPidCoefficients()
    {
        return posPidCtrl != null? posPidCtrl.getPidCoefficients(): null;
    }   //getPositionPidCoefficients

    /**
     * This method sets the PID coefficients of the motor's velocity PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    public void setVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        if (velPidCtrl != null)
        {
            velPidCtrl.setPidCoefficients(pidCoeff);
        }
        else
        {
            throw new IllegalStateException("Velocity control mode is not enabled.");
        }
    }   //setVelocityPidCoefficients

    /**
     * This method returns the PID coefficients of the motor's velocity PID controller.
     *
     * @return PID coefficients of the motor's veloicty PID controller.
     */
    public TrcPidController.PidCoefficients getVelocityPidCoefficients()
    {
        return velPidCtrl != null? velPidCtrl.getPidCoefficients(): null;
    }   //getVelocityPidCoefficients

    /**
     * This method sets the motor controller to position control mode.
     *
     * @param pidCoeff specifies the PID coefficients for position PID control.
     * @param useSoftwarePid must specify true because if this is not overridden by super class, the motor does not
     *        support native PID control.
     */
    @Override
    public void enablePositionMode(TrcPidController.PidCoefficients pidCoeff, boolean useSoftwarePid)
    {
        if (!useSoftwarePid)
        {
            throw new UnsupportedOperationException("Motor does not support native PID control.");
        }

        if (pidCoeff == null)
        {
            throw new IllegalArgumentException("Must provide PID coefficients.");
        }

        ControlMode controlMode = getControlMode();

        if (controlMode == ControlMode.VelocityMode)
        {
            this.disableVelocityMode();
        }

        if (controlMode == ControlMode.PositionMode)
        {
            // Already in Position Mode, just update the PID coefficients.
            posPidCtrl.setPidCoefficients(pidCoeff);
        }
        else
        {
            posPidCtrl = new TrcPidController(instanceName + ".posPidCtrl", pidCoeff, 1.0, this::getPosition);
            posCtrlTaskObj.registerTask(TaskType.POST_PERIODIC_TASK);
        }
    }   //enablePositionMode

    /**
     * This method disables position control mode returning it to power control mode.
     */
    @Override
    public void disablePositionMode()
    {
        if (getControlMode() == ControlMode.PositionMode)
        {
            posCtrlTaskObj.unregisterTask();
            posPidCtrl = null;
        }
    }   //disablePositionMode

    /**
     * This method sets the motor controller to velocity control mode.
     *
     * @param pidCoeff specifies the PID coefficients for velocity PID control.
     * @param useSoftwarePid must specify true because if this is not overridden by super class, the motor does not
     *        support native PID control.
     */
    @Override
    public void enableVelocityMode(TrcPidController.PidCoefficients pidCoeff, boolean useSoftwarePid)
    {
        if (maxMotorVelocity == 0.0)
        {
            throw new UnsupportedOperationException(
                "Software velocity mode not supported because maximum motor velocity is not provided.");
        }

        if (!useSoftwarePid)
        {
            throw new UnsupportedOperationException("Motor does not support native PID control.");
        }

        if (pidCoeff == null)
        {
            throw new IllegalArgumentException("Must provide PID coefficients.");
        }

        ControlMode controlMode = getControlMode();

        if (controlMode == ControlMode.PositionMode)
        {
            this.disablePositionMode();
        }

        if (controlMode == ControlMode.VelocityMode)
        {
            // Already in velocity control mode, just update PID coefficients and maxVelocity.
            velPidCtrl.setPidCoefficients(pidCoeff);
        }
        else
        {
            velPidCtrl = new TrcPidController(instanceName + ".velPidCtrl", pidCoeff, 1.0, this::getNormalizedVelocity);
            velCtrlTaskObj.registerTask(TaskType.POST_PERIODIC_TASK);
        }
    }   //enableVelocityMode

    /**
     * This method disables velocity control mode returning it to power control mode.
     */
    @Override
    public void disableVelocityMode()
    {
        if (getControlMode() == ControlMode.VelocityMode)
        {
            velCtrlTaskObj.unregisterTask();
            velPidCtrl = null;
        }
    }   //disableVelocityMode

    /**
     * This method returns the current control mode.
     *
     * @return current control mode.
     */
    @Override
    public ControlMode getControlMode()
    {
        return posPidCtrl != null ? ControlMode.PositionMode :
               velPidCtrl != null ? ControlMode.VelocityMode : ControlMode.PowerMode;
    }   //getControlMode

    /**
     * This method commands the motor to go to the given position using software PID control.
     *
     * @param pos specifies the motor position in raw sensor units (encoder counts).
     */
    @Override
    public void setMotorPosition(double pos)
    {
        // Allow this only if we are in close loop position control mode.
        if (getControlMode() == ControlMode.PositionMode)
        {
            posPidCtrl.setTarget(pos);
        }
        else
        {
            throw new IllegalStateException("Motor is not in Position Mode.");
        }
    }   //setMotorPosition

    /**
     * This method commands the motor to run at the given velocity using software PID control.
     *
     * @param vel specifies the motor velocity in raw sensor units per second (encoder counts per second).
     */
    @Override
    public void setMotorVelocity(double vel)
    {
        // Allow this only if we are in close loop velocity control mode.
        if (getControlMode() == ControlMode.VelocityMode)
        {
            velPidCtrl.setTarget(vel);
        }
        else
        {
            throw new IllegalStateException("Motor is not in Velocity Mode.");
        }
    }   //setMotorVelocity

    //
    // Odometry.
    //

    /**
     * This method clears the list of motors that register for odometry monitoring. This method should only be called
     * by the task scheduler.
     *
     * @param removeOdometryTask specifies true to also remove the odometry task object, false to leave it alone.
     *        This is mainly for FTC, FRC should always set this to false.
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
     *        require some time to complete and will block this method from returning until finish.
     */
    @Override
    public void resetOdometry(boolean resetHardware)
    {
        resetPosition(resetHardware);
        synchronized (odometry)
        {
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
                    motor.odometry.currPos = motor.getMotorPosition() - motor.zeroPosition;
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
                        // It doesn't support velocity data so calculate it ourselves.
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
    private void odometryCleanupTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        clearOdometryMotorsList(false);
    }   //odometryCleanupTask

    //
    // Close-loop control modes.
    //

    /**
     * This method performs software position control and is called periodically to set the motor power required to
     * approach the target position. When target position is reached, this method will try to maintain the position
     * indefinitely until position control is turned off.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is running.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private synchronized void posCtrlTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        if (posPidCtrl != null)
        {
            double motorPower = posPidCtrl.getOutput();

            setMotorPower(motorPower);
            synchronized (followingMotorsList)
            {
                for (TrcMotor motor : followingMotorsList)
                {
                    motor.setMotorPower(motorPower);
                }
            }

            if (msgTracer != null && tracePidInfo)
            {
                posPidCtrl.printPidInfo(msgTracer, verbosePidInfo, battery);
            }
        }
    }   //posCtrlTask

    /**
     * This method performs software velocity control and is called periodically to set the motor power required to
     * approach the target velocity. When target velocity is reached, this method will try to maintain the velocity
     * indefinitely until velocity control is turned off.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the competition mode that is running.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private synchronized void velCtrlTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        if (velPidCtrl != null)
        {
            /*
            // PID controller gives us the desired stall torque percentage.
            double motorPower = transformTorqueToMotorPower(velPidCtrl.getOutput());
             */
            double motorPower = velPidCtrl.getOutput();

            setMotorPower(motorPower);
            synchronized (followingMotorsList)
            {
                for (TrcMotor motor : followingMotorsList)
                {
                    motor.setMotorPower(motorPower);
                }
            }

            if (msgTracer != null && tracePidInfo)
            {
                velPidCtrl.printPidInfo(msgTracer, verbosePidInfo, battery);
            }
        }
    }   //velCtrlTask

//    /**
//     * Transforms the desired percentage of motor stall torque to the motor duty cycle (aka power)
//     * that would give us that amount of torque at the current motor speed.
//     *
//     * @param desiredStallTorquePercentage specifies the desired percentage of motor torque to receive in percent of
//     *        motor stall torque.
//     * @return power percentage to apply to the motor to generate the desired torque (to the best ability of the motor).
//     */
//    private double transformTorqueToMotorPower(double desiredStallTorquePercentage)
//    {
//        final String funcName = "transformTorqueToMotorPower";
//        double power;
//        //
//        // Leverage motor curve information to linearize torque output across varying RPM
//        // as best we can. We know that max torque is available at 0 RPM and zero torque is
//        // available at max RPM. Use that relationship to proportionately boost voltage output
//        // as motor speed increases.
//        //
//        final double currSpeedSensorUnitPerSec = Math.abs(getVelocity());
//        final double currNormalizedSpeed = currSpeedSensorUnitPerSec / maxMotorVelocity;
//
//        // Max torque percentage declines proportionally to motor speed.
//        final double percentMaxTorqueAvailable = 1 - currNormalizedSpeed;
//
//        if (percentMaxTorqueAvailable > 0)
//        {
//            power = desiredStallTorquePercentage / percentMaxTorqueAvailable;
//        }
//        else
//        {
//            // When we exceed max motor speed (and the correction factor is undefined), apply 100% voltage.
//            power = Math.signum(desiredStallTorquePercentage);
//        }
//
//        if (debugEnabled)
//        {
//            globalTracer.traceInfo(
//                funcName, "%s.%s: torque=%f,power=%f", moduleName, instanceName, desiredStallTorquePercentage, power);
//        }
//
//        return power;
//    }   //transformTorqueToMotorPower

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

            if (motorSetPowerElapsedTimer == null)
            {
                motorSetPowerElapsedTimer = new TrcElapsedTimer(moduleName + ".setPower", 2.0);
            }

            if (motorSetVelElapsedTimer == null)
            {
                motorSetVelElapsedTimer = new TrcElapsedTimer(moduleName + ".setVel", 2.0);
            }
        }
        else
        {
            motorGetPosElapsedTimer = null;
            motorSetPowerElapsedTimer = null;
            motorSetVelElapsedTimer = null;
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

        if (motorSetPowerElapsedTimer != null)
        {
            motorSetPowerElapsedTimer.printElapsedTime(tracer);
        }

        if (motorSetVelElapsedTimer != null)
        {
            motorSetVelElapsedTimer.printElapsedTime(tracer);
        }
    }   //printElapsedTime

}   //class TrcMotor
