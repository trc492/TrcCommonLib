/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.Locale;

/**
 * This class implements a platform independent auto-assist intake subsystem. It contains a motor or a continuous
 * servo and a sensor that detects if the intake has captured objects. It provides the autoAssist method that allows
 * the caller to call the intake subsystem to pickup or dump objects on a press of a button and the intake subsystem
 * will stop itself once it is done. While it provides the auto-assist functionality to pickup or dump objects, it
 * also supports exclusive subsystem access by implementing TrcExclusiveSubsystem. This enables the intake subsystem
 * to be aware of multiple callers' access to the subsystem. While one caller starts the intake for an operation,
 * nobody can access it until the previous caller is done with the operation.
 */
public class TrcIntake implements TrcExclusiveSubsystem
{
    /**
     * This class contains all the parameters related to the intake.
     */
    public static class Parameters
    {
        public Double analogThreshold = null;
        public boolean analogTriggerInverted = false;

        /**
         * This method returns the string form of all the parameters.
         *
         * @return string form of all the parameters.
         */
        @Override
        public String toString()
        {
            return "analogThreshold=" + analogThreshold +
                   ", analogTriggerInverted=" + analogTriggerInverted;
        }   //toString

        /**
         * This method sets the anlog sensor threshold value.
         *
         * @param threshold specifies the sensor threshold value.
         * @param triggerInverted specifies true if got object triggered below threshold, false if triggered above
         *        threshold.
         * @return this parameter object.
         */
        public Parameters setAnalogThreshold(double threshold, boolean triggerInverted)
        {
            this.analogThreshold = threshold;
            this.analogTriggerInverted = triggerInverted;
            return this;
        }   //setAnalogThreshold

    }   //class Parameters

    /**
     * This class encapsulates all the parameters required to perform the intake action.
     */
    private static class ActionParams
    {
        boolean intake;
        double intakePower;
        double retainPower;
        double finishDelay;
        TrcEvent event;
        double timeout;

        ActionParams(
            boolean intake, double intakePower, double retainPower, double finishDelay,
            TrcEvent event, double timeout)
        {
            this.intake = intake;
            this.intakePower = intakePower;
            this.retainPower = retainPower;
            this.finishDelay = finishDelay;
            this.event = event;
            this.timeout = timeout;
        }   //ActionParams

        @Override
        public String toString()
        {
            return "intake=" + intake +
                   ", intakePower=" + intakePower +
                   ", retainPower=" + retainPower +
                   ", finishDelay=" + finishDelay +
                   ", event=" + event +
                   ", timeout=" + timeout;
        }   //toString

    }   //class ActionParams

    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcMotor motor;
    private final Parameters params;
    private final TrcTrigger sensorTrigger;
    private final TrcEvent.Callback eventCallback;
    private final TrcTimer timer;
    private final TrcEvent timerEvent;
    private ActionParams actionParams = null;
    private String currOwner = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param motor specifies the motor object.
     * @param params specifies the parameters object.
     * @param sensorTrigger specifies the sensor trigger object, can be null if none.
     * @param eventCallback specifies the notification callback when a trigger event occurred,
     *        null if no sensorTrigger.
     */
    private TrcIntake(
        String instanceName, TrcMotor motor, Parameters params, TrcTrigger sensorTrigger,
        TrcEvent.Callback eventCallback)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.motor = motor;
        this.params = params;
        this.sensorTrigger = sensorTrigger;
        this.eventCallback = eventCallback;
        timer = new TrcTimer(instanceName);
        timerEvent = new TrcEvent(instanceName + ".timerEvent");
    }   //TrcIntake

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param motor specifies the motor object.
     * @param params specifies the parameters object.
     */
    public TrcIntake(String instanceName, TrcMotor motor, Parameters params)
    {
        this(instanceName, motor, params, null, null);
    }   //TrcIntake

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return String.format(
            Locale.US, "%s: pwr=%.3f, current=%.3f, autoAssistActive=%s, hasObject=%s",
            instanceName, getPower(), motor != null? motor.getMotorCurrent(): 0.0, isAutoAssistActive(), hasObject());
    }   //toString

    /**
     * This method sets the trace level for logging trace messages.
     *
     * @param msgLevel specifies the message level.
     */
    public void setTraceLevel(TrcDbgTrace.MsgLevel msgLevel)
    {
        tracer.setTraceLevel(msgLevel);
    }   //setTraceLevel

    /**
     * This method returns the current motor power.
     *
     * @return current motor power.
     */
    public double getPower()
    {
        return motor.getMotorPower();
    }   //getPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param delay specifies the delay in seconds to wait before setting the power of the motor.
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void setPower(String owner, double delay, double power, double duration, TrcEvent event)
    {
        motor.setPower(owner, delay, power, duration, event);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param delay specifies the delay in seconds to wait before setting the power of the motor.
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void setPower(double delay, double power, double duration, TrcEvent event)
    {
        setPower(null, delay, power, duration, event);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void setPower(double power, double duration, TrcEvent event)
    {
        setPower(null, 0.0, power, duration, event);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param delay specifies the delay in seconds to wait before setting the power of the motor.
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to have power set.
     */
    public void setPower(double delay, double power, double duration)
    {
        setPower(null, delay, power, duration, null);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param duration specifies the duration in seconds to have power set.
     */
    public void setPower(double power, double duration)
    {
        setPower(null, 0.0, power, duration, null);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.
     *
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     */
    public void setPower(double power)
    {
        setPower(null, 0.0, power, 0.0, null);
    }   //setPower

    /**
     * This method is called to finish the auto-assist operation and clean up. It can be called either at the end of
     * the timeout or when object is detected to finish the auto-assist operation and signal the caller for
     * completion. It can also be called if the caller explicitly cancel the auto-assist operation in which case
     * the event will be set to canceled.
     *
     * @param canceled specifies true if the operation is canceled, false otherwsie.
     */
    private void finishAutoAssist(boolean canceled)
    {
        if (isAutoAssistActive())
        {
            tracer.traceDebug(
                instanceName, "canceled=%s, AutoAssistTimedOut=%s, hasObject=%s, finishDelay=%.3f",
                canceled, timerEvent.isSignaled(), hasObject(), actionParams.finishDelay);
            double power = !canceled && hasObject()? actionParams.retainPower: 0.0;
            setPower(actionParams.finishDelay, power, 0.0);
            timer.cancel();
            sensorTrigger.disableTrigger();

            if (actionParams.event != null)
            {
                if (canceled)
                {
                    actionParams.event.cancel();
                }
                else
                {
                    actionParams.event.signal();
                }
                actionParams.event = null;
            }

            actionParams = null;

            if (currOwner != null)
            {
                releaseExclusiveAccess(currOwner);
                currOwner = null;
            }
        }
    }   //finishAutoAssist

    /**
     * This method performs the auto-assist action.
     *
     * @param context specifies the action parameters.
     */
    private void performAutoAssist(Object context)
    {
        ActionParams actionParams = (ActionParams) context;
        boolean objCaptured = hasObject();

        if (actionParams.intake ^ objCaptured)
        {
            // Picking up object and we don't have one yet, or dumping object and we still have one.
            tracer.traceDebug(instanceName, "AutoAssist: " + actionParams + ", hasObject=" + objCaptured);
            motor.setPower(actionParams.intakePower);
            sensorTrigger.enableTrigger(this::triggerCallback);

            if (actionParams.timeout > 0.0)
            {
                timerEvent.setCallback(this::autoAssistTimedOut, null);
                timer.set(actionParams.timeout, timerEvent);
            }
        }
        else
        {
            // Picking up object but we already have one, or dumping object but there isn't any.
            tracer.traceDebug(instanceName, "Already done: hasObject=" + objCaptured);
            finishAutoAssist(false);
        }
    }   //performAutoAssist

    /**
     * This method is called when the sensor trigger state has changed.
     *
     * @param context not used.
     */
    private void triggerCallback(Object context)
    {
        finishAutoAssist(false);
        if (eventCallback != null)
        {
            eventCallback.notify(context);
        }
    }   //triggerCallback

    /**
     * This method is called when the auto-assist operation has timed out.
     *
     * @param context not used.
     */
    private void autoAssistTimedOut(Object context)
    {
        finishAutoAssist(true);
    }   //autoAssistTimedOut

    /**
     * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once object is picked up or dumped in the intake at which time the given event will be
     * signaled.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param intake specifies true for intake operation, false for spitout operation.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured, applicable only to intake
     *        object.
     * @param finishDelay specifies the delay in seconds to fnish the auto-assist operation to give it extra time
     *        spinning the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    private void autoAssistOperation(
        String owner, boolean intake, double delay, double power, double retainPower, double finishDelay,
        TrcEvent event, double timeout)
    {
        if (sensorTrigger == null || power == 0.0)
        {
            throw new RuntimeException("Must have sensor and non-zero power to perform AutoAssist Intake.");
        }
        // Caller specified an owner but did not acquire ownership, acquire it on its behalf.
        if (owner != null && !hasOwnership(owner) && acquireExclusiveAccess(owner))
        {
            currOwner = owner;
        }
        //
        // This is an auto-assist operation, make sure the caller has ownership.
        //
        if (validateOwnership(owner))
        {
            actionParams = new ActionParams(intake, power, retainPower, finishDelay, event, timeout);
            if (delay > 0.0)
            {
                timerEvent.setCallback(this::performAutoAssist, actionParams);
                timer.set(delay, timerEvent);
            }
            else
            {
                performAutoAssist(actionParams);
            }
        }
    }   //autoAssistOperation

    /**
     * This method is an auto-assist intake operation. It allows the caller to start the intake spinning at the given
     * power and it will stop itself once object is picked up in the intake at which time the given event will be
     * signaled.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured.
     * @param finishDelay specifies the delay in seconds to fnish the auto-assist operation to give it extra time
     *        spinning the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoAssistIntake(
        String owner, double delay, double power, double retainPower, double finishDelay,
        TrcEvent event, double timeout)
    {
        autoAssistOperation(owner, true, delay, power, retainPower, finishDelay, event, timeout);
    }   //autoAssistIntake

    /**
     * This method is an auto-assist spitout operation. It allows the caller to start the intake spinning at the given
     * power and it will stop itself once object is dumped in the intake at which time the given event will be
     * signaled.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to dump.
     * @param finishDelay specifies the delay in seconds to fnish the auto-assist operation to give it extra time
     *        spinning the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoAssistSpitout(
        String owner, double delay, double power, double finishDelay, TrcEvent event, double timeout)
    {
        autoAssistOperation(owner, false, delay, power, 0.0, finishDelay, event, timeout);
    }   //autoAssistSpitout

    /**
     * This method is an auto-assist intake operation. It allows the caller to start the intake spinning at the given
     * power and it will stop itself once object is picked up in the intake at which time the given event will be
     * signaled.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured.
     * @param finishDelay specifies the delay in seconds to fnish the auto-assist operation to give it extra time
     *        spinning the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoAssistIntake(
        double delay, double power, double retainPower, double finishDelay, TrcEvent event, double timeout)
    {
        autoAssistOperation(null, true, delay, power, retainPower, finishDelay, event, timeout);
    }   //autoAssistIntake

    /**
     * This method is an auto-assist spitout operation. It allows the caller to start the intake spinning at the given
     * power and it will stop itself once object is dumped in the intake at which time the given event will be
     * signaled.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to dump.
     * @param finishDelay specifies the delay in seconds to fnish the auto-assist operation to give it extra time
     *        spinning the intake.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public void autoAssistSpitout(double delay, double power, double finishDelay, TrcEvent event, double timeout)
    {
        autoAssistOperation(null, false, delay, power, 0.0, finishDelay, event, timeout);
    }   //autoAssistSpitout

    /**
     * This method is an auto-assist intake operation. It allows the caller to start the intake spinning at the given
     * power and it will stop itself once object is picked up in the intake at which time the given event will be
     * signaled.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured.
     * @param finishDelay specifies the delay in seconds to fnish the auto-assist operation to give it extra time
     *        spinning the intake.
     */
    public void autoAssistIntake(double delay, double power, double retainPower, double finishDelay)
    {
        autoAssistOperation(null, true, delay, power, retainPower, finishDelay, null, 0.0);
    }   //autoAssistIntake

    /**
     * This method is an auto-assist spitout operation. It allows the caller to start the intake spinning at the given
     * power and it will stop itself once object is dumped in the intake at which time the given event will be
     * signaled.
     *
     * @param delay specifies the delay time in seconds before executing the action.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to dump.
     * @param finishDelay specifies the delay in seconds to fnish the auto-assist operation to give it extra time
     *        spinning the intake.
     */
    public void autoAssistSpitout(double delay, double power, double finishDelay)
    {
        autoAssistOperation(null, false, delay, power, 0.0, finishDelay, null, 0.0);
    }   //autoAssistSpitout

    /**
     * This method is an auto-assist intake operation. It allows the caller to start the intake spinning at the given
     * power and it will stop itself once object is picked up in the intake at which time the given event will be
     * signaled.
     *
     * @param power specifies the power value to spin the intake.
     * @param retainPower specifies the power to retain the object after it's captured.
     * @param finishDelay specifies the delay in seconds to fnish the auto-assist operation to give it extra time
     *        spinning the intake.
     */
    public void autoAssistIntake(double power, double retainPower, double finishDelay)
    {
        autoAssistOperation(null, true, 0.0, power, retainPower, finishDelay, null, 0.0);
    }   //autoAssistIntake

    /**
     * This method is an auto-assist spitout operation. It allows the caller to start the intake spinning at the given
     * power and it will stop itself once object is dumped in the intake at which time the given event will be
     * signaled.
     *
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to dump.
     * @param finishDelay specifies the delay in seconds to fnish the auto-assist operation to give it extra time
     *        spinning the intake.
     */
    public void autoAssistSpitout(double power, double finishDelay)
    {
        autoAssistOperation(null, false, 0.0, power, 0.0, finishDelay, null, 0.0);
    }   //autoAssistSpitout

    /**
     * This method cancels the auto-assist operation if one is active.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     */
    public void autoAssistCancel(String owner)
    {
        if (validateOwnership(owner))
        {
            if (isAutoAssistActive())
            {
                finishAutoAssist(true);
            }
            else if (getPower() != 0.0)
            {
                setPower(0.0);
            }
        }
    }   //autoAssistCancel

    /**
     * This method cancels the auto-assist operation if one is active.
     */
    public void autoAssistCancel()
    {
        autoAssistCancel(null);
    }   //autoAssistCancel

    /**
     * This method returns the sensor value read from the analog sensor.
     *
     * @return analog sensor value.
     */
    public double getSensorValue()
    {
        return sensorTrigger != null && params.analogThreshold != null? sensorTrigger.getSensorValue(): 0.0;
    }   //getSensorValue

    /**
     * This method returns the sensor state read from the digital sensor.
     *
     * @return digital sensor state.
     */
    public boolean getSensorState()
    {
        return sensorTrigger != null && params.analogThreshold == null && sensorTrigger.getSensorState();
    }   //getSensorState

    /**
     *
     * This method checks if object is detected in the intake.
     *
     * @return true if object is detected in the intake, false otherwise.
     */
    public boolean hasObject()
    {
        boolean gotObject = false;

        if (sensorTrigger != null)
        {
            if (params.analogThreshold != null)
            {
                gotObject = getSensorValue() > params.analogThreshold;
                if (params.analogTriggerInverted)
                {
                    gotObject = !gotObject;
                }
            }
            else
            {
                gotObject = getSensorState();
            }
        }

        return gotObject;
    }   //hasObject

    /**
     * This method checks if auto-assist pickup is active.
     *
     * @return true if auto-assist is in progress, false otherwise.
     */
    public boolean isAutoAssistActive()
    {
        return actionParams != null;
    }   //isAutoAssistActive

}   //class TrcIntake
