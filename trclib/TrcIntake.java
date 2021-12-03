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
 * This class implements a platform independent auto-assist intake subsystem. It contains a motor and a sensor that
 * detects if the intake has captured objects. It provides the autoAssist method that allows the caller to call the
 * intake subsystem to pickup or dump objects on a press of a button and the intake subsystem will stop itself once
 * it is done. While it provides the auto-assist functionality to pickup or dump objects, it also supports exclusive
 * subsystem access by implementing TrcExclusiveSubsystem. This enables the intake subsystem to be aware of multiple
 * callers' access to the subsystem. While one caller starts the intake for an operation, nobody can access it until
 * the previous caller is done with the operation.
 */
public class TrcIntake<D> implements TrcExclusiveSubsystem
{
    private static final String moduleName = "TrcIntake";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public static class Parameters
    {
        public boolean motorInverted = false;
        public double sensorThreshold = 0.0;
        public boolean detectedBelowThreshold = false;
        public TrcDbgTrace msgTracer = null;

        /**
         * This method sets the direction of the motor.
         *
         * @param inverted specifies true if motor is inverted, false otherwise.
         * @return this parameter object.
         */
        public Parameters setMotorInverted(boolean inverted)
        {
            this.motorInverted = inverted;
            return this;
        }   //setMotorInverted

        /**
         * This method sets the sensor threshold value.
         *
         * @param threshold specifies the sensor threshold value.
         * @param detectedBelowThreshold specifies true if object is detected when below threshold, false if object is
         *        detected when above threshold.
         * @return this parameter object.
         */
        public Parameters setSensorThreshold(double threshold, boolean detectedBelowThreshold)
        {
            this.sensorThreshold = threshold;
            this.detectedBelowThreshold = detectedBelowThreshold;
            return this;
        }   //setSensorThreshold

        /**
         * This method sets the message tracer for logging trace messages.
         *
         * @param tracer specifies the tracer for logging messages.
         * @return this parameter object.
         */
        public Parameters setMsgTracer(TrcDbgTrace tracer)
        {
            this.msgTracer = tracer;
            return this;
        }   //setMsgTracer

        /**
         * This method returns the string form of all the parameters.
         *
         * @return string form of all the parameters.
         */
        @Override
        public String toString()
        {
           return String.format(
               Locale.US, "motorInverted=%s,sensorThreshold=%.2f,detectedBelowThreshold=%s",
               motorInverted, sensorThreshold, detectedBelowThreshold);
        }   //toString

    }   //class Parameters

    private final String instanceName;
    private final TrcMotor motor;
    private final Parameters params;
    private final TrcSensor<D> sensor;
    private final int sensorIndex;
    private final D sensorDataType;
    private final TrcAnalogSensorTrigger<D> sensorTrigger;
    private final TrcTimer timer;
    private TrcEvent onFinishEvent = null;
    private TrcNotifier.Receiver onFinishCallback = null;
    private double autoAssistPower = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param motor specifies the motor object.
     * @param params specifies the parameters object.
     * @param sensor specifies the sensor object, can be null if no sensor is used.
     * @param sensorIndex specifies the sensor index to access the data.
     * @param sensorDataType specifies the sensor data type.
     */
    public TrcIntake(
        String instanceName, TrcMotor motor, Parameters params, TrcSensor<D> sensor, int sensorIndex, D sensorDataType)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.motor = motor;
        this.params = params;
        this.sensor = sensor;
        this.sensorIndex = sensorIndex;
        this.sensorDataType = sensorDataType;

        motor.setInverted(params.motorInverted);
        if (sensor != null)
        {
            sensorTrigger = new TrcAnalogSensorTrigger<>(
                instanceName + ".sensorTrigger", sensor, sensorIndex, sensorDataType,
                new double[] {params.sensorThreshold}, this::triggerHandler, false);
            timer = new TrcTimer(moduleName + "Timer");
        }
        else
        {
            sensorTrigger = null;
            timer = null;
        }
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
        this(instanceName, motor, params, null, 0, null);
    }   //TrcIntake

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
     * This method returns the current motor power.
     *
     * @return current motor power.
     */
    public double getPower()
    {
        final String funcName = "getPower";
        double power = motor.getPower();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", power);
        }

        return power;
    }   //getPower

    /**
     * This method spins the intake with the given power. It will only do this if the caller has acquired exclusive
     * access to the intake already.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param power specifies the power value to spin the intake.
     */
    public synchronized void setPower(String owner, double power)
    {
        final String funcName = "setPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "owner=%s,power=%.1f", owner, power);
        }

        if (validateOwnership(owner))
        {
            motor.set(power);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPower

    /**
     * This method spins the intake with the given power. It will only do this if nobody acquires exclusive access
     * to the intake already.
     *
     * @param power specifies the power value to spin the intake.
     */
    public void setPower(double power)
    {
        setPower(null, power);
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.The value can be power or velocity percentage depending on whether the motor controller is in
     * power mode or velocity mode.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param time specifies the time period in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public synchronized void setPower(String owner, double power, double time, TrcEvent event)
    {
        final String funcName = "setPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API, "owner=%s,power=%.1f,time=%.3f,event=%s",
                owner, power, time, event);
        }

        if (validateOwnership(owner))
        {
            motor.set(power, time, event);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPower

    /**
     * This method sets the motor output value for the set period of time. The motor will be turned off after the
     * set time expires.The value can be power or velocity percentage depending on whether the motor controller is in
     * power mode or velocity mode.
     *
     * @param power specifies the percentage power or velocity (range -1.0 to 1.0) to be set.
     * @param time specifies the time period in seconds to have power set.
     * @param event specifies the event to signal when time has expired.
     */
    public void setPower(double power, double time, TrcEvent event)
    {
        setPower(null, power, time, event);
    }   //setPower

    /**
     * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once object is detected in the intake at which time the given event will be signaled
     * or it will notify the caller's handler.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
     *              power to dump.
     * @param event specifies the event to signal when object is detected in the intake.
     * @param callback specifies the callback handler to call when object is detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasObject() to figure out if it has given up.
     */
    public synchronized void autoAssist(
        String owner, double power, TrcEvent event, TrcNotifier.Receiver callback, double timeout)
    {
        final String funcName = "autoAssist";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API, "owner=%s,power=%.1f,event=%s,timeout=%.3f",
                owner, power, event, timeout);
        }

        if (sensor == null || power == 0.0)
        {
            throw new RuntimeException("Must have sensor and non-zero power to perform AutoAssist.");
        }
        //
        // This is an auto-assist operation, make sure the caller has ownership.
        //
        if (validateOwnership(owner))
        {
            if (event != null)
            {
                event.clear();
            }

            if (power > 0.0 ^ hasObject())
            {
                // Picking up object and we don't have one yet, or dumping object and we still have one.
                if (params.msgTracer != null)
                {
                    params.msgTracer.traceInfo(
                        funcName, "owner=%s, power=%.1f, event=%s, timeout=%.3f", owner, power, event, timeout);
                }
                motor.set(power);
                this.onFinishEvent = event;
                this.onFinishCallback = callback;
                if (timeout > 0.0)
                {
                    timer.set(timeout, this::timeoutHandler);
                }
                sensorTrigger.setEnabled(true);
                this.autoAssistPower = power;
            }
            else
            {
                // Picking up object but we already have one, or dumping object but there isn't any.
                if (params.msgTracer != null)
                {
                    params.msgTracer.traceInfo(funcName, "Already done: hasFreight=%s", hasObject());
                }

                if (event != null)
                {
                    event.signal();
                }

                if (callback != null)
                {
                    callback.notify(null);
                }
            }
        }
    }   //autoAssist

    /**
     * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once object is detected in the intake at which time the given event will be signaled
     * or it will notify the caller's handler.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
     * @param power specifies the power value to spin the intake.
     */
    public void autoAssist(String owner, double power)
    {
        autoAssist(owner, power, null, null, 0.0);
    }   //autoAssist

    /**
     * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once object is detected in the intake at which time the given event will be signaled
     * or it will notify the caller's handler.
     *
     * @param power specifies the power value to spin the intake.
     * @param event specifies the event to signal when there is freight detected in the intake.
     * @param callback specifies the callback handler to call when there is freight detected in the intake.
     * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
     *                must call hasFreight to figure out if it has given up.
     */
    public void autoAssist(double power, TrcEvent event, TrcNotifier.Receiver callback, double timeout)
    {
        autoAssist(null, power, event, callback, timeout);
    }   //autoAssist

    /**
     * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
     * and it will stop itself once object is detected in the intake at which time the given event will be signaled
     * or it will notify the caller's handler.
     *
     * @param power specifies the power value to spin the intake.
     */
    public void autoAssist(double power)
    {
        autoAssist(null, power, null, null, 0.0);
    }   //autoAssist

    /**
     * This method returns the sensor data read from the sensor.
     *
     * @return value read from the sensor.
     */
    public double getSensorData()
    {
        @SuppressWarnings("unchecked") TrcSensor.SensorData<Double> sensorData =
            sensor != null? (TrcSensor.SensorData<Double>) sensor.getRawData(sensorIndex, sensorDataType) : null;
        return sensorData != null ? sensorData.value : 0.0;
    }   //getSensorData

    /**
     *
     * This method checks if object is detected in the intake.
     *
     * @return true if object is detected in the intake, false otherwise.
     */
    public boolean hasObject()
    {
        return params.detectedBelowThreshold ^ getSensorData() >= params.sensorThreshold;
    }   //hasObject

    /**
     * This method checks if auto-assist pickup is active.
     *
     * @return true if auto-assist is in progress, false otherwise.
     */
    public synchronized boolean isAutoAssistActive()
    {
        return autoAssistPower != 0.0;
    }   //isAutoAssistActive

    /**
     * This method is called internally either at the end of the timeout or when freight is detected to stop the
     * intake's spinning and signal or notify the caller for completion.
     */
    public synchronized void cancelAutoAssist()
    {
        if (isAutoAssistActive())
        {
            // AutoAssist is still in progress, cancel it.
            motor.set(0.0);
            timer.cancel();
            sensorTrigger.setEnabled(false);

            if (onFinishEvent != null)
            {
                onFinishEvent.signal();
            }

            if (onFinishCallback != null)
            {
                onFinishCallback.notify(null);
            }

            autoAssistPower = 0.0;
        }
    }   //cancelAutoAssist

    /**
     * This method is called when object is detected in the intake.
     *
     * @param currZone specifies the current threshold zone.
     * @param prevZone specifies the previous threshold zone.
     * @param zoneValue specifies the sensor value.
     */
    private synchronized void triggerHandler(int currZone, int prevZone, double zoneValue)
    {
        final String funcName = "triggerHandler";

        if (params.msgTracer != null)
        {
            params.msgTracer.traceInfo(funcName, "Zone=%d->%d, value=%.3f", prevZone, currZone, zoneValue);
        }

        if (isAutoAssistActive() && prevZone != -1)
        {
            if (params.msgTracer != null)
            {
                params.msgTracer.traceInfo(funcName, "Trigger: hasObject=%s", hasObject());
            }

            cancelAutoAssist();
        }
    }   //triggerHandler

    /**
     * This method is called when timeout expires.
     *
     * @param timer specifies the timer that has expired.
     */
    private synchronized void timeoutHandler(Object timer)
    {
        final String funcName = "timeoutHandler";

        if (!this.timer.isCanceled())
        {
            if (params.msgTracer != null)
            {
                params.msgTracer.traceInfo(funcName, "AutoAssist timed out.");
            }
            cancelAutoAssist();
        }
    }   //timeoutHandler

}   //class TrcIntake
