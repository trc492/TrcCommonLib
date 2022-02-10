/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

import TrcCommonLib.trclib.TrcTaskMgr.TaskType;

/**
 * This class implements a platform independent auto-assist intake subsystem. It contains a motor and a sensor that
 * detects if the intake has captured objects. It provides the autoAssist method that allows the caller to call the
 * intake subsystem to pickup or dump objects on a press of a button and the intake subsystem will stop itself once
 * it is done. While it provides the auto-assist functionality to pickup or dump objects, it also supports exclusive
 * subsystem access by implementing TrcExclusiveSubsystem. This enables the intake subsystem to be aware of multiple
 * callers' access to the subsystem. While one caller starts the intake for an operation, nobody can access it until
 * the previous caller is done with the operation.
 */
public class TrcPidConveyor extends TrcPidMotor
{
    /**
     * This class contains all the parameters related to the motor actuator.
     */
    public static class Parameters
    {
        public double scale = 1.0;
        public TrcPidController.PidParameters pidParams;
        public double movePower = 1.0;
        public double objectDistance = 0.0;
        public int maxCapacity = 1;

        /**
         * This method sets the scale of the conveyor distance sensor. It allows the conveyor to report real world
         * position units such as inches instead of sensor units.
         *
         * @param scale specifies the scale multiplier to convert position sensor unit to real world unit.
         * @return this parameter object.
         */
        public Parameters setScale(double scale)
        {
            this.scale = scale;
            return this;
        }   //setScale

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
         * This method sets the conveyor move power.
         *
         * @param power specifies the motor power to move the object in the conveyor.
         * @return this parameter object.
         */
        public Parameters setMovePower(double power)
        {
            this.movePower = power;
            return this;
        }   //setMovePower

        /**
         * This method sets the distance between objects inside the converyor.
         *
         * @param distance specifies the distance between objects in the conveyor.
         * @return this parameter object.
         */
        public Parameters setObjectDistance(double distance)
        {
            this.objectDistance = distance;
            return this;
        }   //setObjectDistance

        /**
         * This method sets the maximum number of objects the conveyor can hold.
         *
         * @param maxCapacity specifies maximum number of objects the conveyor can hold.
         * @return this parameter object.
         */
        public Parameters setMaxCapacity(int capacity)
        {
            this.maxCapacity = capacity;
            return this;
        }   //setMaxCapacity

    }   //class Parameters

    private final TrcDigitalInput entranceSensor;
    private final TrcDigitalInput exitSensor;
    private final Parameters params;
    private final TrcDigitalInputTrigger entranceTrigger;
    private final TrcDigitalInputTrigger exitTrigger;
    private final TrcTaskMgr.TaskObject advanceTaskObj;
    private TrcNotifier.Receiver entranceEventHandler;
    private TrcNotifier.Receiver exitEventHandler;
    private int numObjects = 0;
    private TrcEvent onFinishEvent;
    private TrcNotifier.Receiver onFinishCallback;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the hardware name.
     * @param motor specifies the motor object.
     * @param entranceSensor specifies the sensor object for the conveyor entrance.
     * @param exitSensor specifies the sensor object for the conveyor exit.
     * @param params specifies the parameters object.
     */
    public TrcPidConveyor(
        String instanceName, TrcMotor motor, TrcDigitalInput entranceSensor, TrcDigitalInput exitSensor,
        Parameters params)
    {
        super(instanceName, motor, params.pidParams, 0.0);
        this.entranceSensor = entranceSensor;
        this.exitSensor = exitSensor;
        this.params = params;

        setPositionScale(params.scale);

        entranceTrigger = entranceSensor != null?
            new TrcDigitalInputTrigger(instanceName + ".entranceTrigger", entranceSensor, this::entranceEvent): null;
        exitTrigger = exitSensor != null?
            new TrcDigitalInputTrigger(instanceName + ".exitTrigger", exitSensor, this::exitEvent): null;

        advanceTaskObj = TrcTaskMgr.createTask(instanceName + ".advanceTask", this::advanceTask);
    }   //TrcPidConveyor

    public void cancel(String owner)
    {
        if (validateOwnership(owner))
        {
            advanceTaskObj.unregisterTask();
            setPower(0.0);
        }
    }   //cancel

    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method returns the number of objects in the conveyor.
     * Note: Conveyor can only keep track of the number of objects if it has entrance and exit sensors. Otherwise,
     * the number of objects returned will not be correct.
     *
     * @return number of objects in the conveyor.
     */
    public int getNumObjects()
    {
        return numObjects;
    }   //getNumObjects

    /**
     * This method sets the number of preloaded objects in the conveyor.
     *
     * @param num specifies the number of preloaded objects in the conveyor.
     */
    public void setPreloadedObjects(int num)
    {
        this.numObjects = num;
    }   //setPreloadedObjects

    public void registerEntranceEventHandler(TrcNotifier.Receiver handler)
    {
        if (entranceTrigger != null)
        {
            entranceEventHandler = handler;
        }
    }   //registerEntranceEventHandler

    public void registerExitEventHandler(TrcNotifier.Receiver handler)
    {
        if (exitTrigger != null)
        {
            exitEventHandler = handler;
        }
    }   //registerExitEventHandler

    private void entranceEvent(boolean active)
    {
        if (active)
        {
            numObjects++;
            setPower(params.movePower);
        }
        else
        {
            setPower(0.0);
        }

        if (entranceEventHandler != null)
        {
            entranceEventHandler.notify(active);
        }
}   //entranceEvent

    private void exitEvent(boolean active)
    {
        if (active)
        {
            numObjects--;
        }
        else
        {
            setPower(0.0);
        }

        if (exitEventHandler != null)
        {
            exitEventHandler.notify(active);
        }
}   //exitEvent

    public void advance(String owner, int units, TrcEvent event, TrcNotifier.Receiver callback)
    {
        if (validateOwnership(owner))
        {
            if (event != null)
            {
                event.clear();
            }
            this.onFinishEvent = event;
            this.onFinishCallback = callback;
            setTarget(units*params.objectDistance, false, event);
            setTaskEnabled(true);
        }
    }   //advance

    public void advance(int units)
    {
        advance(null, units, null, null);
    }   //advance

    public void advance()
    {
        advance(null, 1, null, null);
    }   //advance

    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            advanceTaskObj.registerTask(TaskType.OUTPUT_TASK);
        }
        else
        {
            advanceTaskObj.unregisterTask(TaskType.OUTPUT_TASK);
        }
    }   //setTaskEnabled

    private void advanceTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (onTarget())
        {
            if (onFinishEvent != null)
            {
                onFinishEvent.signal();
                onFinishEvent = null;
            }

            if (onFinishCallback != null)
            {
                onFinishCallback.notify(null);
                onFinishCallback = null;
            }

            advanceTaskObj.unregisterTask(TaskType.OUTPUT_TASK);
        }
    }   //advanceTask

    private boolean onTarget()
    {
        return true;
    }   //onTarget

    // /**
    //  * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
    //  * and it will stop itself once object is picked up or dumped in the intake at which time the given event will be
    //  * signaled or it will notify the caller's handler.
    //  *
    //  * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
    //  * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
    //  *              power to dump.
    //  * @param event specifies the event to signal when object is detected in the intake.
    //  * @param callback specifies the callback handler to call when object is detected in the intake.
    //  * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
    //  *                must call hasObject() to figure out if it has given up.
    //  */
    // public synchronized void autoAssist(
    //     String owner, double power, TrcEvent event, TrcNotifier.Receiver callback, double timeout)
    // {
    //     final String funcName = "autoAssist";

    //     if (debugEnabled)
    //     {
    //         dbgTrace.traceEnter(
    //             funcName, TrcDbgTrace.TraceLevel.API, "owner=%s,power=%.1f,event=%s,timeout=%.3f",
    //             owner, power, event, timeout);
    //     }

    //     if (sensorTrigger == null || power == 0.0)
    //     {
    //         throw new RuntimeException("Must have sensor and non-zero power to perform AutoAssist.");
    //     }
    //     //
    //     // This is an auto-assist operation, make sure the caller has ownership.
    //     //
    //     if (validateOwnership(owner))
    //     {
    //         if (power > 0.0 ^ hasObject())
    //         {
    //             // Picking up object and we don't have one yet, or dumping object and we still have one.
    //             if (params.msgTracer != null)
    //             {
    //                 params.msgTracer.traceInfo(
    //                     funcName, "owner=%s, power=%.1f, event=%s, timeout=%.3f", owner, power, event, timeout);
    //             }
    //             motor.set(power);
    //             this.onFinishEvent = event;
    //             this.onFinishCallback = callback;
    //             if (timeout > 0.0)
    //             {
    //                 timer.set(timeout, this::timeoutHandler);
    //             }
    //             sensorTrigger.setEnabled(true);
    //             this.autoAssistPower = power;
    //         }
    //         else
    //         {
    //             // Picking up object but we already have one, or dumping object but there isn't any.
    //             if (params.msgTracer != null)
    //             {
    //                 params.msgTracer.traceInfo(funcName, "Already done: hasObject=%s", hasObject());
    //             }

    //             if (event != null)
    //             {
    //                 event.signal();
    //             }

    //             if (callback != null)
    //             {
    //                 callback.notify(null);
    //             }
    //         }
    //     }
    // }   //autoAssist

    // /**
    //  * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
    //  * and it will stop itself once object is picked up or dumped in the intake at which time the given event will be
    //  * signaled or it will notify the caller's handler.
    //  *
    //  * @param owner specifies the owner ID to check if the caller has ownership of the intake subsystem.
    //  * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
    //  *              power to dump.
    //  */
    // public void autoAssist(String owner, double power)
    // {
    //     autoAssist(owner, power, null, null, 0.0);
    // }   //autoAssist

    // /**
    //  * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
    //  * and it will stop itself once object is picked up or dumped in the intake at which time the given event will be
    //  * signaled or it will notify the caller's handler.
    //  *
    //  * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
    //  *              power to dump.
    //  * @param event specifies the event to signal when object is detected in the intake.
    //  * @param callback specifies the callback handler to call when object is detected in the intake.
    //  * @param timeout specifies a timeout value at which point it will give up and signal completion. The caller
    //  *                must call hasObject() to figure out if it has given up.
    //  */
    // public void autoAssist(double power, TrcEvent event, TrcNotifier.Receiver callback, double timeout)
    // {
    //     autoAssist(null, power, event, callback, timeout);
    // }   //autoAssist

    // /**
    //  * This method is an auto-assist operation. It allows the caller to start the intake spinning at the given power
    //  * and it will stop itself once object is picked up or dumped in the intake at which time the given event will be
    //  * signaled or it will notify the caller's handler.
    //  *
    //  * @param power specifies the power value to spin the intake. It assumes positive power to pick up and negative
    //  *              power to dump.
    //  */
    // public void autoAssist(double power)
    // {
    //     autoAssist(null, power, null, null, 0.0);
    // }   //autoAssist

    /**
     * This method returns the sensor state read from the digital sensor.
     *
     * @return digital sensor state.
     */
    public boolean isEntranceSensorActive()
    {
        return entranceSensor != null && entranceSensor.isActive();
    }   //isEntranceSensorActive

    /**
     * This method returns the sensor state read from the digital sensor.
     *
     * @return digital sensor state.
     */
    public boolean isExitSensorActive()
    {
        return exitSensor != null && exitSensor.isActive();
    }   //isExitSensorActive

}   //class TrcPidConveyor
