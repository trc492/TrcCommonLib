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
import java.util.Iterator;

/**
 * This class implements the TrcEvent. TrcEvent is very important in our event driven architecture where things
 * only happen when an event is signaled.
 */
public class TrcEvent
{
    private static final String moduleName = "TrcEvent";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * An event has three possible states:
     *  - CLEARED: event should be in this state before starting an asynchronous operation. This is also the default
     *             state when an event is created.
     *  - SIGNALED: when an asynchronous operation is completed, the event is set to this state.
     *  - CANCELED: when an asynchronous operation is canceled, the event is set to this state.
     */
    public enum EventState
    {
        CLEARED,
        SIGNALED,
        CANCELED
    }   //enum EventState

    /**
     * This interface is implemented by the caller so that it can be notified when the event is signaled.
     */
    public interface Callback
    {
        void notify(Object context);
    }   //interface Callback

    private static ArrayList<TrcEvent> eventCallbackList = new ArrayList<>();
    private static TrcTaskMgr.TaskObject eventCallbackTaskObj = null;

    private final String instanceName;
    private volatile EventState eventState;
    private Callback callback;
    private Object callbackContext;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param eventState specifies the initial state of the event.
     */
    public TrcEvent(String instanceName, EventState eventState)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.eventState = eventState;
    }   //TrcEvent

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcEvent(String instanceName)
    {
        this(instanceName, EventState.CLEARED);
    }   //TrcEvent

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return String.format("Event:%s=%s", instanceName, eventState);
    }   //toString

    /**
     * This method clears an event.
     */
    public synchronized void clear()
    {
        final String funcName = "clear";

        eventState = EventState.CLEARED;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, " (%s)", eventState);
        }
    }   //clear

    /**
     * This method signals an event if it is not canceled.
     */
    public synchronized void signal()
    {
        final String funcName = "signal";

        if (eventState != EventState.CANCELED)
        {
            eventState = EventState.SIGNALED;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, " (%s)", eventState);
        }
    }   //signal

    /**
     * This method cancels an event if it is not already signaled. An event is either signaled or canceled by the
     * event source either of which will cause whoever is waiting for it to move on.
     */
    public synchronized void cancel()
    {
        final String funcName = "cancel";

        if (eventState == EventState.CLEARED)
        {
            eventState = EventState.CANCELED;
        }

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, " (%s)", eventState);
        }
    }   //cancel

    /**
     * This method checks if the event is signaled.
     *
     * @return true if the event is signaled, false otherwise.
     */
    public synchronized boolean isSignaled()
    {
        final String funcName = "isSignaled";
        boolean signaled = eventState == EventState.SIGNALED;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", signaled);
        }

        return signaled;
    }   //isSignaled

    /**
     * This method checks if the event was canceled.
     *
     * @return true if the event was canceled, false otherwise.
     */
    public synchronized boolean isCanceled()
    {
        final String funcName = "isCanceled";
        boolean canceled = eventState == EventState.CANCELED;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", canceled);
        }

        return canceled;
    }   //isCanceled

    /**
     * This method sets a callback handler so that when the event is signaled, the callback handler is called on
     * the main robot thread. This could be very useful if the caller wants to perform some minor actions after an
     * asynchronous operation is completed. Without the callback, the caller would have to set up a state machine
     * waiting for the event to signal, then perform the action. Since the callback is done on the main robot thread,
     * the caller doesn't have to worry about thread safety with the main robot code.
     *
     * @param callback specifies the callback handler, null for removing previous callback handler.
     * @param context specifies the context object passing back to the callback handler.
     */
    public void setCallback(Callback callback, Object context)
    {
        clear();
        synchronized (eventCallbackList)
        {
            boolean inList = eventCallbackList.contains(this);

            this.callback = callback;
            this.callbackContext = context;
            if (callback != null && !inList)
            {
                // Callback handler is not already in the callback list, add it.
                eventCallbackList.add(this);
                if (eventCallbackTaskObj == null)
                {
                    // This is the first time a callback handler is set, create the event callback task on demand.
                    eventCallbackTaskObj = TrcTaskMgr.createTask("EventCallbackTask", TrcEvent::eventCallbackTask);
                }

                if (!eventCallbackTaskObj.isRegistered())
                {
                    // Enable the task if not already. Registering it as a SYSTEM_TASK will make it run on the main
                    // robot thread.
                    eventCallbackTaskObj.registerTask(TrcTaskMgr.TaskType.SYSTEM_TASK);
                }
            }
            else if (callback == null && inList)
            {
                // Remove the callback from the list. Note: if the list becomes empty, the task will disable itself.
                eventCallbackList.remove(this);
                this.callbackContext = null;
            }
        }
    }   //setCallback

    /**
     * This method is run periodically on the main robot thread. It checks all the events on the callback list. If
     * any of them are signaled or canceled, the callback will be issued.
     *
     * @param taskType specifies the task type to be SYSTEM_TASK (not used).
     * @param runMode specifies the current RunMode (not used).
     */
    private static void eventCallbackTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        synchronized (eventCallbackList)
        {
            // Use a list Iterator because it is fail-fast and allows removing entries while iterating the list.
            Iterator<TrcEvent> listIterator = eventCallbackList.iterator();

            while (listIterator.hasNext())
            {
                TrcEvent event = listIterator.next();
                if (event.isSignaled() || event.isCanceled())
                {
                    Callback callback = event.callback;
                    Object context = event.callbackContext;
                    // Clear the callback stuff before doing the callback since the callback may reuse and chain to
                    // another callback.
                    event.callback = null;
                    event.callbackContext = null;
                    callback.notify(context);
                    // Remove it from the list after the callback is issued.
                    listIterator.remove();
                }
            }

            if (eventCallbackList.isEmpty())
            {
                // The list is empty, disable the task.
                eventCallbackTaskObj.unregisterTask();
            }
        }
    }   //eventCallbackTask

}   //class TrcEvent
