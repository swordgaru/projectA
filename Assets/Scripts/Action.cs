using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Sion.Action
{
    public class Action
    {
        public Action()
        {

        }

        protected IAccessHandle<ActionRunner> runnerHandle;

        public ActionRunner runner => runnerHandle?.instance;

        private Action _chain;

        public Action chain
        {
            get { return _chain; }
            set { _chain = value; }
        }

        public Action tail
        {
            get
            {
                for (var act = this; true; act = act._chain)
                    if (act._chain == null)
                        return act;
            }
        }

        public bool stopOnInterrupt
        {
            get { return statusFlags.Any((uint)StatusFlag.StopOnInterrupt); }
            set
            {
                if (interrupted)
                    throw new InvalidOperationException($"Can't change {nameof(stopOnInterrupt)} on interrupt");

                statusFlags.Set((uint)StatusFlag.StopOnInterrupt, value);
            }
        }

        public bool stopOnException
        {
            get { return statusFlags.Any((uint)StatusFlag.StopOnException); }
            set { statusFlags.Set((uint)StatusFlag.StopOnException, value); }
        }

        public bool interruptOnReentry
        {
            get { return statusFlags.Any((uint)StatusFlag.InterruptOnReentry); }
            set { statusFlags.Set((uint)StatusFlag.InterruptOnReentry, value); }
        }

        #region IActivityContextAccess

        public virtual T GetContext<T>() where T : class, IActionContext
            => GetInternalContext<T>() ?? GetExternalContext<T>();

        protected virtual T GetInternalContext<T>() where T : class, IActionContext
            => this as T;

        private T GetExternalContext<T>() where T : class, IActionContext
        {
            if (externalContexts != null)
            {
                foreach (var ctx in externalContexts)
                {
                    var context = ctx as T;
                    if (context != null)
                        return context;
                }
            }

            return null;
        }

        public T AttachContext<T>(T context) where T : class, IActionContext
        {
            if (GetContext<T>() != null)
                throw new ArgumentException($"Context of {typeof(T).Name} already attached.");

            externalContexts = externalContexts ?? new List<IActionContext>();
            externalContexts.Add(context);

            return context;
        }

        public bool DetachContext<T>(T context) where T : class, IActionContext
            => externalContexts != null ? externalContexts.Remove(context) : false;

        private List<IActionContext> externalContexts;
        #endregion

        protected BitSet32 statusFlags;

        [Flags]
        protected enum StatusFlag : uint
        {
            Preparing   = 0x01,
            Prepared    = 0x02,
            EverStarted = 0x04,
            Starting    = 0x08,
            Running     = 0x10,
            Stopped     = 0x20,
            Interrupted = 0x40,

            StopOnInterrupt = 0x100,
            StopOnException = 0x200,
            InterruptOnReentry = 0x400,
        }

        public bool prepared => statusFlags.Any((uint)StatusFlag.Prepared);
        public bool everStarted => statusFlags.Any((uint)StatusFlag.EverStarted);
        public bool starting => statusFlags.Any((uint)StatusFlag.Starting);
        public bool running => statusFlags.Any((uint)StatusFlag.Running);
        public bool interrupted => statusFlags.Any((uint)StatusFlag.Interrupted);
        public bool stopped => statusFlags.Any((uint)StatusFlag.Stopped);

        public delegate void PrepareCallback(Control control, Action previous);
        public delegate void StartCallback(Control control);
        public delegate void TickCallback(Control control, float deltaTime);
        public delegate void InteruptCallback(Control control, Action by);
        public delegate void StopCallback(Control control);

        public class BasicHandlers
        {
            public PrepareCallback onPrepare;
            public StartCallback onStart;
            public TickCallback onTick;
            public InteruptCallback onInterrupt;
            public StopCallback onStop;
        }

        BasicHandlers basicHandlers;

        BasicHandlers EnsureBasicHandlers()
        {
            if(statusFlags.Any((uint)(StatusFlag.Preparing | StatusFlag.Prepared)))
                throw new InvalidOperationException($"Can't modify handlers during and after prepare.");

            return basicHandlers = basicHandlers ?? new BasicHandlers();
        }

        public event PrepareCallback onPrepare
        {
            add { EnsureBasicHandlers().onPrepare += value; }
            remove { EnsureBasicHandlers().onPrepare -= value; }
        }

        public event StartCallback onStart
        {
            add { EnsureBasicHandlers().onStart += value; }
            remove { EnsureBasicHandlers().onStart -= value; }
        }

        public event TickCallback onTick
        {
            add { EnsureBasicHandlers().onTick += value; }
            remove { EnsureBasicHandlers().onTick -= value; }
        }

        public event InteruptCallback onInterrupt
        {
            add { EnsureBasicHandlers().onInterrupt += value; }
            remove { EnsureBasicHandlers().onInterrupt -= value; }
        }

        public event StopCallback onStop
        {
            add { EnsureBasicHandlers().onStop += value; }
            remove { EnsureBasicHandlers().onStop -= value; }
        }

        public struct Control : IActionContextAccess
        {
            private Action action;

            internal Control(Action action) { this.action = action; }

            public bool prepared => action.prepared;
            public bool everStarted => action.everStarted;
            public bool starting => action.starting;
            public bool running => action.running;
            public bool interrupted => action.interrupted;
            public bool stopped => action.stopped;

            public void Stop() => action.Stop();

            public void Chain(params Action[] nexts)
                => action.Chain(nexts);

            #region IActivityContextAccess
            public T GetContext<T>() where T : class, IActionContext
                => action.GetContext<T>();
            #endregion
        }

        public Action Chain(params Action[] nexts)
        {
            var act = this;
            foreach (var next in nexts)
            {
                act.chain = next;
                act = next;
            }

            return act;
        }

        #region ActivityRunner operations

        protected void CheckHandle(IAccessHandle<ActionRunner> handle)
        {
            if (handle == null)
                throw new InvalidOperationException();

            if (runnerHandle != null && runnerHandle != handle)
                throw new InvalidOperationException();
        }

        public void Prepare(IAccessHandle<ActionRunner> handle, Action previous)
        {
            if (running)
                throw new InvalidOperationException("Tried to prepare while running!");

            CheckHandle(handle);

            statusFlags.Clear((uint)(StatusFlag.Prepared | StatusFlag.Stopped));

            // NOTE:
            // Should not catch-and-rethrow here,
            // otherwise will throw 'Tried to stop while preparing' instead of real exception.

            try
            {
                statusFlags.Set((uint)StatusFlag.Preparing);

                runnerHandle = handle;

                DoPrepare(previous);

                statusFlags.Set((uint)StatusFlag.Prepared);
            }
            finally
            {
                statusFlags.Clear((uint)StatusFlag.Preparing);
            }
        }

        protected void Start()
        {
            try
            {
                DoStart();

                statusFlags.Set((uint)(StatusFlag.Running | StatusFlag.EverStarted));
                statusFlags.Clear((uint)(StatusFlag.Starting | StatusFlag.Stopped | StatusFlag.Interrupted));
            }
            catch
            {
                if (stopOnException) Stop();
                throw;
            }
        }

        public void Tick(IAccessHandle<ActionRunner> handle, float dt)
        {
            CheckHandle(handle);

            try
            {
                if (!running)
                {
                    bool ready;
                    DoStarting(dt, out ready);

                    if (ready)
                        Start();
                    else
                        statusFlags.Set((uint)StatusFlag.Starting);
                }

                if (running)
                    DoTick(dt);
            }
            catch
            {
                if (stopOnException) Stop();
                throw;
            }
        }

        public void Stop(IAccessHandle<ActionRunner> handle)
        {
            CheckHandle(handle);

            Stop();
        }

        public void InterruptBy(IAccessHandle<ActionRunner> handle, Action interrupter)
        {
            CheckHandle(handle);

            InterruptBy(interrupter);
        }

        #endregion

        #region Controller functionality

        protected void Stop()
        {
            if (statusFlags.Any((uint)StatusFlag.Preparing))
                throw new InvalidOperationException("Can't stop while preparing!");

            bool validWhen = statusFlags.Any((uint)
                (StatusFlag.Preparing | StatusFlag.Prepared | StatusFlag.Starting | StatusFlag.Running | StatusFlag.Interrupted));

            if (!validWhen)
                return;

            // NOTE:
            // Should not catch-and-rethrow here,
            // otherwise will run infinite stopOnException -> Stop() chain!
            // Here we don't need to Stop() on exception anyway :)

            try
            {
                statusFlags.Set((uint)StatusFlag.Stopped);

                DoStop();
            }
            finally
            {
                statusFlags.Clear((uint)(StatusFlag.Starting | StatusFlag.Running | StatusFlag.Interrupted));
                runnerHandle = null;
            }
        }

        private void InterruptBy(Action interrupter)
        {
            if (stopOnInterrupt)
            {
                Stop();
                return;
            }

            bool validWhen = statusFlags.Any((uint)
                (StatusFlag.Prepared | StatusFlag.Starting | StatusFlag.Running));

            if (!validWhen)
                return;

            try
            {
                statusFlags.Set((uint)StatusFlag.Interrupted);

                DoInterruptBy(interrupter);
            }
            catch
            {
                if (stopOnException) Stop();
                throw;
            }
            finally
            {
                statusFlags.Clear((uint)(StatusFlag.Starting | StatusFlag.Running));
                runnerHandle = null;
            }
        }

        protected virtual void DoPrepare(Action previous)
            => basicHandlers?.onPrepare?.Invoke(new Control(this), previous);

        protected virtual void DoStarting(float dt, out bool ready)
            => ready = true;

        protected virtual void DoStart()
            => basicHandlers?.onStart?.Invoke(new Control(this));

        protected virtual void DoTick(float dt)
            => basicHandlers?.onTick?.Invoke(new Control(this), dt);

        protected virtual void DoStop()
            => basicHandlers?.onStop?.Invoke(new Control(this));

        protected virtual void DoInterruptBy(Action interrupter)
            => basicHandlers?.onInterrupt?.Invoke(new Control(this), interrupter);

        #endregion
    }
}
