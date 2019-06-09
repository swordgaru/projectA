using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Sion.Action
{
    public interface IActionRunner<TAction> where TAction : Action
    {
        TAction current { get; set; }
        TAction @default { get; set; }
        bool TrySetCurrent(TAction action);
    }

    public interface IActionContext
    {

    }

    public interface IActionContextAccess
    {
        T GetContext<T>() where T : class, IActionContext;
    }

    public class ActionRunner : IActionRunner<Action>, IActionContextAccess
    {
        private Action _current;
        private Action _lat;

        public Action current
        {
            get
            {
                throw new System.NotImplementedException();
            }

            set
            {
                throw new System.NotImplementedException();
            }
        }

        private Action _default;    
        public Action @default
        {
            get { return _default; }
        }

        Action IActionRunner<Action>.@default
        {
            get
            {
                throw new System.NotImplementedException();
            }

            set
            {
                throw new System.NotImplementedException();
            }
        }

        public bool TrySetCurrent(Action action)
        {
            throw new System.NotImplementedException();
        }

        T IActionContextAccess.GetContext<T>()
        {
            throw new System.NotImplementedException();
        }
    }
}