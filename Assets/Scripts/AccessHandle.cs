using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Sion.Action
{
    public interface IAccessHandle<T> where T : class
    {
        T instance { get; }
    }

    public class WeakAccessHandle<T> : IAccessHandle<T> where T : class
    {
        protected WeakAccessHandle(T instance)
        {
            weakRef = new WeakReference<T>(instance);
        }

        protected WeakReference<T> weakRef;
        public T instance => GetInstance();
        
        private T GetInstance()
        {
            T result;
            weakRef.TryGetTarget(out result);
            return result;
        }
    }

}