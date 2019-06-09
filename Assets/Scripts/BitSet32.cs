using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Sion.Action
{
    public struct BitSet32
    {
        public uint bits;

        public BitSet32(uint bits)
            => this.bits = bits;

        public bool Any(uint flags)
            => (bits & flags) != (uint)0;

        public void Set(uint flags)
            => bits |= flags;

        public void Set(uint flags, bool set = true)
            => bits = set ? (bits | flags) : (bits & ~flags);

        public void Clear(uint flags)
            => bits &= ~flags;
    }
}