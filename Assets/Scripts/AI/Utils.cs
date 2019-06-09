using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ting.AI
{
    public interface INumber<T>
    {
        T GetNumber();
        T Sub<T2>(T2 t2);
    }

    public class Utils
    {
        public const double Pi = Math.PI;
        public const double TwoPi = Math.PI * 2;
        public const double HalfPi = Math.PI / 2;
        public const double QuarterPi = Math.PI / 4;

        //public static T MaxOf<T>(T a, T b) where T : class, IComparable
        //{
        //    if (a.CompareTo(b) > 0)
        //        return a;
        //    return b;
        //}
        public static double MaxOf(double a, double b)
        {
            if (a.CompareTo(b) > 0)
                return a;
            return b;
        }

        public static T MinOf<T>(T a, T b) where T : class, IComparable
        {
            if (a.CompareTo(b) < 0)
                return a;
            return b;
        }

        public static T Clamp<T>(T arg, T min, T max) where T : INumber<double>
        {
            if (arg.GetNumber() < min.GetNumber())
                return min;

            if (arg.GetNumber() > max.GetNumber())
                return max;

            return arg;
        }

        //public static bool isEqual<T>(T a, T b) where T : INumber<double>
        //{
        //    if (System.Math.Abs(a.Sub(b)) < Double.Epsilon)
        //    {
        //        return true;
        //    }
        //    return false;
        //}

        public static bool isEqual(double a, double b)
        {
            if (System.Math.Abs(a - b) < Double.Epsilon)
            {
                return true;
            }
            return false;
        }

        public static bool isNaN<T>(T val)
        {
            return !(val != null);
        }

        static public double DegsToRads(double degs)
        {
            return TwoPi * (degs / 360.0);
        }

        //compares two real numbers. Returns true if they are equal
        static public bool isEqual(float a, float b)
        {
            if (Math.Abs(a - b) < 1E-12)
            {
                return true;
            }

            return false;
        }

        static private System.Random rand = new System.Random();
        static private Int16 RAND_MAX = 0x7fff;

        //returns a random integer between x and y
        static public int RandInt(int x, int y)
        {
            Debug.Assert(y >= x, "<RandInt>: y is less than x");
            return rand.Next(int.MaxValue - x) % (y - x + 1) + x;
        }

        //returns a random double between zero and 1
        static public double RandFloat()
        {
            return rand.NextDouble() / (RAND_MAX + 1.0);
        }

        static public double RandInRange(double x, double y)
        {
            return x + RandFloat() * (y - x);
        }

        //returns a random bool
        static public bool RandBool()
        {
            if (RandFloat() > 0.5)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        //returns a random double in the range -1 < n < 1
        static public double RandomClamped()
        {
            return RandFloat() - RandFloat();
        }

        //returns a random number with a normal distribution. See method at
        //http://www.taygeta.com/random/gaussian.html
        static public double RandGaussian()
        {
            return RandGaussian(0, 1);
        }
        static private double y2 = 0;
        static private bool use_last = false;

        static public double RandGaussian(double mean, double standard_deviation)
        {

            double x1, x2, w, y1;

            if (use_last) /* use value from previous call */
            {
                y1 = y2;
                use_last = false;
            }
            else
            {
                do
                {
                    x1 = 2.0 * RandFloat() - 1.0;
                    x2 = 2.0 * RandFloat() - 1.0;
                    w = x1 * x1 + x2 * x2;
                } while (w >= 1.0);

                w = Math.Sqrt((-2.0 * Math.Log(w)) / w);
                y1 = x1 * w;
                y2 = x2 * w;
                use_last = true;
            }

            return (mean + y1 * standard_deviation);
        }

        //-----------------------------------------------------------------------
        //  
        //  some handy little functions
        //-----------------------------------------------------------------------
        public static double Sigmoid(double input)
        {
            return Sigmoid(input, 1.0);
        }

        public static double Sigmoid(double input, double response)
        {
            return (1.0 / (1.0 + Math.Exp(-input / response)));
        }

        // Gaussian
        // https://kin.naver.com/qna/detail.nhn?d1id=11&dirId=110812&docId=50873962&qb=R2F1c3NpYW4=&enc=utf8&section=kin&rank=3&search_sort=0&spq=0&pid=UnVGGspVuF4ssZVrE2Rssssstmh-193037&sid=m/BTyN5LF9I/ZXdDtjno7A%3D%3D
    }
}