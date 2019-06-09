using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ting.AI
{
    public class Vector2D
    {
        public double x;
        public double y;

        public Vector2D()
        {
            x = 0.0f;
            y = 0.0f;
        }

        public Vector2D(double x, double y)
        {
            this.x = x;
            this.y = y;
        }

        public Vector2D(Vector2D v) : base()
        {
            this.Set(v);
        }

        public Vector2D Set(Vector2D v)
        {
            this.x = v.x;
            this.y = v.y;
            return this;
        }

        public void Zero()
        {
            x = 0.0f;
            y = 0.0f;
        }

        public bool IsZero()
        {
            return (x * x + y * y) < double.MinValue;
        }

        public double Length()
        {
            return System.Math.Sqrt(x * x + y * y);
        }

        public double LengthSq()
        {
            return (x * x + y * y);
        }

        public void Normalize()
        {
            double vectorLength = this.Length();

            if (vectorLength > double.Epsilon)
            {
                this.x /= vectorLength;
                this.y /= vectorLength;
            }
        }

        public double Dot(Vector2D v)
        {
            return x * v.x + y * v.y;
        }

        public const int clockwise = 1;
        public const int antiClockwise = -1;

        public int Sign(Vector2D v)
        {
            if (y * v.x > x * v.y)
            {
                return antiClockwise;
            }
            else
            {
                return clockwise;
            }
        }

        public Vector2D Perp()
           => new Vector2D(-y, x);

        public void Truncate(double max)
        {
            if (this.Length() > max)
            {
                this.Normalize();
                this.mul(max);
            }
        }

        public double Distance(Vector2D v)
        {
            double ySeparation = v.y - y;
            double xSeparation = v.x - x;

            return System.Math.Sqrt(ySeparation * ySeparation + xSeparation * xSeparation);
        }

        public double DistanceSq(Vector2D v)
        {
            double ySeparation = v.y - y;
            double xSeparation = v.x - x;

            return ySeparation * ySeparation + xSeparation * xSeparation;
        }

        public void Reflect(Vector2D norm)
        {
            this.add(norm.GetReverse().mul(2.0 * Dot(norm)));
        }

        public Vector2D GetReverse()
        {
            return new Vector2D(-this.x, -this.y);
        }

        public Vector2D add(Vector2D rhs)
        {
            x += rhs.x;
            y += rhs.y;

            return this;
        }

        public Vector2D sub(Vector2D rhs)
        {
            x -= rhs.x;
            y -= rhs.y;

            return this;
        }

        public Vector2D mul(double rhs)
        {
            x *= rhs;
            y *= rhs;

            return this;
        }

        public Vector2D div(double rhs)
        {
            x /= rhs;
            y /= rhs;

            return this;
        }

        public bool isEqual(Vector2D rhs)
        {
            return System.Math.Abs(x - rhs.x) < double.Epsilon && System.Math.Abs(y - rhs.y) < double.Epsilon;
        }

        public bool notEqual(Vector2D rhs)
        {
            return (x != rhs.x) || (y != rhs.y);
        }

        static public Vector2D mul(Vector2D lhs, double rhs)
        {
            Vector2D result = new Vector2D(lhs);
            result.mul(rhs);
            return result;
        }

        static public Vector2D mul(double lhs, Vector2D rhs)
        {
            Vector2D result = new Vector2D(rhs);
            result.mul(lhs);
            return result;
        }

        static public Vector2D sub(Vector2D lhs, Vector2D rhs)
        {
            Vector2D result = new Vector2D(lhs);
            result.x -= rhs.x;
            result.y -= rhs.y;

            return result;
        }

        static public Vector2D add(Vector2D lhs, Vector2D rhs)
        {
            Vector2D result = new Vector2D(lhs);
            result.x += rhs.x;
            result.y += rhs.y;

            return result;
        }

        static public Vector2D div(Vector2D lhs, double val)
        {
            Vector2D result = new Vector2D(lhs);
            result.x /= val;
            result.y /= val;

            return result;
        }

        public static void WrapAround(Vector2D pos, int MaxX, int MaxY)
        {
            if (pos.x > MaxX)
            {
                pos.x = 0.0;
            }

            if (pos.x < 0)
            {
                pos.x = (double)MaxX;
            }

            if (pos.y < 0)
            {
                pos.y = (double)MaxY;
            }

            if (pos.y > MaxY)
            {
                pos.y = 0.0;
            }
        }

        public static Vector2D Vec2DNormalize(Vector2D v)
        {
            Vector2D vec = new Vector2D(v);

            double vector_length = vec.Length();

            if (vector_length > double.Epsilon)
            {
                vec.x /= vector_length;
                vec.y /= vector_length;
            }

            return vec;
        }


        public static double Vec2DDistance(Vector2D v1, Vector2D v2)
        {

            double ySeparation = v2.y - v1.y;
            double xSeparation = v2.x - v1.x;

            return System.Math.Sqrt(ySeparation * ySeparation + xSeparation * xSeparation);
        }

        public static double Vec2DDistanceSq(Vector2D v1, Vector2D v2)
        {

            double ySeparation = v2.y - v1.y;
            double xSeparation = v2.x - v1.x;

            return ySeparation * ySeparation + xSeparation * xSeparation;
        }

        public static double Vec2DLength(Vector2D v)
        {
            return System.Math.Sqrt(v.x * v.x + v.y * v.y);
        }

        public static double Vec2DLengthSq(Vector2D v)
        {
            return (v.x * v.x + v.y * v.y);
        }
    }
}