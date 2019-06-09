using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ting.AI
{
    public class MovingEntity : Entity
    {
        public Vector2D velocity;
        public Vector2D heading;
        public Vector2D side;

        public double mass;
        public double maxSpeed;
        public double maxForce;
        public double maxTurnRate;

        public MovingEntity(Vector2D position, double radius, Vector2D velocity, double maxSpeed, Vector2D heading, double mass,
            Vector2D scale, double turnRate, double maxForce): base(0, position, radius)
        {
            this.heading = heading;
            this.velocity = velocity;
            this.mass = mass;
            this.side = heading.Perp();
            this.maxSpeed = maxSpeed;
            this.maxTurnRate = turnRate;
            this.maxForce = maxForce;
        }

        public bool RotateHeadingToFacePosition(Vector2D target)
        {
            Vector2D toTarget = Vector2D.Vec2DNormalize(Vector2D.sub(target, pos));

            //first determine the angle between the heading vector and the target
            double angle = System.Math.Acos(heading.Dot(toTarget));
            if (double.IsNaN(angle))
            {
                angle = 0;
            }

            //return true if the player is facing the target
            if (angle < 0.00001)
            {
                return true;
            }

            //clamp the amount to turn to the max turn rate
            if (angle > maxTurnRate)
            {
                angle = maxTurnRate;
            }

            //The next few lines use a rotation matrix to rotate the player's heading
            //vector accordingly
            C2DMatrix RotationMatrix = new C2DMatrix();

            //notice how the direction of rotation has to be determined when creating
            //the rotation matrix
            RotationMatrix.Rotate(angle * heading.Sign(toTarget));
            RotationMatrix.TransformVector2Ds(heading);
            RotationMatrix.TransformVector2Ds(velocity);

            //finally recreate m_vSide
            side = heading.Perp();

            return false;
        }

        void SetHeading(Vector2D new_heading)
        {
            Debug.Assert((new_heading.LengthSq() - 1.0) < 0.00001);

            heading = new_heading;

            //the side vector must always be perpendicular to the heading
            side = heading.Perp();
        }

        public double Speed()
            => velocity.Length();

        public void SetMaxForce(double mf)
            => maxForce = mf;

        public void SetMaxSpeed(double new_speed)
        => maxSpeed = new_speed;
    }
}