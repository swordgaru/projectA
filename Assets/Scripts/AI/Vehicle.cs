using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ting.AI
{
    public class Vehicle : MovingEntity
    {
        public GameWorld world;
        public SteeringBehavior steering;

        double timeElapsed;

        GameObject vehicleObject;
        public Vehicle(GameWorld world, Vector2D position, double rotation, Vector2D velocity, double mass, double maxForce, double maxSpeed, double maxTurnRate, double scale) 
            : base(position, scale, velocity, maxSpeed, new Vector2D(System.Math.Sin(rotation), -System.Math.Cos(rotation)), mass, new Vector2D(scale, scale), maxTurnRate, maxForce)
        {
            this.world = world;
            timeElapsed = 0;

            steering = new SteeringBehavior(this);

            //vehicleObject = new GameObject($"Vehicle {id}");
            //vehicleObject = GameObject.CreatePrimitive(PrimitiveType.Plane);
            //vehicleObject.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            //vehicleObject.name = $"Vehicle {id}";

            vehicleObject = new GameObject($"Vehicle {id}");
            vehicleObject.transform.localScale = new Vector3((float)scale, (float)scale, (float)scale);
            EntityViewer entityViewer = vehicleObject.AddComponent<EntityViewer>();
            entityViewer.CreateTriange();

            AITick.Instance.vehicleObjects.Add(vehicleObject);
        }
        
        public double TimeElapsed()
        {
            return timeElapsed;
        }

        public void Update(double time_elapsed)
        {
            //update the time elapsed
            timeElapsed = time_elapsed;

            //keep a record of its old position so we can update its cell later
            //in this method
            Vector2D OldPos = pos;

            Vector2D SteeringForce;

            //calculate the combined force from each steering behavior in the 
            //vehicle's list
            SteeringForce = steering.Calculate();

            //Acceleration = Force/Mass
            Vector2D acceleration = Vector2D.div(SteeringForce, mass);

            //update velocity
            velocity.add(Vector2D.mul(acceleration, time_elapsed));

            //make sure vehicle does not exceed maximum velocity
            velocity.Truncate(maxSpeed);

            //update the position
            pos.add(Vector2D.mul(velocity, time_elapsed));

            //update the heading if the vehicle has a non zero velocity
            if (velocity.LengthSq() > 0.00000001)
            {
                heading = Vector2D.Vec2DNormalize(velocity);

                side = heading.Perp();
            }

            //EnforceNonPenetrationConstraint(this, World()->Agents());

            //treat the screen as a toroid
            Vector2D.WrapAround(pos, world.cx, world.cy);
        }

        public double getTimeElapsed()
        {
            return timeElapsed;
        }

        public void Render()
        {
            //a vector to hold the transformed vertices
            List<Vector2D> m_vecVehicleVBTrans;

            vehicleObject.transform.position = new Vector3((float)pos.x, 0, (float)pos.y);
            vehicleObject.transform.forward = new Vector3((float)heading.x, 0, (float)heading.y);
                //m_vecVehicleVBTrans = Transformation.WorldTransform(vecVehicleVB,
                //        Pos(),
                //        Heading(),
                //        Side(),
                //        Scale());
            
            //steering.RenderAids();
        }
    }
}