using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Ting.AI
{
    public class GameWorld
    {
        private List<Vehicle> vehicles = new List<Vehicle>(AIConfig.NumAgents);
        public int cx { get; set; }
        public int cy { get; set; }

        public void TagVehiclesWithinViewRange(Entity pVehicle, double range)
        {
            var entityList = vehicles.OfType<Entity>().ToList();
            Entity.TagNeighbors(pVehicle, entityList, range);
            vehicles = entityList.OfType<Vehicle>().ToList();
        }

        public List<Vehicle> Agents()
        {
            return vehicles;
        }

        public GameWorld()
        {
            cx = 500;
            cy = 500;
            Entity.resetId();
            for (int a = 0; a < AIConfig.NumAgents; ++a)
            {
                //determine a random starting position
                Vector2D SpawnPos = new Vector2D(cx / 2.0 + Utils.RandomClamped() * cx / 2.0,
                        cy / 2.0 + Utils.RandomClamped() * cy / 2.0);


                Vehicle pVehicle = new Vehicle(this,
                        SpawnPos, //initial position
                        Utils.RandFloat() * Utils.TwoPi, //start rotation
                        new Vector2D(0, 0), //velocity
                        AIConfig.VehicleMass, //mass
                        AIConfig.SteeringForce * AIConfig.SteeringForceTweaker, //max force
                        AIConfig.MaxSpeed, //max velocity
                        AIConfig.MaxTurnRatePerSecond, //max turn rate
                        AIConfig.VehicleScale);        //scale

                pVehicle.steering.FlockingOn();

                vehicles.Add(pVehicle);
            }

            bool SHOAL = true;
            if (SHOAL)
            {
                vehicles[AIConfig.NumAgents - 1].steering.FlockingOff();
                //vehicles[AIConfig.NumAgents - 1].SetScale(new Vector2D(10, 10));
                vehicles[AIConfig.NumAgents - 1].steering.WanderOn();
                vehicles[AIConfig.NumAgents - 1].SetMaxSpeed(70);

                for (int i = 0; i < AIConfig.NumAgents - 1; ++i)
                {
                    vehicles[i].steering.EvadeOn(vehicles[AIConfig.NumAgents - 1]);
                }
            }
            //create any obstacles or walls
            //CreateObstacles();
            //CreateWalls();
        }

        public void Update(double time_elapsed)
        {         
            for (int a = 0; a < vehicles.Count; ++a)
            {
                vehicles[a].Update(time_elapsed);
            }
        }

        public void Render()
        {
            //render the agents
            for (int a = 0; a < vehicles.Count; ++a)
            {
                vehicles[a].Render();
            }          
        }
    }
}