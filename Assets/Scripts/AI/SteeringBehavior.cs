using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ting.AI
{
    public class SteeringBehavior
    {
        const double WanderRad = 1.2;
        //distance the wander circle is projected in front of the agent
        const double WanderDist = 2.0;
        //the maximum amount of displacement along the circle each frame
        const double WanderJitterPerSec = 80.0;

        //used in path following
        const double WaypointSeekDist = 20;

        enum summing_method { weighted_average, prioritized, dithered };

        enum behavior_type
        {
            none = 0x00000,
            seek = 0x00002,
            flee = 0x00004,
            arrive = 0x00008,
            wander = 0x00010,
            cohesion = 0x00020,
            separation = 0x00040,
            allignment = 0x00080,
            obstacle_avoidance = 0x00100,
            wall_avoidance = 0x00200,
            follow_path = 0x00400,
            pursuit = 0x00800,
            evade = 0x01000,
            interpose = 0x02000,
            hide = 0x04000,
            flock = 0x08000,
            offset_pursuit = 0x10000,
        };

        Vehicle vehicle;

        //the steering force created by the combined effect of all
        //the selected behaviors
        Vector2D steeringForce = new Vector2D(0, 0);

        //these can be used to keep track of friends, pursuers, or prey
        Vehicle targetAgent1;
        Vehicle targetAgent2;

        //the current target
        Vector2D target = new Vector2D(0, 0);

        //length of the 'detection box' utilized in obstacle avoidance
        //double dBoxLength;


        //a vertex buffer to contain the feelers rqd for wall avoidance  
        //List<Vector2D> feelers;

        Vector2D wanderTarget;

        //explained above
        double wanderJitter;
        double wanderRadius;
        double wanderDistance;


        //multipliers. These can be adjusted to effect strength of the  
        //appropriate behavior. Useful to get flocking the way you require
        //for example.
        double weightSeparation;
        double weightCohesion;
        double weightAlignment;
        double weightWander;
        double weightObstacleAvoidance;
        double weightWallAvoidance;
        double weightSeek;
        double weightFlee;
        double weightArrive;
        double weightPursuit;
        double weightOffsetPursuit;
        double weightInterpose;
        double weightHide;
        double weightEvade;
        double weightFollowPath;

        //how far the agent can 'see'
        double viewDistance;
        //double wallDetectionFeelerLength;

        //any offset used for formations or offset pursuit
        Vector2D offset;



        //binary flags to indicate whether or not a behavior should be active
        int flags;


        //Arrive makes use of these to determine how quickly a vehicle
        //should decelerate to its target
        enum Deceleration { slow = 3, normal = 2, fast = 1 };

        //default
        Deceleration deceleration;

        //what type of method is used to sum any active behavior
        summing_method summingMethod;


        //this function tests if a specific bit of m_iFlags is set
        bool On(behavior_type bt) { return (flags & (int)bt) == (int)bt; }

        public SteeringBehavior(Vehicle agent)
        {


            vehicle = agent;
            flags = 0;
            //dBoxLength = AIConfig.MinDetectionBoxLength;
            weightCohesion = AIConfig.CohesionWeight;
            weightAlignment = AIConfig.AlignmentWeight;
            weightSeparation = AIConfig.SeparationWeight;

            weightWander = AIConfig.WanderWeight;
            viewDistance = AIConfig.ViewDistance;
            //wallDetectionFeelerLength = AIConfig.WallDetectionFeelerLength;

            //feelers = new List<Vector2D>(3);

            deceleration = Deceleration.normal;

            targetAgent1 = null;
            targetAgent2 = null;
            wanderDistance = WanderDist;
            wanderJitter = WanderJitterPerSec;
            wanderRadius = WanderRad;

            weightSeek = AIConfig.SeekWeight;
            weightFlee = AIConfig.FleeWeight;
            weightArrive = AIConfig.ArriveWeight;
            weightPursuit = AIConfig.PursuitWeight;
            weightOffsetPursuit = AIConfig.OffsetPursuitWeight;
            weightInterpose = AIConfig.InterposeWeight;
            weightHide = AIConfig.HideWeight;
            weightEvade = AIConfig.EvadeWeight;

            summingMethod = summing_method.prioritized;
            
            //stuff for the wander behavior
            double theta = Utils.RandFloat() * Utils.TwoPi;

            //create a vector to a target position on the wander circle
            wanderTarget = new Vector2D(wanderRadius * System.Math.Cos(theta),
                                        wanderRadius * System.Math.Sin(theta));
        }

        //----------------------- Calculate --------------------------------------
        //
        //  calculates the accumulated steering force according to the method set
        //  in m_SummingMethod
        //------------------------------------------------------------------------
        public Vector2D Calculate()
        {
            //reset the steering force
            steeringForce.Zero();

            if (On(behavior_type.separation) || On(behavior_type.allignment) || On(behavior_type.cohesion))
            {
                vehicle.world.TagVehiclesWithinViewRange(vehicle, viewDistance);
            }

            steeringForce = CalculatePrioritized();

            return steeringForce;
        }


        //------------------------- ForwardComponent -----------------------------
        //
        //  returns the forward oomponent of the steering force
        //------------------------------------------------------------------------
        double ForwardComponent()
        {
            return vehicle.heading.Dot(steeringForce);
        }

        //--------------------------- SideComponent ------------------------------
        //  returns the side component of the steering force
        //------------------------------------------------------------------------
        double SideComponent()
        {
            return vehicle.side.Dot(steeringForce);
        }


        //--------------------- AccumulateForce ----------------------------------
        //
        //  This function calculates how much of its max steering force the 
        //  vehicle has left to apply and then applies that amount of the
        //  force to add.
        //------------------------------------------------------------------------
        bool AccumulateForce(Vector2D RunningTot, Vector2D ForceToAdd)
        {

            //calculate how much steering force the vehicle has used so far
            double MagnitudeSoFar = RunningTot.Length();

            //calculate how much steering force remains to be used by this vehicle
            double MagnitudeRemaining = vehicle.maxForce - MagnitudeSoFar;

            //return false if there is no more force left to use
            if (MagnitudeRemaining <= 0.0) return false;

            //calculate the magnitude of the force we want to add
            double MagnitudeToAdd = ForceToAdd.Length();

            //if the magnitude of the sum of ForceToAdd and the running total
            //does not exceed the maximum force available to this vehicle, just
            //add together. Otherwise add as much of the ForceToAdd vector is
            //possible without going over the max.
            if (MagnitudeToAdd < MagnitudeRemaining)
            {
                RunningTot.add(ForceToAdd);
            }

            else
            {
                //add it to the steering force
                RunningTot.add(Vector2D.mul(Vector2D.Vec2DNormalize(ForceToAdd), MagnitudeRemaining));
            }

            return true;
        }


        private Vector2D CalculatePrioritized()
        {
            Vector2D force = new Vector2D();

           
            if (On(behavior_type.evade))
            {
                force = Vector2D.mul(Evade(targetAgent1), weightEvade);

                if (!AccumulateForce(steeringForce, force))
                {
                    return steeringForce;
                }
            }


            //these next three can be combined for flocking behavior (wander is
            //also a good behavior to add into this mix)       
            {
                if (On(behavior_type.separation))
                {
                    force = Vector2D.mul(Separation(vehicle.world.Agents()), weightSeparation);

                    if (!AccumulateForce(steeringForce, force))
                    {
                        return steeringForce;
                    }
                }

                if (On(behavior_type.allignment))
                {
                    force = Vector2D.mul(Alignment(vehicle.world.Agents()), weightAlignment);

                    if (!AccumulateForce(steeringForce, force))
                    {
                        return steeringForce;
                    }
                }

                if (On(behavior_type.cohesion))
                {
                    force = Vector2D.mul(Cohesion(vehicle.world.Agents()), weightCohesion);

                    if (!AccumulateForce(steeringForce, force))
                    {
                        return steeringForce;
                    }
                }
            }
           

            if (On(behavior_type.wander))
            {
                force = Vector2D.mul(Wander(), weightWander);

                if (!AccumulateForce(steeringForce, force))
                {
                    return steeringForce;
                }
            }

            return steeringForce;
        }

        public void FleeOn()
        {
            flags |= (int)behavior_type.flee;
        }

        public void SeekOn()
        {
            flags |= (int)behavior_type.seek;
        }

        public void ArriveOn()
        {
            flags |= (int)behavior_type.arrive;
        }

        public void WanderOn()
        {
            flags |= (int)behavior_type.wander;
        }

        public void PursuitOn(Vehicle v)
        {
            flags |= (int)behavior_type.pursuit;
            targetAgent1 = v;
        }

        public void EvadeOn(Vehicle v)
        {
            flags |= (int)behavior_type.evade;
            targetAgent1 = v;
        }

        public void CohesionOn()
        {
            flags |= (int)behavior_type.cohesion;
        }

        public void SeparationOn()
        {
            flags |= (int)behavior_type.separation;
        }

        public void AlignmentOn()
        {
            flags |= (int)behavior_type.allignment;
        }

        public void ObstacleAvoidanceOn()
        {
            flags |= (int)behavior_type.obstacle_avoidance;
        }

        public void WallAvoidanceOn()
        {
            flags |= (int)behavior_type.wall_avoidance;
        }

        public void FollowPathOn()
        {
            flags |= (int)behavior_type.follow_path;
        }

        public void InterposeOn(Vehicle v1, Vehicle v2)
        {
            flags |= (int)behavior_type.interpose;
            targetAgent1 = v1;
            targetAgent2 = v2;
        }

        public void HideOn(Vehicle v)
        {
            flags |= (int)behavior_type.hide;
            targetAgent1 = v;
        }

        public void OffsetPursuitOn(Vehicle v1, Vector2D offset)
        {
            flags |= (int)behavior_type.offset_pursuit;
            this.offset = offset;
            targetAgent1 = v1;
        }

        public void FlockingOn()
        {
            CohesionOn();
            AlignmentOn();
            SeparationOn();
            WanderOn();
        }

        public void FleeOff()
        {
            if (On(behavior_type.flee))
            {
                flags ^= (int)behavior_type.flee;
            }
        }

        public void SeekOff()
        {
            if (On(behavior_type.seek))
            {
                flags ^= (int)behavior_type.seek;
            }
        }

        public void ArriveOff()
        {
            if (On(behavior_type.arrive))
            {
                flags ^= (int)behavior_type.arrive;
            }
        }

        public void WanderOff()
        {
            if (On(behavior_type.wander))
            {
                flags ^= (int)behavior_type.wander;
            }
        }

        public void PursuitOff()
        {
            if (On(behavior_type.pursuit))
            {
                flags ^= (int)behavior_type.pursuit;
            }
        }

        public void EvadeOff()
        {
            if (On(behavior_type.evade))
            {
                flags ^= (int)behavior_type.evade;
            }
        }

        public void CohesionOff()
        {
            if (On(behavior_type.cohesion))
            {
                flags ^= (int)behavior_type.cohesion;
            }
        }

        public void SeparationOff()
        {
            if (On(behavior_type.separation))
            {
                flags ^= (int)behavior_type.separation;
            }
        }

        public void AlignmentOff()
        {
            if (On(behavior_type.allignment))
            {
                flags ^= (int)behavior_type.allignment;
            }
        }

        public void ObstacleAvoidanceOff()
        {
            if (On(behavior_type.obstacle_avoidance))
            {
                flags ^= (int)behavior_type.obstacle_avoidance;
            }
        }

        public void WallAvoidanceOff()
        {
            if (On(behavior_type.wall_avoidance))
            {
                flags ^= (int)behavior_type.wall_avoidance;
            }
        }

        public void FollowPathOff()
        {
            if (On(behavior_type.follow_path))
            {
                flags ^= (int)behavior_type.follow_path;
            }
        }

        public void InterposeOff()
        {
            if (On(behavior_type.interpose))
            {
                flags ^= (int)behavior_type.interpose;
            }
        }

        public void HideOff()
        {
            if (On(behavior_type.hide))
            {
                flags ^= (int)behavior_type.hide;
            }
        }

        public void OffsetPursuitOff()
        {
            if (On(behavior_type.offset_pursuit))
            {
                flags ^= (int)behavior_type.offset_pursuit;
            }
        }

        public void FlockingOff()
        {
            CohesionOff();
            AlignmentOff();
            SeparationOff();
            WanderOff();
        }

        public bool isFleeOn()
        {
            return On(behavior_type.flee);
        }

        public bool isSeekOn()
        {
            return On(behavior_type.seek);
        }

        public bool isArriveOn()
        {
            return On(behavior_type.arrive);
        }

        public bool isWanderOn()
        {
            return On(behavior_type.wander);
        }

        public bool isPursuitOn()
        {
            return On(behavior_type.pursuit);
        }

        public bool isEvadeOn()
        {
            return On(behavior_type.evade);
        }

        public bool isCohesionOn()
        {
            return On(behavior_type.cohesion);
        }

        public bool isSeparationOn()
        {
            return On(behavior_type.separation);
        }

        public bool isAlignmentOn()
        {
            return On(behavior_type.allignment);
        }

        public bool isObstacleAvoidanceOn()
        {
            return On(behavior_type.obstacle_avoidance);
        }

        public bool isWallAvoidanceOn()
        {
            return On(behavior_type.wall_avoidance);
        }

        public bool isFollowPathOn()
        {
            return On(behavior_type.follow_path);
        }

        public bool isInterposeOn()
        {
            return On(behavior_type.interpose);
        }

        public bool isHideOn()
        {
            return On(behavior_type.hide);
        }

        public bool isOffsetPursuitOn()
        {
            return On(behavior_type.offset_pursuit);
        }

        private void CreateFeelers()
        {
            //feelers.Clear();
            ////feeler pointing straight in front
            //feelers.Add(Vector2D.add(vehicle.pos, Vector2D.mul(wallDetectionFeelerLength, vehicle.heading)));

            ////feeler to left
            //Vector2D temp = new Vector2D(vehicle.heading);
            //Transformation.Vec2DRotateAroundOrigin(temp, Utils.HalfPi * 3.5f);
            //feelers.Add(Vector2D.add(vehicle.pos, Vector2D.mul(wallDetectionFeelerLength / 2.0f, temp)));

            ////feeler to right
            //temp = new Vector2D(vehicle.heading);
            //Transformation.Vec2DRotateAroundOrigin(temp, Utils.HalfPi * 0.5f);
            //feelers.Add(Vector2D.add(vehicle.pos, Vector2D.mul(wallDetectionFeelerLength / 2.0f, temp)));
        }

        /////////////////////////////////////////////////////////////////////////////// START OF BEHAVIORS
        /**
         * Given a target, this behavior returns a steering force which will
         *  direct the agent towards the target
         */
        private Vector2D Seek(Vector2D TargetPos)
        {
            Vector2D DesiredVelocity = Vector2D.mul(Vector2D.Vec2DNormalize(Vector2D.sub(TargetPos, vehicle.pos)),
                    vehicle.maxSpeed);

            return Vector2D.sub(DesiredVelocity, vehicle.velocity);
        }


        private Vector2D Flee(Vector2D TargetPos)
        {
            //only flee if the target is within 'panic distance'. Work in distance
            //squared space.
            /* const double PanicDistanceSq = 100.0f * 100.0;
                   if (Vec2DDistanceSq(m_pVehicle.Pos(), target) > PanicDistanceSq)
                   {
                   return new Vector2D(0,0);
                   }
                    */

            Vector2D DesiredVelocity = Vector2D.mul(Vector2D.Vec2DNormalize(Vector2D.sub(vehicle.pos, TargetPos)),
                    vehicle.maxSpeed);

            return Vector2D.sub(DesiredVelocity, vehicle.velocity);
        }

        /**
  * This behavior is similar to seek but it attempts to arrive at the
  *  target with a zero velocity
  */
        private Vector2D Arrive(Vector2D TargetPos, Deceleration deceleration)
        {
            Vector2D ToTarget = Vector2D.sub(TargetPos, vehicle.pos);

            //calculate the distance to the target
            double dist = ToTarget.Length();

            if (dist > 0)
            {
                //because Deceleration is enumerated as an int, this value is required
                //to provide fine tweaking of the deceleration..
                double DecelerationTweaker = 0.3;

                //calculate the speed required to reach the target given the desired
                //deceleration
                double speed = dist / ((double)deceleration * DecelerationTweaker);

                //make sure the velocity does not exceed the max
                speed = System.Math.Min(speed, vehicle.maxSpeed);

                //from here proceed just like Seek except we don't need to normalize 
                //the ToTarget vector because we have already gone to the trouble
                //of calculating its length: dist. 
                Vector2D DesiredVelocity = Vector2D.mul(ToTarget, speed / dist);

                return Vector2D.sub(DesiredVelocity, vehicle.velocity);
            }

            return new Vector2D(0, 0);
        }

        /**
         *  similar to pursuit except the agent Flees from the estimated future
         *  position of the pursuer
         */
        private Vector2D Evade(Vehicle pursuer)
        {
            // Not necessary to include the check for facing direction this time

            Vector2D ToPursuer = Vector2D.sub(pursuer.pos, vehicle.pos);

            //uncomment the following two lines to have Evade only consider pursuers 
            //within a 'threat range'
            double ThreatRange = 100.0;
            if (ToPursuer.LengthSq() > ThreatRange * ThreatRange)
            {
                return new Vector2D();
            }

            //the lookahead time is propotional to the distance between the pursuer
            //and the pursuer; and is inversely proportional to the sum of the
            //agents' velocities
            double LookAheadTime = ToPursuer.Length()
                    / (vehicle.maxSpeed + pursuer.Speed());

            //now flee away from predicted future position of the pursuer
            return Flee(Vector2D.add(pursuer.pos, Vector2D.mul(pursuer.velocity, LookAheadTime)));
        }

        /**
         * This behavior makes the agent wander about randomly
         */
        private Vector2D Wander()
        {
            //this behavior is dependent on the update rate, so this line must
            //be included when using time independent framerate.
            double JitterThisTimeSlice = wanderJitter * vehicle.getTimeElapsed();

            //first, add a small random vector to the target's position
            wanderTarget.add(new Vector2D(Utils.RandomClamped() * JitterThisTimeSlice,
                    Utils.RandomClamped() * JitterThisTimeSlice));

            //reproject this new vector back on to a unit circle
            wanderTarget.Normalize();

            //increase the length of the vector to the same as the radius
            //of the wander circle
            wanderTarget.mul(wanderRadius);

            //move the target into a position WanderDist in front of the agent
            Vector2D target = Vector2D.add(wanderTarget, new Vector2D(wanderDistance, 0));

            //project the target into world space
            Vector2D Target = Transformation.PointToWorldSpace(target,
                    vehicle.heading,
                    vehicle.side,
                    vehicle.pos);

            //and steer towards it
            return Vector2D.sub(Target, vehicle.pos);
        }


        /**
         * this calculates a force repelling from the other neighbors
         */
        Vector2D Separation(List<Vehicle> neighbors)
        {
            Vector2D SteeringForce = new Vector2D();

            for (int a = 0; a < neighbors.Count; ++a)
            {
                //make sure this agent isn't included in the calculations and that
                //the agent being examined is close enough. ***also make sure it doesn't
                //include the evade target ***
                if ((neighbors[a] != vehicle) && neighbors[a].IsTagged()
                        && (neighbors[a] != targetAgent1))
                {
                    Vector2D ToAgent = Vector2D.sub(vehicle.pos, neighbors[a].pos);

                    //scale the force inversely proportional to the agents distance  
                    //from its neighbor.
                    SteeringForce.add(Vector2D.div(Vector2D.Vec2DNormalize(ToAgent), ToAgent.Length()));
                }
            }

            return SteeringForce;
        }

        /**
         * returns a force that attempts to align this agents heading with that
         * of its neighbors
         */
        private Vector2D Alignment(List<Vehicle> neighbors)
        {
            //used to record the average heading of the neighbors
            Vector2D AverageHeading = new Vector2D();

            //used to count the number of vehicles in the neighborhood
            int NeighborCount = 0;

            //iterate through all the tagged vehicles and sum their heading vectors  
            for (int a = 0; a < neighbors.Count; ++a)
            {
                //make sure *this* agent isn't included in the calculations and that
                //the agent being examined  is close enough ***also make sure it doesn't
                //include any evade target ***
                if ((neighbors[a] != vehicle) && neighbors[a].IsTagged()
                        && (neighbors[a] != targetAgent1))
                {
                    AverageHeading.add(neighbors[a].heading);

                    ++NeighborCount;
                }
            }

            //if the neighborhood contained one or more vehicles, average their
            //heading vectors.
            if (NeighborCount > 0)
            {
                AverageHeading.div((double)NeighborCount);
                AverageHeading.sub(vehicle.heading);
            }

            return AverageHeading;
        }

        /**
         * returns a steering force that attempts to move the agent towards the
         * center of mass of the agents in its immediate area
         */
        private Vector2D Cohesion(List<Vehicle> neighbors)
        {
            //first find the center of mass of all the agents
            Vector2D CenterOfMass = new Vector2D(), SteeringForce = new Vector2D();

            int NeighborCount = 0;

            //iterate through the neighbors and sum up all the position vectors
            for (int a = 0; a < neighbors.Count; ++a)
            {
                //make sure *this* agent isn't included in the calculations and that
                //the agent being examined is close enough ***also make sure it doesn't
                //include the evade target ***
                if ((neighbors[a] != vehicle) && neighbors[a].IsTagged()
                        && (neighbors[a] != targetAgent1))
                {
                    CenterOfMass.add(neighbors[a].pos);

                    ++NeighborCount;
                }
            }

            if (NeighborCount > 0)
            {
                //the center of mass is the average of the sum of positions
                CenterOfMass.div((double)NeighborCount);

                //now seek towards that position
                SteeringForce = Seek(CenterOfMass);
            }

            //the magnitude of cohesion is usually much larger than separation or
            //allignment so it usually helps to normalize it.
            return Vector2D.Vec2DNormalize(SteeringForce);
        }


        /**
         * Produces a steering force that keeps a vehicle at a specified offset
         * from a leader vehicle
         */
        private Vector2D OffsetPursuit(Vehicle leader,
                Vector2D offset)
        {
            //calculate the offset's position in world space
            Vector2D WorldOffsetPos = Transformation.PointToWorldSpace(offset,
                    leader.heading,
                    leader.side,
                    leader.pos);

            Vector2D ToOffset = Vector2D.sub(WorldOffsetPos, vehicle.pos);

            //the lookahead time is propotional to the distance between the leader
            //and the pursuer; and is inversely proportional to the sum of both
            //agent's velocities
            double LookAheadTime = ToOffset.Length()
                    / (vehicle.maxSpeed + leader.Speed());

            //now Arrive at the predicted future position of the offset
            return Arrive(Vector2D.add(WorldOffsetPos, Vector2D.mul(leader.velocity, LookAheadTime)), Deceleration.fast);
        }
    }
}