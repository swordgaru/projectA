using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ting.AI
{
    public class AIConfig
    {
        public const int NumAgents = 100;

        public const double SeparationWeight = 1.0;
        public const double CohesionWeight = 2.0;
        public const double AlignmentWeight = 1.0;
        public const double ObstacleAvoidanceWeight = 10.0;

        public const double WanderWeight = 1.0;
        public const double SeekWeight = 1.0;
        public const double FleeWeight = 1.0;
        public const double ArriveWeight = 1.0;
        public const double PursuitWeight = 1.0;
        public const double OffsetPursuitWeight = 1.0;
        public const double InterposeWeight = 1.0;
        public const double HideWeight = 1.0;
        public const double EvadeWeight = 0.01;

        public const double ViewDistance = 10.0; //50.0;
        //public const double MinDetectionBoxLength = 40;
        //public const double WallDetectionFeelerLength = 40;

        public const double SteeringForceTweaker = 200.0;
        public const double SteeringForce = 2.0;
        public const double MaxSpeed = 150.0;
        public const double VehicleMass = 1.0;
        public const double VehicleScale = 6.0;

        public const double MaxTurnRatePerSecond = Utils.Pi;
    }
}