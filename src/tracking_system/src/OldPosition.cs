using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WeedKiller
{
    public class Position
    {
        static public double SPRAY_DELAY_TIME = 0.26; // 260ms (sprayer spec says 1/4 second turn on time)
        private DateTime time;
        private double changeInTime;
        private double xPosition;
        private double yPosition;
        private double yaw;
        private double velocity;
        const int NUMBER_OF_SPRAYERS = 8;

        private static readonly double[,] CAMERA_LOCATIONS = new double[8, 2] { { -0.875, 0.765 }, { -0.625, 0.765 }, { -0.375, 0.765 }, { -0.125, 0.765 }, //Sprayer Relay 1 (Sprayers 1-4)
                                                              { 0.125, 0.765 }, { 0.375, 0.765 }, { 0.625, 0.765 }, { 0.875, 0.765 } }; // Sprayer Relay 2 (Sprayers 5-8)


        private static readonly double[,] SPRAYER_LOCATIONS = new double[8, 2] { { -0.875, -1.05 }, { -0.625, -1.05 }, { -0.375, -1.05 }, { -0.125, -1.05 }, //Sprayer Relay 1 (Sprayers 1-4)
                                                              { 0.125, -1.05 }, { 0.375, -1.05 }, { 0.625, -1.05 }, { 0.875, -1.05 } }; // Sprayer Relay 2 (Sprayers 5-8)


        public Position(DateTime time, double changeInTime, double xPosition, double yPosition, double yaw, double velocity = 0)
        {
            this.time = time;
            this.changeInTime = changeInTime;
            this.xPosition = xPosition;
            this.yPosition = yPosition;
            this.yaw = yaw;
            this.velocity = velocity;
        }

        public Position(double xPosition, double yPosition)
        {
            time = DateTime.Now;
            changeInTime = 0;
            this.xPosition = xPosition;
            this.yPosition = yPosition;
            yaw = 0;
            velocity = 0;
        }

        public DateTime getTime()
        {
            return time;
        }

        public double getChangeInTime()
        {
            return changeInTime;
        }

        public double getXPosition()
        {
            return xPosition;
        }

        public double getYPosition()
        {
            return yPosition;
        }

        public double getYaw()
        {
            return yaw;
        }

        public double getVelocity()
        {
            return velocity;
        }

        public Position clone()
        {
            return new Position(this.time, this.changeInTime, this.xPosition, this.yPosition, this.yaw, this.velocity);
        }

        static public Boolean isPositionWithinLimits(double xMin, double xMax, double yMin, double yMax, Position position)
        {
            if (((position.getXPosition() < xMax) & (position.getXPosition() > xMin))
                & ((position.getYPosition() < yMax) & (position.getYPosition() > yMin)))
            {
                return true;
            }
            return false;
        }

        static public Boolean isPositionWithinLimits(Position targetCentrePosition, Position sprayerCentrePosition, Double radius, double velocity)
        {
            double delayDistance = velocity * SPRAY_DELAY_TIME;
            //double delayDistance = 0.1 * velocity;
            double diffX = sprayerCentrePosition.getXPosition() - targetCentrePosition.getXPosition();
            double diffY = sprayerCentrePosition.getYPosition() - (targetCentrePosition.getYPosition() - delayDistance); // - delayDistance;
            //double delayDistance = SPRAY_DELAY_TIME * velocity; // 0.6;// The distance the sprayer will move from the time it is "turned on" to the time it actually activates.
            // Returns true if the sprayer is on or within the delay distance of the target
            // Ideally this will hit the target dead on the centre point
            return (diffX * diffX + diffY * diffY) <= radius * radius;

            //if ((diffX*diffX + diffY*diffY) <= (radius + delayDistance) * (radius + delayDistance))
            //{
            //    return true;
            //}
            //return false;
        }

        static public Position CalculateGlobalCameraPosition(uint cameraSerial, Position currentPosition)
        {
            int cameraNumber = Array.IndexOf(View.SERIAL_NUMBERS, cameraSerial) + 4;

            double rotatedCameraXPosition = (CAMERA_LOCATIONS[cameraNumber, 0] * Math.Cos(-currentPosition.getYaw())) - (CAMERA_LOCATIONS[cameraNumber, 1] * Math.Sin(-currentPosition.getYaw()));
            double rotatedCameraYPosition = (CAMERA_LOCATIONS[cameraNumber, 0] * Math.Sin(-currentPosition.getYaw())) + (CAMERA_LOCATIONS[cameraNumber, 1] * Math.Cos(-currentPosition.getYaw()));

            double globalCameraXPosition = currentPosition.getXPosition() + rotatedCameraXPosition;
            double globalCameraYPosition = currentPosition.getYPosition() + rotatedCameraYPosition;

            return new Position(currentPosition.getTime(), currentPosition.getChangeInTime(), globalCameraXPosition, globalCameraYPosition, currentPosition.getYaw());
        }

        static public Position[] CalculateGlobalSprayerPositions(Position currentPosition)
        {
            Position[] sprayerGlobalPositions = new Position[8];

            for (int i = 0; i < NUMBER_OF_SPRAYERS; i++)
            {
                double rotatedXSprayerPosition = (SPRAYER_LOCATIONS[i, 0] * Math.Cos(-currentPosition.getYaw())) - (SPRAYER_LOCATIONS[i, 1] * Math.Sin(-currentPosition.getYaw()));
                double rotatedYSprayerPosition = (SPRAYER_LOCATIONS[i, 0] * Math.Sin(-currentPosition.getYaw())) + (SPRAYER_LOCATIONS[i, 1] * Math.Cos(-currentPosition.getYaw()));

                double globalXSprayerPosition = currentPosition.getXPosition() + rotatedXSprayerPosition;
                double globalYSprayerPosition = currentPosition.getYPosition() + rotatedYSprayerPosition;

                sprayerGlobalPositions[i] = new Position(currentPosition.getTime(), currentPosition.getChangeInTime(), globalXSprayerPosition, globalYSprayerPosition, currentPosition.getYaw());
            }

            return sprayerGlobalPositions;
        }
    }
}
