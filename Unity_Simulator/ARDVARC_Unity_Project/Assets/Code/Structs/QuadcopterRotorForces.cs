namespace ARDVARC_Unity_Project.Structs
{
    public struct QuadcopterRotorForces
    {
        public float F1, F2, F3, F4;

        public QuadcopterRotorForces(float f1, float f2, float f3, float f4)
        {
            F1 = f1;
            F2 = f2;
            F3 = f3;
            F4 = f4;
        }
    }
}
