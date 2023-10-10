using UnityEngine;

namespace AALG3.Structs
{
    public struct LMNZ
    {
        const float L_MAX = 300, M_MAX = 10, N_MAX = 300, Z_MAX = 1000;
        public float L, M, N, Z;

        public LMNZ(float l, float m, float n, float z)
        {
            L = Mathf.Clamp(l, -L_MAX, L_MAX);
            M = Mathf.Clamp(m, -M_MAX, M_MAX);
            N = Mathf.Clamp(n, -N_MAX, N_MAX);
            Z = Mathf.Clamp(z, -Z_MAX, Z_MAX);
        }
    }
}