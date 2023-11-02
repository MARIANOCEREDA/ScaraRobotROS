
#define PI 3.1415912

namespace Robot
{
    struct robot_dimensions
    {
        float a0 = 0.2;
        float a1 = 1;
        float a2 = 1;
        float limits_j0[2] = {0.0, 0.4};
        float limits_j1[2] = {-PI/2, PI/2};
        float limits_j2[2] = {-PI/2, PI/2};
    };
}



