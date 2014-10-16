#include "pid.h"

int main(int argc, char **argv)
{
    //ros::init(argc, argv, "testnode");

    //static 1d examples
    double u;
    u = pd::P_control(1,0,1);
    u = pd::PD_control(0.1,0.1,0.2,1,0.1,1, 0.1);

    //--------------------------------------------------------------------------
    // 1d example
    pid_1d controller_1d = pid_1d(0.3,0.1,0.01);
    double x = 0;
    double t = 1;

    for(int i = 0; i < 100; ++i) {
        x += controller_1d.control_1d(x,t,0.1);
    }


    //--------------------------------------------------------------------------
    // 2d example
    // Note: pid<n>::Params is only a typedef of the Eigen matrix class.
    //       One can always use Vector2d or Matrix<double, 2, 1> or whatever.

    pid<2> controller_2d(pid<2>::Params(0.3,0.3),
                         pid<2>::Params(0.1,0.1),
                         pid<2>::Params(0.01,0.01) );

    Vector2d target(1,1); //also possible to use: pid<2>::Params(1,1).
    Vector2d state(0,0);  //same here

    for(int i = 0; i < 100; ++i) {
        state += controller_2d.control(state, target, 0.1);

        std::cout << state << std::endl;
    }

    return 0;
}
