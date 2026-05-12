#include <cmath>
#include "geometry_msgs/msg/vector3.hpp"


namespace mrg_modimoop_localization
{
    /*struct Vector3
    {
        double x{0.0};
        double y{0.0};
        double z{0.0};

        Vector3() = default;

        Vector3(double x_in, double y_in, double z_in = 0.0) : x(x_in), y(y_in), z(z_in){}
        
        geometry_msgs::msg::Vector3 toMsg() const
        {
            geometry_msgs::msg::Vector3 msg;
            msg.x = x;
            msg.y = y;
            msg.z = z;
            return msg;
        }

    } */

    inline double wrapTo2Pi(double angle)
    {
        angle = std::fmod(angle, 2.0 * M_PI);
        if (angle < 0.0) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

} // mrg_modimoop_localization