#include "pixeltometer.hpp"
using namespace std;

//Current camera orientation with respect to the drone frame
//               
//                      z-axis                   
//                    ^ 
//                    |                     
//                    |       
//    (y inwards)     +----> x-axis 



pixeltometer::pixeltometer()
{
    depth=2.93;
    f_x=616.437;
    f_y=616.65;

    this->_input_port_0 = new InputPort(ports_id::IP_0_CAMERA, this);
    this->_input_port_1 = new InputPort(ports_id::IP_1_ROLL, this);
    this->_input_port_2 = new InputPort(ports_id::IP_2_PITCH, this);
    this->_input_port_3 = new InputPort(ports_id::IP_3_YAW, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port_0, _input_port_1,_input_port_2 ,_input_port_3,_output_port};
}

pixeltometer::~pixeltometer()
{
}

void pixeltometer::process(DataMsg* t_msg, Port* t_port) {
    Vector3DMsg *provider = (Vector3DMsg *)t_msg;

    if(t_port->getID() == ports_id::IP_0_CAMERA)
    {
        Vector2DMsg* pixel_location = (Vector2DMsg*) t_msg;
        ball_location.x=pixel_location->data.x;
        ball_location.y=pixel_location->data.y;
        convert_pixel_to_meters();
    }
    else if(t_port->getID() == ports_id::IP_1_ROLL)
    { 
        drone_orientation.x =provider->data.x;
        update_rotation_matrices();
    }
    else if(t_port->getID() == ports_id::IP_2_PITCH)
    { 
        drone_orientation.y =provider->data.x;
    }
    else if(t_port->getID() == ports_id::IP_3_YAW)
    { 
        drone_orientation.z =provider->data.x;
    }
}

Vector3D<float> pixeltometer::update_location(Vector3D<float> object_location)
{
    Vector3D<float> t_results;
    t_results.x = object_location.x * R_inverse(0, 0) + object_location.y * R_inverse(0, 1) + object_location.z * R_inverse(0, 2);
    t_results.y = object_location.x * R_inverse(1, 0) + object_location.y * R_inverse(1, 1) + object_location.z * R_inverse(1, 2);
    t_results.z = object_location.x * R_inverse(2, 0) + object_location.y * R_inverse(2, 1) + object_location.z * R_inverse(2, 2);

    
    
    Vector3DMsg point_msg;
    point_msg.data = t_results;
    this->_output_port->receiveMsgData(&point_msg);

    return t_results;
}


void pixeltometer::convert_pixel_to_meters()
{
    object_location.x=(ball_location.x*depth)/f_x;
    object_location.y=depth;
    object_location.z=(ball_location.y*depth)/f_y;
    

    this->update_location(object_location);
}

void pixeltometer::update_rotation_matrices()    
{
    RotationMatrix3by3 R_d_c;
    Eigen::Matrix<double,3,3> R_drone;

    R_drone= R_d_c.Update(drone_orientation);
    R_inverse=R_drone.inverse();

}