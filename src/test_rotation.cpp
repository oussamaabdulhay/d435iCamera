#include "test_rotation.hpp"
using namespace std;

test_rotation::test_rotation()
{

    f_c=616.5;
    
    this->_input_port_0 = new InputPort(ports_id::IP_0_CAMERA, this);
    this->_input_port_1 = new InputPort(ports_id::IP_1_ROLL, this);
    this->_input_port_2 = new InputPort(ports_id::IP_2_PITCH, this);
    this->_input_port_3 = new InputPort(ports_id::IP_3_YAW, this);
    this->_output_port_0 = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port_0, _input_port_1,_input_port_2 ,_input_port_3,_output_port_0};
}

test_rotation::~test_rotation()
{
}

void test_rotation::process(DataMsg* t_msg, Port* t_port) {
    Vector3DMsg *provider = (Vector3DMsg *)t_msg;

    if(t_port->getID() == ports_id::IP_0_CAMERA)
    {
        Vector3DMsg* pixel_location = (Vector3DMsg*) t_msg;
        ball_location.x= pixel_location->data.x;
        ball_location.y= pixel_location->data.y;

        camera_vector.x = -1 * f_c;
        camera_vector.y = ball_location.x;
        camera_vector.z = ball_location.y;

        this->update_rotation_matrices(drone_orientation);
    }
    else if(t_port->getID() == ports_id::IP_1_ROLL)
    { 
        drone_orientation.x =provider->data.x;
        //drone_orientation.x =0;
    }
    else if(t_port->getID() == ports_id::IP_2_PITCH)
    { 
        drone_orientation.y =provider->data.x;
        //std::cout<<"drone_orientation.y"<<drone_orientation.y<<std::endl;        
    }
    else if(t_port->getID() == ports_id::IP_3_YAW)
    { 
        drone_orientation.z =provider->data.x;        
    }
}


void test_rotation::rotate_camera_vector()
{
    Vector3D<float> t_results;
    t_results.x = camera_vector.x * R_d_to_i_temp(0, 0) + camera_vector.y * R_d_to_i_temp(0, 1) + camera_vector.z * R_d_to_i_temp(0, 2);
    t_results.y = camera_vector.x * R_d_to_i_temp(1, 0) + camera_vector.y * R_d_to_i_temp(1, 1) + camera_vector.z * R_d_to_i_temp(1, 2);
    t_results.z = camera_vector.x * R_d_to_i_temp(2, 0) + camera_vector.y * R_d_to_i_temp(2, 1) + camera_vector.z * R_d_to_i_temp(2, 2);

    Vector3DMsg point_msg;
    point_msg.data = t_results;
    this->_output_port_0->receiveMsgData(&point_msg);
}


void test_rotation::update_rotation_matrices(Vector3D<float> drone_orientation)    
{
    RotationMatrix3by3 R_d_i;
    R_d_to_i_temp = R_d_i.Update(drone_orientation); //Create the rotation matrices
    R_d_to_i_temp.transposeInPlace();

    this->rotate_camera_vector();
}