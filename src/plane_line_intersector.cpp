#include "plane_line_intersector.hpp"
using namespace std;

plane_line_intersector::plane_line_intersector()
{
    p_d_c.x = 0.1308;
    p_d_c.y = 0.038;
    p_d_c.z = -0.1137;

    plane_point1.x=1.021;
    plane_point1.y=3.127;
    plane_point1.z=1.629;

    plane_point2.x=0.244;
    plane_point2.y=3.1517;
    plane_point2.z=1.9122;

    plane_point3.x=-0.535;
    plane_point3.y=3.188;
    plane_point3.z=1.495;
    
    projection_plane.p0=plane_point1;
    projection_plane.p1=plane_point2;
    projection_plane.p2=plane_point3;

    inertial_plane_offset=(plane_point1.y+plane_point2.y+plane_point3.y)/3.0;

    this->_input_port_0 = new InputPort(ports_id::IP_0_UNIT_VEC, this);
    this->_input_port_1 = new InputPort(ports_id::IP_1_DEPTH_DATA, this);
    this->_input_port_2 = new InputPort(ports_id::IP_2_ROLL, this);
    this->_input_port_3 = new InputPort(ports_id::IP_3_PITCH, this);
    this->_input_port_4 = new InputPort(ports_id::IP_4_YAW, this);
    this->_output_port_0 = new OutputPort(ports_id::OP_0_DATA, this);
    this->_output_port_1 = new OutputPort(ports_id::OP_1_DATA, this);
    this->_output_port_2 = new OutputPort(ports_id::OP_2_DATA, this);
    this->_output_port_3 = new OutputPort(ports_id::OP_3_DATA, this);

    _ports = {_input_port_0, _input_port_1, _input_port_2, _input_port_3, _input_port_4, _output_port_0, _output_port_1, _output_port_2, _output_port_3};
}

plane_line_intersector::~plane_line_intersector()
{
}

void plane_line_intersector::process(DataMsg* t_msg, Port* t_port) {
    Vector3DMsg *provider = (Vector3DMsg *)t_msg;

    if(t_port->getID() == ports_id::IP_0_UNIT_VEC)
    {
        rotated_pixel_vector.x=provider->data.x;
        rotated_pixel_vector.y=provider->data.y;
        rotated_pixel_vector.z=provider->data.z;
        this->get_object_location();
    }
    
    else if(t_port->getID() == ports_id::IP_1_DEPTH_DATA) //TODO: Caution about update rate
    { 
        depth=provider->data.x;
    }
       
    else if(t_port->getID() == ports_id::IP_2_ROLL)
    { 
        drone_orientation.x =provider->data.x;
    }
    else if(t_port->getID() == ports_id::IP_3_PITCH)
    { 
        drone_orientation.y =provider->data.x;
    }
    else if(t_port->getID() == ports_id::IP_4_YAW)
    { 
        drone_orientation.z =provider->data.x;
    }
}


Vector3D<float> plane_line_intersector::rotate_offset()    
{
    Eigen::Matrix<float, 3, 3> R_body_to_inertial_temp(3, 3);
    RotationMatrix3by3 R_b_i;

    R_body_to_inertial_temp = R_b_i.Update(drone_orientation); //Create the rotation matrices
    R_body_to_inertial_temp.transposeInPlace(); //drone to inertial

    Vector3D<float> t_results;
    t_results.x = p_d_c.x * R_body_to_inertial_temp(0, 0) + p_d_c.y * R_body_to_inertial_temp(0, 1) + p_d_c.z * R_body_to_inertial_temp(0, 2);
    t_results.y = p_d_c.x * R_body_to_inertial_temp(1, 0) + p_d_c.y * R_body_to_inertial_temp(1, 1) + p_d_c.z * R_body_to_inertial_temp(1, 2);
    t_results.z = p_d_c.x * R_body_to_inertial_temp(2, 0) + p_d_c.y * R_body_to_inertial_temp(2, 1) + p_d_c.z * R_body_to_inertial_temp(2, 2);

    return t_results;
}

Vector3D<float> plane_line_intersector::get_object_location()    
{

    Vector3D<float> object_location,data_transmitted_bo, data_transmitted_ao;

    object_location.x= (rotated_pixel_vector.x * 4.0) / rotated_pixel_vector.y;
    object_location.y= 3.69;
    object_location.z=(rotated_pixel_vector.z * 4.0) / rotated_pixel_vector.y;



    data_transmitted_bo.x = object_location.x; 
    data_transmitted_bo.y = object_location.y;
    data_transmitted_bo.z = object_location.z;

    Vector3DMsg data_before_offset;
    data_before_offset.data = data_transmitted_bo;
    this->_output_port_0->receiveMsgData(&data_before_offset);

    Vector3D<float> rotated_offset = rotate_offset();

    data_transmitted_ao.x = object_location.x + rotated_offset.x; 
    data_transmitted_ao.y = object_location.y + rotated_offset.y;
    data_transmitted_ao.z = object_location.z + rotated_offset.x;

    Vector3DMsg data_after_offset;
    data_after_offset.data = data_transmitted_ao;
    this->_output_port_1->receiveMsgData(&data_after_offset);



}