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
        rotated_unit_vector.x=provider->data.x;
        rotated_unit_vector.y=provider->data.y;
        rotated_unit_vector.z=provider->data.z;
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
    Vector3D<double> p1 , p2 , p_d_c;
    Vector3D<double> p_d_c_rotated=rotate_offset();

    

    float depth_adjusted_for_camera_offset=depth+p_d_c_rotated.y;

    projection_plane.p0.y=inertial_plane_offset-depth_adjusted_for_camera_offset;
    projection_plane.p1.y=inertial_plane_offset-depth_adjusted_for_camera_offset;
    projection_plane.p2.y=inertial_plane_offset-depth_adjusted_for_camera_offset;

    p_i_d.x=0;
    p_i_d.y= -(inertial_plane_offset-depth);
    p_i_d.z=0;

      
    p1=p_i_d + p_d_c_rotated;

    p2=rotated_unit_vector + p1;


    Vector3D<double> intersection_pt= projection_plane.getIntersectingLine(p1,p2);

    //Vector3D<double> data_transmitted;


    // data_transmitted.x=(intersection_pt.x) * -1;
    // data_transmitted.y=intersection_pt.y;
    // data_transmitted.z=(intersection_pt.z) * -1;

    Vector3DMsg p_i_d_msg;
    p_i_d_msg.data = p_i_d;
    this->_output_port_0->receiveMsgData(&p_i_d_msg);

    
    // Vector3D<double> data_transmitted_with_offset;


    // data_transmitted_with_offset.x=offset.x;
    // data_transmitted_with_offset.y=offset.y;
    // data_transmitted_with_offset.z=offset.z;

    Vector3DMsg p1_msg;
    p1_msg.data = p1;
    this->_output_port_1->receiveMsgData(&p1_msg);
  
    Vector3DMsg p2_msg;
    p2_msg.data = p2;
    this->_output_port_2->receiveMsgData(&p2_msg);

    Vector3DMsg intersection_msg;
    intersection_msg.data = intersection_pt;
    this->_output_port_3->receiveMsgData(&intersection_msg);



}