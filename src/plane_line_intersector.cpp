#include "plane_line_intersector.hpp"
using namespace std;

plane_line_intersector::plane_line_intersector()
{
    drone_camera_offset.x = 0.15;
    drone_camera_offset.y = 0;
    drone_camera_offset.z = -0.05;

    plane_point1.x=1.31;
    plane_point1.y=3.18;
    plane_point1.z=1.79;

    plane_point2.x=-1.205;
    plane_point2.y=3.25;
    plane_point2.z=0.940;

    plane_point3.x=-0.065;
    plane_point3.y=3.231;
    plane_point3.z=1.387;
    
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

    _ports = {_input_port_0, _input_port_1, _input_port_2, _input_port_3, _input_port_4, _output_port_0, _output_port_1};
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
        //depth=provider->data.x + offset.y;
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

    R_body_to_inertial_temp=R_body_to_inertial_temp.transpose().eval();

    Vector3D<float> t_results;
    t_results.x = drone_camera_offset.x * R_body_to_inertial_temp(0, 0) + drone_camera_offset.y * R_body_to_inertial_temp(0, 1) + drone_camera_offset.z * R_body_to_inertial_temp(0, 2);
    t_results.y = drone_camera_offset.x * R_body_to_inertial_temp(1, 0) + drone_camera_offset.y * R_body_to_inertial_temp(1, 1) + drone_camera_offset.z * R_body_to_inertial_temp(1, 2);
    t_results.z = drone_camera_offset.x * R_body_to_inertial_temp(2, 0) + drone_camera_offset.y * R_body_to_inertial_temp(2, 1) + drone_camera_offset.z * R_body_to_inertial_temp(2, 2);

    return t_results;
}

Vector3D<float> plane_line_intersector::get_object_location()    
{
    Vector3D<double> line_p1,line_p2;
    Vector3D<double> offset=rotate_offset();

    float depth_adjusted_for_offset=depth+offset.y;

    projection_plane.p0.y=inertial_plane_offset-depth_adjusted_for_offset;
    projection_plane.p1.y=inertial_plane_offset-depth_adjusted_for_offset;
    projection_plane.p2.y=inertial_plane_offset-depth_adjusted_for_offset;

    line_p1.x=0;
    line_p1.y=0;
    line_p1.z=0;

    line_p2=rotated_unit_vector * 20.;


    Vector3D<double> intersection_pt= projection_plane.getIntersectingLine(line_p1,line_p2);

    Vector3D<double> data_transmitted;


    data_transmitted.x=(intersection_pt.x) * -1;
    data_transmitted.y=intersection_pt.y;
    data_transmitted.z=(intersection_pt.z) * -1;

    Vector3DMsg point_msg;
    point_msg.data = data_transmitted;
    this->_output_port_0->receiveMsgData(&point_msg);

    
    Vector3D<double> data_transmitted_with_offset;

    // data_transmitted_with_offset.x=(intersection_pt.x * -1)+offset.x;
    // data_transmitted_with_offset.y=intersection_pt.y;
    // data_transmitted_with_offset.z=(intersection_pt.z * -1)+offset.z;

    data_transmitted_with_offset.x=offset.x;
    data_transmitted_with_offset.y=offset.y;
    data_transmitted_with_offset.z=offset.z;

    Vector3DMsg point_and_offset_msg;
    point_and_offset_msg.data = data_transmitted_with_offset;
    this->_output_port_1->receiveMsgData(&point_and_offset_msg);

}