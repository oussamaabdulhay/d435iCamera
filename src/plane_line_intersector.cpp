#include "plane_line_intersector.hpp"
using namespace std;

plane_line_intersector::plane_line_intersector()
{
    drone_camera_offset.x = 0;
    drone_camera_offset.y = -0.15;
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
    this->_output_port_0 = new OutputPort(ports_id::OP_0_DATA, this);

    _ports = {_input_port_0, _input_port_1, _output_port_0};
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
    }
    
    else if(t_port->getID() == ports_id::IP_1_DEPTH_DATA) //TODO: Caution about update rate
    { 
        depth=provider->data.x;
        
        double current_depth;

        Vector3D<double> line_p1,line_p2;
    
        projection_plane.p0.y=inertial_plane_offset-depth;
        projection_plane.p1.y=inertial_plane_offset-depth;
        projection_plane.p2.y=inertial_plane_offset-depth;

        line_p1.x=0;
        line_p1.y=0;
        line_p1.z=0;

        line_p2=rotated_unit_vector * 20;

    
        Vector3D<double> intersection_pt= projection_plane.getIntersectingLine(line_p1,line_p2);

        Vector3D<double> data_transmitted;


        data_transmitted.x=intersection_pt.x - drone_camera_offset.x;
        data_transmitted.y=intersection_pt.y - drone_camera_offset.y;
        data_transmitted.z=(intersection_pt.z - drone_camera_offset.z) * -1;

    

        Vector3DMsg point_msg;
        point_msg.data = data_transmitted;
        this->_output_port_0->receiveMsgData(&point_msg);


    }
}
