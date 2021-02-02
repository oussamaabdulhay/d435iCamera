#include "test_rotation.hpp"
using namespace std;

test_rotation::test_rotation()
{
    f_c=-616.5;
    
    this->_input_port_0 = new InputPort(ports_id::IP_0_CAMERA, this);
    this->_input_port_1 = new InputPort(ports_id::IP_1_ROLL, this);
    this->_input_port_2 = new InputPort(ports_id::IP_2_PITCH, this);
    this->_input_port_3 = new InputPort(ports_id::IP_3_YAW, this);
    this->_output_port_0 = new OutputPort(ports_id::OP_0_DATA, this);
    this->_output_port_1 = new OutputPort(ports_id::OP_CAMERA_ANGLES_DATA, this);
    this->_output_port_2 = new OutputPort(ports_id::OP_PIXEL_DATA, this);
    _ports = {_input_port_0, _input_port_1,_input_port_2 ,_input_port_3,_output_port_0, _output_port_1, _output_port_2};
}

test_rotation::~test_rotation()
{
}

void test_rotation::process(DataMsg* t_msg, Port* t_port) {
    Vector3DMsg *provider = (Vector3DMsg *)t_msg;

    if(t_port->getID() == ports_id::IP_0_CAMERA)
    {
        Vector2DMsg* pixel_location = (Vector2DMsg*) t_msg;
        ball_location.x=-1 * pixel_location->data.x;
        ball_location.y= pixel_location->data.y;

        update_camera_vector();

        Vector2DMsg pixel_data_raw_msg;
        pixel_data_raw_msg.data = ball_location;
        this->_output_port_2->receiveMsgData(&pixel_data_raw_msg);
        
    }
    else if(t_port->getID() == ports_id::IP_1_ROLL)
    { 
        drone_orientation.x =provider->data.x;
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


void test_rotation::Update_unit_vector(Eigen::Matrix<float,3,3> rotated_matrix)
{
    Vector3D<float> t_results;
    t_results.x = camera_vector.x * rotated_matrix(0, 0) + camera_vector.y * rotated_matrix(0, 1) + camera_vector.z * rotated_matrix(0, 2);
    t_results.y = camera_vector.x * rotated_matrix(1, 0) + camera_vector.y * rotated_matrix(1, 1) + camera_vector.z * rotated_matrix(1, 2);
    t_results.z = camera_vector.x * rotated_matrix(2, 0) + camera_vector.y * rotated_matrix(2, 1) + camera_vector.z * rotated_matrix(2, 2);

 
    
    obj_pos.x=t_results.x;
    obj_pos.y=t_results.y;
    obj_pos.z=t_results.z;

    Vector3DMsg point_msg;
    point_msg.data = obj_pos;
    this->_output_port_0->receiveMsgData(&point_msg);
}


void test_rotation::update_camera_vector()
{
    camera_vector.x = f_c;
    camera_vector.y = ball_location.x;
    camera_vector.z = ball_location.y;

    Vector3DMsg camera_vector_raw_msg;
    camera_vector_raw_msg.data = camera_vector;
    this->_output_port_1->receiveMsgData(&camera_vector_raw_msg);

    this->update_rotation_matrices();
}

void test_rotation::update_rotation_matrices()    
{
    Eigen::Matrix<float, 3, 3> R_i_d_temp(3, 3);

    R_i_d_temp = R_i_d.Update(drone_orientation); //Create the rotation matrices

    R_i_d_temp=R_i_d_temp.inverse().eval();

    this->Update_unit_vector(R_i_d_temp);
}