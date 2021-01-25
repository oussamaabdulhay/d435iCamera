#include "test_rotation.hpp"
using namespace std;

test_rotation::test_rotation()
{
    U_v.x = 1;
    U_v.y = 0;
    U_v.z = 0;

    
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

        update_camera_angles();

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

MatrixXd test_rotation::MultiplyMatrices(MatrixXd R_1, MatrixXd R_2)
{
    MatrixXd rotated_matrix(3, 3);
    rotated_matrix(0, 0) = R_1(0, 0) * R_2(0, 0) + R_1(0, 1) * R_2(1, 0) + R_1(0, 2) * R_2(2, 0);

    rotated_matrix(1, 0) = R_1(1, 0) * R_2(0, 0) + R_1(1, 1) * R_2(1, 0) + R_1(1, 2) * R_2(2, 0);

    rotated_matrix(2, 0) = R_1(2, 0) * R_2(0, 0) + R_1(2, 1) * R_2(1, 0) + R_1(2, 2) * R_2(2, 0);

    rotated_matrix(0, 1) = R_1(0, 0) * R_2(0, 1) + R_1(0, 1) * R_2(1, 1) + R_1(0, 2) * R_2(2, 1);

    rotated_matrix(1, 1) = R_1(1, 0) * R_2(0, 1) + R_1(1, 1) * R_2(1, 1) + R_1(1, 2) * R_2(2, 1);

    rotated_matrix(2, 1) = R_1(2, 0) * R_2(0, 1) + R_1(2, 1) * R_2(1, 1) + R_1(2, 2) * R_2(2, 1);

    rotated_matrix(0, 2) = R_1(0, 0) * R_2(0, 2) + R_1(0, 1) * R_2(1, 2) + R_1(0, 2) * R_2(2, 2);

    rotated_matrix(1, 2) = R_1(1, 0) * R_2(0, 2) + R_1(1, 1) * R_2(1, 2) + R_1(1, 2) * R_2(2, 2);

    rotated_matrix(2, 2) = R_1(2, 0) * R_2(0, 2) + R_1(2, 1) * R_2(1, 2) + R_1(2, 2) * R_2(2, 2);
    return rotated_matrix;
}

void test_rotation::Update_unit_vector(MatrixXd rotated_matrix)
{
    Vector3D<float> t_results;
    t_results.x = U_v.x * rotated_matrix(0, 0) + U_v.y * rotated_matrix(0, 1) + U_v.z * rotated_matrix(0, 2);
    t_results.y = U_v.x * rotated_matrix(1, 0) + U_v.y * rotated_matrix(1, 1) + U_v.z * rotated_matrix(1, 2);
    t_results.z = U_v.x * rotated_matrix(2, 0) + U_v.y * rotated_matrix(2, 1) + U_v.z * rotated_matrix(2, 2);

 
    
    obj_pos.x=t_results.x;
    obj_pos.y=t_results.y;
    obj_pos.z=t_results.z;

    Vector3DMsg point_msg;
    point_msg.data = obj_pos;
    this->_output_port_0->receiveMsgData(&point_msg);
}


void test_rotation::update_camera_angles()
{
    float theta_yaw =  (1.2043 / 640.0) * ball_location.x;
    float theta_pitch = (0.7330 / 480.0) * ball_location.y;

    camera_angle.x = 0;
    camera_angle.y = theta_pitch;
    camera_angle.z = theta_yaw;

    Vector3DMsg camera_angles_raw_msg;
    camera_angles_raw_msg.data = camera_angle;
    this->_output_port_1->receiveMsgData(&camera_angles_raw_msg);

    this->update_rotation_matrices();
}

void test_rotation::update_rotation_matrices()    
{
    MatrixXd R_d_c_temp(3, 3);
    MatrixXd R_i_d_temp(3, 3);
    MatrixXd rotated_matrix(3, 3);

    R_i_d_temp = R_i_d.Update(drone_orientation); //Create the rotation matrices
    R_d_c_temp = R_d_c.Update(camera_angle);

    //R_d_c_temp=R_d_c_temp.transpose().eval();
    rotated_matrix = MultiplyMatrices(R_i_d_temp, R_d_c_temp); //Multiply the rotation matrices;
    //std::cout<<"PITCH - cam=  "<<camera_angle.y<<std::endl;
    //std::cout<<"PITCH - drne= "<<drone_orientation.y<<std::endl;
    // std::cout<<"YAW="<<drone_orientation.z<<std::endl;
    this->Update_unit_vector(rotated_matrix);
}