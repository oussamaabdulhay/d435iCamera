#include "rayrotation_rgb.hpp"
using namespace std;

rayrotation_rgb::rayrotation_rgb()
{
    U_v.x = 1;
    U_v.y = 0;
    U_v.z = 0;

    
    this->_input_port_0 = new InputPort(ports_id::IP_0_CAMERA, this);
    this->_input_port_1 = new InputPort(ports_id::IP_1_ROLL, this);
    this->_input_port_2 = new InputPort(ports_id::IP_2_PITCH, this);
    this->_input_port_3 = new InputPort(ports_id::IP_3_YAW, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port_0, _input_port_1,_input_port_2 ,_input_port_3,_output_port};
}

rayrotation_rgb::~rayrotation_rgb()
{
}

void rayrotation_rgb::process(DataMsg* t_msg, Port* t_port) {
    Vector3DMsg *provider = (Vector3DMsg *)t_msg;

    if(t_port->getID() == ports_id::IP_0_CAMERA)
    {
        Vector2DMsg* pixel_location = (Vector2DMsg*) t_msg;
        ball_location.x=pixel_location->data.x;
        ball_location.y=pixel_location->data.y;
        update_camera_angles();
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

MatrixXd rayrotation_rgb::MultiplyMatrices(MatrixXd R_inertia, MatrixXd R_drone)
{

    MatrixXd rotated_matrix(3, 3);
    rotated_matrix(0, 0) = R_drone(0, 0) * R_inertia(0, 0) + R_drone(0, 1) * R_inertia(1, 0) + R_drone(0, 2) * R_inertia(2, 0);

    rotated_matrix(1, 0) = R_drone(1, 0) * R_inertia(0, 0) + R_drone(1, 1) * R_inertia(1, 0) + R_drone(1, 2) * R_inertia(2, 0);

    rotated_matrix(2, 0) = R_drone(2, 0) * R_inertia(0, 0) + R_drone(2, 1) * R_inertia(1, 0) + R_drone(2, 2) * R_inertia(2, 0);

    rotated_matrix(0, 1) = R_drone(0, 0) * R_inertia(0, 1) + R_drone(0, 1) * R_inertia(1, 1) + R_drone(0, 2) * R_inertia(2, 1);

    rotated_matrix(1, 1) = R_drone(1, 0) * R_inertia(0, 1) + R_drone(1, 1) * R_inertia(1, 1) + R_drone(1, 2) * R_inertia(2, 1);

    rotated_matrix(2, 1) = R_drone(2, 0) * R_inertia(0, 1) + R_drone(2, 1) * R_inertia(1, 1) + R_drone(2, 2) * R_inertia(2, 1);

    rotated_matrix(0, 2) = R_drone(0, 0) * R_inertia(0, 2) + R_drone(0, 1) * R_inertia(1, 2) + R_drone(0, 2) * R_inertia(2, 2);

    rotated_matrix(1, 2) = R_drone(1, 0) * R_inertia(0, 2) + R_drone(1, 1) * R_inertia(1, 2) + R_drone(1, 2) * R_inertia(2, 2);

    rotated_matrix(2, 2) = R_drone(2, 0) * R_inertia(0, 2) + R_drone(2, 1) * R_inertia(1, 2) + R_drone(2, 2) * R_inertia(2, 2);
    return rotated_matrix;
}

Vector3D<float> rayrotation_rgb::Update_unit_vector(MatrixXd rotated_matrix)
{
    Vector3D<float> t_results;
    t_results.x = U_v.x * rotated_matrix(0, 0) + U_v.y * rotated_matrix(0, 1) + U_v.z * rotated_matrix(0, 2);
    t_results.y = U_v.x * rotated_matrix(1, 0) + U_v.y * rotated_matrix(1, 1) + U_v.z * rotated_matrix(1, 2);
    t_results.z = U_v.x * rotated_matrix(2, 0) + U_v.y * rotated_matrix(2, 1) + U_v.z * rotated_matrix(2, 2);

 
    
    obj_pos.x=0;
    obj_pos.y=t_results.y * 100 * -1;
    obj_pos.z=t_results.z * 100 * -1;

    Vector3DMsg point_msg;
    point_msg.data = obj_pos;
    this->_output_port->receiveMsgData(&point_msg);

    return t_results;
}


void rayrotation_rgb::update_camera_angles()
{
    float theta_yaw =  -1*(1.2043 / 640.0) * ball_location.x;
    float theta_roll = (0.7330 / 480.0) * ball_location.y;

    camera_angle.x = theta_roll;
    camera_angle.y = 0;
    camera_angle.z = theta_yaw;

    this->update_rotation_matrices();
}

void rayrotation_rgb::update_rotation_matrices()    
{

    MatrixXd R_drone(3, 3);
    MatrixXd R_inertia(3, 3);
    MatrixXd rotated_matrix(3, 3);

    R_inertia = R_o_d.Update(drone_orientation); //Create the rotation matrices
    R_drone = R_d_c.Update(camera_angle);

    R_inertia=R_inertia.inverse();

    rotated_matrix = MultiplyMatrices(R_inertia, R_drone); //Multiply the rotation matrices;

    rotated_unit_vector = Update_unit_vector(rotated_matrix);

}