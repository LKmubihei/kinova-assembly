#include "hyy_external_controller/hyy_external_controller.hpp"

namespace hyy_external_controller
{

HyyExternalController::HyyExternalController() :
if_add_force_sensor(0),
if_add_hand(0),
if_add_gripper(0),
start_controller(false)
{
    gripper_joint_degree = 0;
    hand_joints_position = {0, 0, 0, 0, 0, 0};
}

HyyExternalController::~HyyExternalController(){}

controller_interface::CallbackReturn HyyExternalController::on_init() {

    try{

        param_listener_ = std::make_shared<ParamListener>(get_node());
        if (!param_listener_)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
            return controller_interface::CallbackReturn::ERROR;
        }
        params_ = param_listener_->get_params();
        if_add_force_sensor = params_.if_add_force_sensor;
        if_add_gripper = params_.if_add_gripper;
        if_add_hand = params_.if_add_hand;

        if(if_add_force_sensor){
            if (params_.force_sensor_name.empty()){
                RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter was empty");
                return controller_interface::CallbackReturn::ERROR;
            }else{
                for (const auto & sensor : params_.force_sensor_name){
                    sensors_name.push_back(sensor);
                }
            }
        }
        if(if_add_gripper){
            port_gripper = params_.port_gripper;
            baudrate_gripper = params_.baudrate_gripper;
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "port_gripper: " << port_gripper << "  baudrate_gripper:" << baudrate_gripper);
        }
        if (if_add_gripper){
            port_hand= params_.port_hand;
            baudrate_hand = params_.baudrate_hand;
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "port_hand: " << port_hand << "  baudrate_hand:" << baudrate_hand);
        }
        
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;

}

controller_interface::CallbackReturn HyyExternalController::on_configure(const rclcpp_lifecycle::State &previous_state) {

    /*
    ** hand configure
    */

    if (if_add_hand){
        ros_ser.setPort(port_hand);
        ros_ser.setBaudrate(baudrate_hand);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ros_ser.setTimeout(to);
        try{
            ros_ser.open();
        }catch(serial::IOException &e){
            RCLCPP_INFO(get_node()->get_logger(), "Hand serial unable to open.");
            return controller_interface::CallbackReturn::ERROR;
        }
        if(ros_ser.isOpen()){
            RCLCPP_INFO(get_node()->get_logger(), "Hand serial open success.");
        }else{
            return controller_interface::CallbackReturn::ERROR;
        }

        Setangle_Server = get_node()->create_service<hyySetangleMsg>("~/Setangle",
                                    std::bind(&HyyExternalController::setangle_callback,this,std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setangle);
        if(!Setangle_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Setangle.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Setangle is now ready.");
        }

        Setpos_Server = get_node()->create_service<hyySetposMsg>("~/Setpos",
                                    std::bind(&HyyExternalController::setpos_callback,this,std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setpos);
        if(!Setpos_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Setpos.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Setpos is now ready.");
        }

        Setspeed_Server = get_node()->create_service<hyySetspeedMsg>("~/Setspeed",
                                    std::bind(&HyyExternalController::setspeed_callback,this,std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setspeed);
        if(!Setspeed_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Setspeed.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Setspeed is now ready.");
        }

        Setforce_Server = get_node()->create_service<hyySetforceMsg>("~/Setforce",
                                    std::bind(&HyyExternalController::setforce_callback,this,std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setforce);
        if(!Setforce_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Setforce.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Setforce is now ready.");
        }

        Getangleact_Server = get_node()->create_service<hyyGetangleactMsg>("~/Getangleact",
                                    std::bind(&HyyExternalController::getangleact_callback, this, std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getangleact);
        if(!Getangleact_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Getangleact.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Getangleact is now ready.");
        }

        Getangleset_Server = get_node()->create_service<hyyGetanglesetMsg>("~/Getangleset",
                                    std::bind(&HyyExternalController::getangleset_callback, this, std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getangleset);
        if(!Getangleset_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Getangleset.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Getangleset is now ready.");
        }

        Getposact_Server = get_node()->create_service<hyyGetposactMsg>("~/Getposact",
                                    std::bind(&HyyExternalController::getposact_callback, this, std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getposact);
        if(!Getposact_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Getposact.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Getposact is now ready.");
        }

        Getposset_Server = get_node()->create_service<hyyGetpossetMsg>("~/Getposset",
                                    std::bind(&HyyExternalController::getposset_callback, this, std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getposset);
        if(!Getposset_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Getposset.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Getposset is now ready.");
        }

        Getspeedset_Server = get_node()->create_service<hyyGetspeedsetMsg>("~/Getspeedset",
                                    std::bind(&HyyExternalController::getspeedset_callback, this, std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getspeedset);
        if(!Getspeedset_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Getspeedset.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Getspeedset is now ready.");
        }

        Getforceact_Server = get_node()->create_service<hyyGetforceactMsg>("~/Getforceact",
                                    std::bind(&HyyExternalController::getforceact_callback, this, std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getforceact);
        if(!Getforceact_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Getforceact.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Getforceact is now ready.");
        }

        Getforceset_Server = get_node()->create_service<hyyGetforcesetMsg>("~/Getforceset",
                                    std::bind(&HyyExternalController::getforceset_callback, this, std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getforceset);
        if(!Getforceset_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Getforceset.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Getforceset is now ready.");
        }

        Getcurrentact_Server = get_node()->create_service<hyyGetcurrentactMsg>("~/Getcurrentact",
                                    std::bind(&HyyExternalController::getcurrentact_callback, this, std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getcurrentact);
        if(!Getcurrentact_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Getcurrentact.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Getcurrentact is now ready.");
        }

        Geterror_Server = get_node()->create_service<hyyGeterrorMsg>("~/Geterror",
                                    std::bind(&HyyExternalController::geterror_callback, this, std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_geterror);
        if(!Geterror_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Geterror.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Geterror is now ready.");
        }
                        
        Gettemp_Server = get_node()->create_service<hyyGettempMsg>("~/Gettemp",
                                    std::bind(&HyyExternalController::gettemp_callback, this, std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_gettemp);
        if(!Gettemp_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service Gettemp.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/Gettemp is now ready.");
        }

    }

    /*
    * gripper configure
    */

    if (if_add_gripper){
        gripper_ser.setPort(port_gripper);
        gripper_ser.setBaudrate(baudrate_gripper);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        gripper_ser.setTimeout(to);
        try{
            gripper_ser.open();
        }catch(serial::IOException &e){
            RCLCPP_INFO(get_node()->get_logger(), "Gripper serial unable to open.");
            return controller_interface::CallbackReturn::ERROR;
        }
        if(gripper_ser.isOpen()){
            RCLCPP_INFO(get_node()->get_logger(), "Gripper serial open success.");
        }else{
            return controller_interface::CallbackReturn::ERROR;
        }

        ControlGripper_Server = get_node()->create_service<hyyGripMsg>("~/ControlGripper",
                                    std::bind(&HyyExternalController::controlgripper_callback,this,std::placeholders::_1,std::placeholders::_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_controlgripper);
        if(!ControlGripper_Server){
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service ControlGripper.");
        }else{
            RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/ControlGripper is now ready.");
        }
    }

    /*
    * force sensor configure
    */

    if (if_add_force_sensor){
    
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration HyyExternalController::command_interface_configuration() const {


    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & joint : params_.joints){
        for (const auto & interface_name : params_.command_interfaces){
             command_interfaces_config.names.push_back(joint + "/" + interface_name);
        }
    }

    return command_interfaces_config;

}

controller_interface::InterfaceConfiguration HyyExternalController::state_interface_configuration() const {

    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &joint : params_.joints){
        for (const auto & interface_name : params_.state_interfaces){
            state_interfaces_config.names.push_back(joint + "/" + interface_name);
        }
    }

    return state_interfaces_config;
}

controller_interface::CallbackReturn HyyExternalController::on_activate(const rclcpp_lifecycle::State &previous_state) {
    
    start_controller = true;
    
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HyyExternalController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
   
    start_controller = false;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type HyyExternalController::update(const rclcpp::Time &time, const rclcpp::Duration &period) {

	if (start_controller)
	{
        hand_lock.lock();
        gripper_lock.lock();
        std::vector<double> externalDevicePosition = hand_joints_position;
        externalDevicePosition.push_back(gripper_joint_degree);
        gripper_lock.unlock();
        hand_lock.unlock();
        for (auto index = 0ul; index < command_interfaces_.size(); ++index)
        {
            command_interfaces_[index].set_value(externalDevicePosition[index]);
        }
    }
    
    return controller_interface::return_type::OK;
}

void HyyExternalController::setangle_callback(const hyySetangleMsg::Request::SharedPtr request,
                            const hyySetangleMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "set_angle")
    {
        // 打印指令类型
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x0F;
        send_buffer[4] = 0x12;
        send_buffer[5] = 0xCE;
        send_buffer[6] = 0x05;
        send_buffer[7] = (request->angle[0] & 0xFF);
        send_buffer[8] = ((request->angle[0] >> 8) & 0xFF);
        send_buffer[9] = (request->angle[1] & 0xFF);
        send_buffer[10] = ((request->angle[1] >> 8) & 0xFF);
        send_buffer[11] = (request->angle[2] & 0xFF);
        send_buffer[12] = ((request->angle[2] >> 8) & 0xFF);
        send_buffer[13] = (request->angle[3] & 0xFF);
        send_buffer[14] = ((request->angle[3] >> 8) & 0xFF);
        send_buffer[15] = (request->angle[4] & 0xFF);
        send_buffer[16] = ((request->angle[4] >> 8) & 0xFF);
        send_buffer[17] = (request->angle[5] & 0xFF);
        send_buffer[18] = ((request->angle[5] >> 8) & 0xFF);
        for(int i = 2;i < 19;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[19] = check_sum;
        ros_ser.write(send_buffer,20);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[7] == 0x01)
            {
                response->angle_accepted = true;
                RCLCPP_INFO(get_node()->get_logger(), "set command successed.");
            }
            else
            {
                response->angle_accepted = false;
                RCLCPP_INFO(get_node()->get_logger(), "set command failed.");
            }
        }
        hand_lock.lock();
        hand_joints_position = store_hand_state(request->angle);
        hand_lock.unlock();
    }
    else
    {
        //设置指令报错
        response->angle_accepted = false;
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::setpos_callback(const hyySetposMsg::Request::SharedPtr request,
                            const hyySetposMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "set_pos")
    {
        // 打印指令类型
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x0F;
        send_buffer[4] = 0x12;
        send_buffer[5] = 0xC2;
        send_buffer[6] = 0x05;
        send_buffer[7] = (request->pos[0] & 0xFF);
        send_buffer[8] = ((request->pos[0] >> 8) & 0xFF);
        send_buffer[9] = (request->pos[1] & 0xFF);
        send_buffer[10] = ((request->pos[1] >> 8) & 0xFF);
        send_buffer[11] = (request->pos[2] & 0xFF);
        send_buffer[12] = ((request->pos[2] >> 8) & 0xFF);
        send_buffer[13] = (request->pos[3] & 0xFF);
        send_buffer[14] = ((request->pos[3] >> 8) & 0xFF);
        send_buffer[15] = (request->pos[4] & 0xFF);
        send_buffer[16] = ((request->pos[4] >> 8) & 0xFF);
        send_buffer[17] = (request->pos[5] & 0xFF);
        send_buffer[18] = ((request->pos[5] >> 8) & 0xFF);
        for(int i = 2;i < 19;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[19] = check_sum;
        ros_ser.write(send_buffer,20);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[7] == 0x01)
            {
                response->pos_accepted = true;
                RCLCPP_INFO(get_node()->get_logger(), "set command successed.");
            }
            else
            {
                response->pos_accepted = false;
                RCLCPP_INFO(get_node()->get_logger(), "set command failed.");
            }
        }
    }
    else
    {
        //设置指令报错
        response->pos_accepted = false;
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::setspeed_callback(const hyySetspeedMsg::Request::SharedPtr request,
                            const hyySetspeedMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "set_speed")
    {
        // 打印指令类型
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x0F;
        send_buffer[4] = 0x12;
        send_buffer[5] = 0xF2;
        send_buffer[6] = 0x05;
        send_buffer[7] = (request->speed[0] & 0xFF);
        send_buffer[8] = ((request->speed[0] >> 8) & 0xFF);
        send_buffer[9] = (request->speed[1] & 0xFF);
        send_buffer[10] = ((request->speed[1] >> 8) & 0xFF);
        send_buffer[11] = (request->speed[2] & 0xFF);
        send_buffer[12] = ((request->speed[2] >> 8) & 0xFF);
        send_buffer[13] = (request->speed[3] & 0xFF);
        send_buffer[14] = ((request->speed[3] >> 8) & 0xFF);
        send_buffer[15] = (request->speed[4] & 0xFF);
        send_buffer[16] = ((request->speed[4] >> 8) & 0xFF);
        send_buffer[17] = (request->speed[5] & 0xFF);
        send_buffer[18] = ((request->speed[5] >> 8) & 0xFF);
        for(int i = 2;i < 19;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[19] = check_sum;
        ros_ser.write(send_buffer,20);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[7] == 0x01)
            {
                response->speed_accepted = true;
                RCLCPP_INFO(get_node()->get_logger(), "set command successed.");

            }
            else
            {
                response->speed_accepted = false;
                RCLCPP_INFO(get_node()->get_logger(), "set command failed.");
            }
        }
    }
    else
    {
        //设置指令报错
        response->speed_accepted = false;
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::setforce_callback(const hyySetforceMsg::Request::SharedPtr request,
                            const hyySetforceMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "set_force")
    {
        // 打印指令类型
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x0F;
        send_buffer[4] = 0x12;
        send_buffer[5] = 0xDA;
        send_buffer[6] = 0x05;
        send_buffer[7] = (request->force[0] & 0xFF);
        send_buffer[8] = ((request->force[0] >> 8) & 0xFF);
        send_buffer[9] = (request->force[1] & 0xFF);
        send_buffer[10] = ((request->force[1] >> 8) & 0xFF);
        send_buffer[11] = (request->force[2] & 0xFF);
        send_buffer[12] = ((request->force[2] >> 8) & 0xFF);
        send_buffer[13] = (request->force[3] & 0xFF);
        send_buffer[14] = ((request->force[3] >> 8) & 0xFF);
        send_buffer[15] = (request->force[4] & 0xFF);
        send_buffer[16] = ((request->force[4] >> 8) & 0xFF);
        send_buffer[17] = (request->force[5] & 0xFF);
        send_buffer[18] = ((request->force[5] >> 8) & 0xFF);
        for(int i = 2;i < 19;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[19] = check_sum;
        ros_ser.write(send_buffer,20);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[7] == 0x01)
            {
                response->force_accepted = true;
                RCLCPP_INFO(get_node()->get_logger(), "set command successed.");

            }
            else
            {
                response->force_accepted = false;
                RCLCPP_INFO(get_node()->get_logger(), "set command failed.");
            }
        }
    }
    else
    {
        //设置指令报错
        response->force_accepted = false;
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::getangleact_callback(const hyyGetangleactMsg::Request::SharedPtr request,
                            const hyyGetangleactMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "get_angleact")
    {
        // 打印请求
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x04;
        send_buffer[4] = 0x11;
        send_buffer[5] = 0x0A;
        send_buffer[6] = 0x06;
        send_buffer[7] = 0x0C;
        for(int i = 2;i < 8;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[8] = check_sum;
        ros_ser.write(send_buffer,9);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[4] == 0x11)
            {
                response->curangleact.clear();
                for (int i = 0; i < 6; i++)
                {
                    response->curangleact.push_back((recv_buffer[7 + i * 2] & 0xFF) + ((recv_buffer[8 + i * 2] << 8) & 0xFF00));
                }
                RCLCPP_INFO(get_node()->get_logger(), "actual angle of hand: %d\t%d\t%d\t%d\t%d\t%d",
                            response->curangleact[0], response->curangleact[1], response->curangleact[2],
                            response->curangleact[3], response->curangleact[4], response->curangleact[5]);
            }else{
                RCLCPP_INFO(get_node()->get_logger(), "can't get hand actual angle");
            }
        }
    }
    else
    {
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::getangleset_callback(const hyyGetanglesetMsg::Request::SharedPtr request,
                            const hyyGetanglesetMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "get_angleset")
    {
        // 打印请求
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x04;
        send_buffer[4] = 0x11;
        send_buffer[5] = 0xCE;
        send_buffer[6] = 0x05;
        send_buffer[7] = 0x0C;
        for(int i = 2;i < 8;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[8] = check_sum;
        ros_ser.write(send_buffer,9);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[4] == 0x11)
            {
                response->curangleset.clear();
                for (int i = 0; i < 6; i++)
                {
                    response->curangleset.push_back((recv_buffer[7 + i * 2] & 0xFF) + ((recv_buffer[8 + i * 2] << 8) & 0xFF00));
                }
                RCLCPP_INFO(get_node()->get_logger(), "get desired hand angle: %d\t%d\t%d\t%d\t%d\t%d",
                            response->curangleset[0], response->curangleset[1], response->curangleset[2],
                            response->curangleset[3], response->curangleset[4], response->curangleset[5]);
            }else{
                RCLCPP_INFO(get_node()->get_logger(), "can't get desired hand angle");
            }
        }
    }
    else
    {
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::getposact_callback(const hyyGetposactMsg::Request::SharedPtr request,
                            const hyyGetposactMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "get_posact")
    {
        // 打印请求
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x04;
        send_buffer[4] = 0x11;
        send_buffer[5] = 0xFE;
        send_buffer[6] = 0x05;
        send_buffer[7] = 0x0C;
        for(int i = 2;i < 8;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[8] = check_sum;
        ros_ser.write(send_buffer,9);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[4] == 0x11)
            {
                response->curposact.clear();
                for (int i = 0; i < 6; i++)
                {
                    response->curposact.push_back((recv_buffer[7 + i * 2] & 0xFF) + ((recv_buffer[8 + i * 2] << 8) & 0xFF00));
                }
                RCLCPP_INFO(get_node()->get_logger(), "get actual driver location: %d\t%d\t%d\t%d\t%d\t%d",
                            response->curposact[0], response->curposact[1], response->curposact[2],
                            response->curposact[3], response->curposact[4], response->curposact[5]);
            }else{
                RCLCPP_INFO(get_node()->get_logger(), "can't get actual driver location\n");
            }
        }
    }
    else
    {
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::getposset_callback(const hyyGetpossetMsg::Request::SharedPtr request,
                            const hyyGetpossetMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "get_posset")
    {
        // 打印请求
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x04;
        send_buffer[4] = 0x11;
        send_buffer[5] = 0xC2;
        send_buffer[6] = 0x05;
        send_buffer[7] = 0x0C;
        for(int i = 2;i < 8;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[8] = check_sum;
        ros_ser.write(send_buffer,9);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[4] == 0x11)
            {
                response->curposset.clear();
                for (int i = 0; i < 6; i++)
                {
                    response->curposset.push_back((recv_buffer[7 + i * 2] & 0xFF) + ((recv_buffer[8 + i * 2] << 8) & 0xFF00));
                }
                RCLCPP_INFO(get_node()->get_logger(), "get desired driver location: %d\t%d\t%d\t%d\t%d\t%d", 
                response->curposset[0],response->curposset[1],response->curposset[2],response->curposset[3],response->curposset[4],response->curposset[5]);
            }
            else
            {
                RCLCPP_INFO(get_node()->get_logger(), "can't get desired driver location\n");
            } 
        }
    }
    else
    {
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::getspeedset_callback(const hyyGetspeedsetMsg::Request::SharedPtr request,
                            const hyyGetspeedsetMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "get_speedset")
    {
        // 打印请求
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x04;
        send_buffer[4] = 0x11;
        send_buffer[5] = 0xF2;
        send_buffer[6] = 0x05;
        send_buffer[7] = 0x0C;
        for(int i = 2;i < 8;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[8] = check_sum;
        ros_ser.write(send_buffer,9);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[4] == 0x11)
            {
                response->curspeedset.clear();
                for (int i = 0; i < 6; i++)
                {
                    response->curspeedset.push_back((recv_buffer[7 + i * 2] & 0xFF) + ((recv_buffer[8 + i * 2] << 8) & 0xFF00));
                }
                RCLCPP_INFO(get_node()->get_logger(), "get desired hand speed: %d\t%d\t%d\t%d\t%d\t%d",
                            response->curspeedset[0], response->curspeedset[1], response->curspeedset[2],
                            response->curspeedset[3], response->curspeedset[4], response->curspeedset[5]);
            }
            else
            {
                RCLCPP_INFO(get_node()->get_logger(),"can't get desired hand speed");
            } 
        }
    }
    else
    {
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::getforceact_callback(const hyyGetforceactMsg::Request::SharedPtr request,
                            const hyyGetforceactMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "get_forceact")
    {
        // 打印请求
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x04;
        send_buffer[4] = 0x11;
        send_buffer[5] = 0x2E;
        send_buffer[6] = 0x06;
        send_buffer[7] = 0x0C;
        for(int i = 2;i < 8;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[8] = check_sum;
        ros_ser.write(send_buffer,9);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[4] == 0x11)
            {
                response->curforceact.clear();
                for (int i = 0; i < 6; i++)
                {
                    response->curforceact.push_back((recv_buffer[7 + i * 2] & 0xFF) + ((recv_buffer[8 + i * 2] << 8) & 0xFF00));
                }
                RCLCPP_INFO(get_node()->get_logger(), "get actual hand force: %d\t%d\t%d\t%d\t%d\t%d",
                            response->curforceact[0], response->curforceact[1], response->curforceact[2],
                            response->curforceact[3], response->curforceact[4], response->curforceact[5]);
            }
            else
            {
                RCLCPP_INFO(get_node()->get_logger(), "can't get actual hand force");
            }
        }
    }
    else
    {
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::getforceset_callback(const hyyGetforcesetMsg::Request::SharedPtr request,
                            const hyyGetforcesetMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "get_forceset")
    {
        // 打印请求
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x04;
        send_buffer[4] = 0x11;
        send_buffer[5] = 0xDA;
        send_buffer[6] = 0x05;
        send_buffer[7] = 0x0C;
        for(int i = 2;i < 8;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[8] = check_sum;
        ros_ser.write(send_buffer,9);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[4] == 0x11)
            {
                response->curforceset.clear();
                for (int i = 0; i < 6; i++)
                {
                    response->curforceset.push_back((recv_buffer[7 + i * 2] & 0xFF) + ((recv_buffer[8 + i * 2] << 8) & 0xFF00));
                }
                RCLCPP_INFO(get_node()->get_logger(), "get desired hand force: %d\t%d\t%d\t%d\t%d\t%d",
                response->curforceset[0],response->curforceset[1],response->curforceset[2],response->curforceset[3],response->curforceset[4],response->curforceset[5]);
            }
            else
            {
                RCLCPP_INFO(get_node()->get_logger(), "can't get desired hand force");
            } 
        }
    }
    else
    {
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::getcurrentact_callback(const hyyGetcurrentactMsg::Request::SharedPtr request,
                            const hyyGetcurrentactMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "get_currentact")
    {
        // 打印请求
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x04;
        send_buffer[4] = 0x11;
        send_buffer[5] = 0x3A;
        send_buffer[6] = 0x06;
        send_buffer[7] = 0x0C;
        for(int i = 2;i < 8;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[8] = check_sum;
        ros_ser.write(send_buffer,9);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[4] == 0x11)
            {
                response->curcurrent.clear();
                for (int i = 0; i < 6; i++)
                {
                    response->curcurrent.push_back((recv_buffer[7 + i * 2] & 0xFF) + ((recv_buffer[8 + i * 2] << 8) & 0xFF00));
                }
                RCLCPP_INFO(get_node()->get_logger(), "get actual hand current: %d\t%d\t%d\t%d\t%d\t%d",
                            response->curcurrent[0], response->curcurrent[1], response->curcurrent[2],
                            response->curcurrent[3], response->curcurrent[4], response->curcurrent[5]);
            }
            else
            {
                RCLCPP_INFO(get_node()->get_logger(), "can't get actual hand current");
            } 
        }
    }
    else
    {
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::geterror_callback(const hyyGeterrorMsg::Request::SharedPtr request,
                            const hyyGeterrorMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "get_error")
    {
        // 打印请求
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x04;
        send_buffer[4] = 0x11;
        send_buffer[5] = 0x46;
        send_buffer[6] = 0x06;
        send_buffer[7] = 0x06;
        for(int i = 2;i < 8;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[8] = check_sum;
        ros_ser.write(send_buffer,9);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[4] == 0x11)
            {
                response->error.clear();
                for (int i = 0; i < 6; i++)
                {
                    response->error.push_back((recv_buffer[7 + i] & 0xFF));
                }
                RCLCPP_INFO(get_node()->get_logger(), "get hand error code: %d\t%d\t%d\t%d\t%d\t%d",
                            response->error[0], response->error[1], response->error[2],
                            response->error[3], response->error[4], response->error[5]);
            }
            else
            {
                RCLCPP_INFO(get_node()->get_logger(), "can't get hand error code");
            }
        }
    }
    else
    {
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }
}

void HyyExternalController::gettemp_callback(const hyyGettempMsg::Request::SharedPtr request,
                            const hyyGettempMsg::Response::SharedPtr response)
{
    u_int8_t check_sum = 0;
    rclcpp::WallRate loop_rate(10.0);
    // 首先判断指令类型
    if(request->status == "get_temp")
    {
        // 打印请求
        RCLCPP_INFO(get_node()->get_logger(), "revieve %s repuest", request->status.c_str());
        // 传递数据到数组
        send_buffer[0] = 0xEB;
        send_buffer[1] = 0x90;
        send_buffer[2] = request->hand_id;
        send_buffer[3] = 0x04;
        send_buffer[4] = 0x11;
        send_buffer[5] = 0x52;
        send_buffer[6] = 0x06;
        send_buffer[7] = 0x06;
        for(int i = 2;i < 8;i++)
        {
            check_sum += send_buffer[i];
        }
        send_buffer[8] = check_sum;
        ros_ser.write(send_buffer,9);
        loop_rate.sleep();    //等待100ms接收数据
        int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
        if (count != 0)  //等待接收数据
        {   
            std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
            count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
            if(recv_buffer[4] == 0x11)
            {
                response->temp.clear();
                for (int i = 0; i < 6; i++)
                {
                    response->temp.push_back((recv_buffer[7 + i] & 0xFF));
                }
                RCLCPP_INFO(get_node()->get_logger(), "get hand figure temperature: %d\t%d\t%d\t%d\t%d\t%d",
                            response->temp[0], response->temp[1], response->temp[2],
                            response->temp[3], response->temp[4], response->temp[5]);
            }
            else
            {
                RCLCPP_INFO(get_node()->get_logger(), "can't get hand figure temperature");
            }
        }
    }
    else
    {
        RCLCPP_INFO(get_node()->get_logger(), "revieve a wrong repuest %s", request->status.c_str());
    }

}  // namespace hyy_external_controller

void HyyExternalController::controlgripper_callback(const hyyGripMsg::Request::SharedPtr request,
                            const hyyGripMsg::Response::SharedPtr response){
    if (!request->type.empty()){
        RCLCPP_INFO_STREAM(get_node()->get_logger(), "type: " << request->type << " -----------");
        if(request->type == "set_position"){
            if (request->value >= 0 && request->value <= 255){
                SendCommand(CommandTopic::RequestedPosition, request->value);
                response->result = 0;
                gripper_lock.lock();
                gripper_joint_degree = store_gripper_state(request->value);
                gripper_lock.unlock();
            }else{
                RCLCPP_ERROR(get_node()->get_logger(), "revieve a wrong repuest position :%d", request->value);
                response->result = ERR_TARGET;
            }
        }else if(request->type == "set_speed"){
            if (request->value >= 0 && request->value <= 255){
                SendCommand(CommandTopic::Speed, request->value);
                response->result = 0;
            }else{
                RCLCPP_ERROR(get_node()->get_logger(), "revieve a wrong repuest speed :%d", request->value);
                response->result = ERR_TARGET;
            }
        }else if(request->type == "set_force"){
            if (request->value >= 0 && request->value <= 255){
                SendCommand(CommandTopic::Force, request->value);
                response->result = 0;
            }else{
                RCLCPP_ERROR(get_node()->get_logger(), "revieve a wrong repuest Force :%d", request->value);
                response->result = ERR_TARGET;
            }
        }else if(request->type == "goto"){
            SendCommand(CommandTopic::Move, 1);
            response->result = 0;
        }else if(request->type == "activate"){
            SendCommand(CommandTopic::Activation, 1);
            response->result = 0;
        }else if(request->type == "deactivate"){
            SendCommand(CommandTopic::Activation, 0);
            response->result = 0;
        }else if(request->type == "isactive"){
            sendQuery(QueryTopic::Activation);
            response->result = (modbus_read[3]);
            RCLCPP_INFO(get_node()->get_logger(), "Gripper: get gripper status %d", response->result);
        }else if(request->type == "get_position"){
            sendQuery(QueryTopic::Status);
            response->result = (modbus_read[7]);
            RCLCPP_INFO(get_node()->get_logger(), "Gripper: get gripper position %d", response->result);
        }else{
            RCLCPP_ERROR(get_node()->get_logger(), "Gripper: control command isn't exist.");
            response->result = ERR_MOVETYPE;
            return;
        }
    }else{
        RCLCPP_ERROR(get_node()->get_logger(), "Gripper: control type is empty, please check.");
        response->result = ERR_MOVETYPE_EMPTY;
        return;
    }
    }

void HyyExternalController::SendCommand(CommandTopic property, uint8_t value)
{
    uint8_t address = 0;
    modbus_write.clear();
    int bitField = -1;

    switch (property)
    {
    case CommandTopic::Activation:
        address = static_cast<int>(writeRegisterAddress::ActionRequest);
        bitField = static_cast<int>(ActionRequestBitField::Activation);
        break;
    case CommandTopic::Move:
        address = static_cast<int>(writeRegisterAddress::ActionRequest);
        bitField = static_cast<int>(ActionRequestBitField::GoTo);
        break;
    case CommandTopic::RequestedPosition:
        address = static_cast<int>(writeRegisterAddress::PositionRequest);
        break;
    case CommandTopic::Speed:
        address = static_cast<int>(writeRegisterAddress::Speed);
        break;
    case CommandTopic::Force:
        address = static_cast<int>(writeRegisterAddress::Force);
        break;
    }
    if (bitField != -1){
        // 对对应位置的寄存器某个位赋值
        setBits(address, bitField, value);
    }else{
        // 直接对对应位置的寄存器赋值,pos,speed,force
        dataRegister[address] = static_cast<int>(value);
    }
    modbus_write.push_back(slaveId);
    modbus_write.push_back(writeRegisterCode);
    modbus_write.push_back((writeRegistersStart >> 8) & 0xFF);
    modbus_write.push_back(writeRegistersStart & 0xFF);
    modbus_write.push_back((registerNumber >> 8) & 0xFF);
    modbus_write.push_back(registerNumber & 0xFF);
    modbus_write.push_back(registerNumber * 2);
    // 开始写数据
    for (uint8_t i = 0; i < sizeof(dataRegister) / sizeof(dataRegister[0]); ++i)
    {
        modbus_write.push_back(dataRegister[i]);
    }
    // 写入CRC校验值
    crcValue = checksum(modbus_write);
    modbus_write.push_back((crcValue >> 8) & 0xFF);
    modbus_write.push_back(crcValue & 0xFF);

    // 发送数据
    std::stringstream Modbus_wtite;
    for (std::vector<uint8_t>::size_type i = 0; i < modbus_write.size(); i++){
        Modbus_wtite << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(modbus_write[i]) << " ";
    }
    RCLCPP_INFO(get_node()->get_logger(), "Modbus wtite: %s", Modbus_wtite.str().c_str());
    gripper_ser.write(modbus_write);

    rclcpp::WallRate loop_rate(10.0);
    loop_rate.sleep();   
    size_t count = gripper_ser.available(); 
    if (count != 0){
        modbus_read.clear();
        count = gripper_ser.read(modbus_read, count);
        std::stringstream Modbus_read;
        for (std::vector<uint8_t>::size_type i = 0; i < modbus_read.size(); i++){
            Modbus_read << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(modbus_read[i]) << " ";
        }
        RCLCPP_INFO(get_node()->get_logger(), "Modbus read: %s", Modbus_read.str().c_str());
    }

}

void HyyExternalController::sendQuery(QueryTopic property)
{
    modbus_write.clear();
    switch (property)
    {
    case QueryTopic::Activation:
        registerNumber = 1;
        break;
    case QueryTopic::Status:
        registerNumber = 3;
        break;
    }
    modbus_write.push_back(slaveId);
    modbus_write.push_back(readRegisterCode);
    modbus_write.push_back((readRegistersStart >> 8) & 0xFF);
    modbus_write.push_back(readRegistersStart & 0xFF);
    modbus_write.push_back(0);
    modbus_write.push_back(registerNumber);
    // 写入CRC校验值
    crcValue = checksum(modbus_write);
    modbus_write.push_back((crcValue >> 8) & 0xFF);
    modbus_write.push_back(crcValue & 0xFF);
    std::stringstream ss;
    for (std::vector<uint8_t>::size_type i = 0; i < modbus_write.size(); i++){
        ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(modbus_write[i]) << " ";
    }
    RCLCPP_INFO(get_node()->get_logger(), "Modbus wtite: %s", ss.str().c_str());
    size_t res_write = gripper_ser.write(modbus_write);

    rclcpp::WallRate loop_rate(10.0);
    loop_rate.sleep();   
    size_t count = gripper_ser.available(); 
    if (count != 0){
        modbus_read.clear();
        count = gripper_ser.read(modbus_read, count);
        std::stringstream Modbus_read;
        for (std::vector<uint8_t>::size_type i = 0; i < modbus_read.size(); i++){
            Modbus_read << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(modbus_read[i]) << " ";
        }
        RCLCPP_INFO(get_node()->get_logger(), "Modbus read: %s", Modbus_read.str().c_str());
    }

}

void HyyExternalController::setBits(uint8_t address, int pos, uint8_t value)
{
    dataRegister[address] = dataRegister[address] | (value << pos);
}

unsigned short HyyExternalController::checksum(const std::vector<uint8_t> &message)
{
    unsigned char lowByte = 0xFF;
    unsigned char highByte = 0xFF;
    unsigned int index;

    for (unsigned char ch : message)
    {
        index = highByte ^ ch;
        highByte = lowByte ^ highByteLookup[index];
        lowByte = lowByteLookup[index];
    }

    return static_cast<unsigned short>(highByte << 8 | lowByte);
}

double HyyExternalController::store_gripper_state(int degree){
    double degree_ = (static_cast<double>(degree)) * 0.024 / 255;
    return degree_;
}

std::vector<double> HyyExternalController::store_hand_state(std::vector<int> degree){
    std::vector<double> lim = {1.7, 1.7, 1.7, 1.7, 0.6, 1.3};
	std::vector<double> degree_(degree.size());
    for (int i = 0; i < degree.size(); i++)
    {
        degree_[i] = (1000 - static_cast<double>(degree[i])) * lim[i] / 1000;
    }
    return degree_;
}
}

PLUGINLIB_EXPORT_CLASS(hyy_external_controller::HyyExternalController, controller_interface::ControllerInterface)

