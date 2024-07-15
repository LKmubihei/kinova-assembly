#include "hyy_controller/hyy_controller.hpp"

namespace hyy_controller
{

HyyController::HyyController():
component_index_(0),
dof_(0),
component_name_(nullptr),
start_controller(false),
block_flag(true),
if_additionaxis(false)
{

	// intialize default_tool_frame
	default_tool_frame = new double[6];
	for (int i = 0; i < 6; i++){
		default_tool_frame[i] = 0.0;
	}

	// intialize default_userframe
	default_userframe = new double[6];
	for (int i = 0; i < 6; i++){
		default_userframe[i] = 0.0;
	}
	// intialize default_workframe
	default_workframe = new double[6];
	for (int i = 0; i < 6; i++){
		default_workframe[i] = 0.0;
	}
	// intialize default_addframe
	default_addframe = new double[6];
	for (int i = 0; i < 6; i++){
		default_addframe[i] = 0.0;
	}

	// intialize default_payload
	default_payload.m = 0.0;
	for (int i = 0; i < 3; i++){
		default_payload.cm[i] = 0.0;
		for (int j = 0; j < 3; j++){
			default_payload.I[i][j] = 0.0;
			default_payload.I2[i][j] = 0.0;
		}
	}

}

HyyController::~HyyController()
{
    delete[] joint_speed;
    delete[] default_userframe;
    delete[] default_workframe;
    delete[] default_tool_frame;
    delete[] default_addframe;
}
  
controller_interface::CallbackReturn HyyController::on_init(){

    try{

        param_listener_ = std::make_shared<ParamListener>(get_node());
        if (!param_listener_)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
            return controller_interface::CallbackReturn::ERROR;
        }
        params_ = param_listener_->get_params();

        component_index_ = params_.component_index;

		if_additionaxis = params_.if_additionaxis;

		if (params_.joints.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
            return controller_interface::CallbackReturn::ERROR;
        }else{
			for (const auto & joint : params_.joints){
				joints_.push_back(joint);
			}
        }

        if (params_.state_interfaces.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "'state_interface' parameter was empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (params_.command_interfaces.empty()){
            RCLCPP_ERROR(get_node()->get_logger(), "'command_interface' parameter was empty");
            return controller_interface::CallbackReturn::ERROR;
        }

    }catch (const std::exception & e){
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

	if(!if_additionaxis){
		component_name_ = HYYRobotBase::get_name_robot_device(HYYRobotBase::get_deviceName(0, NULL), component_index_);
	}else{
		component_name_ = HYYRobotBase::get_name_additionaxis_device(HYYRobotBase::get_deviceName(0, NULL), component_index_);
	}
	if (nullptr == component_name_){
		RCLCPP_ERROR(get_node()->get_logger(), "Can't get component(robot) name");
		return controller_interface::CallbackReturn::ERROR;
	}

    dof_ = HYYRobotBase::get_group_dof(component_name_);
    if (dof_ != static_cast<int>(joints_.size()))
    {
		RCLCPP_ERROR(get_node()->get_logger(), "Robot device dof(%d) != joint num(%ld).", dof_, joints_.size());
		return controller_interface::CallbackReturn::ERROR;
	}

    joint_speed = new double[dof_];
    for (int i = 0; i < dof_; i++){
        joint_speed[i] = 0.1;
    }

	return controller_interface::CallbackReturn::SUCCESS;

}

controller_interface::InterfaceConfiguration HyyController::command_interface_configuration() const{

    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & joint : params_.joints){
        for (const auto & interface_name : params_.command_interfaces){
             command_interfaces_config.names.push_back(joint + "/" + interface_name);
        }
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration HyyController::state_interface_configuration() const{

    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &joint : params_.joints){
        for (const auto & interface_name : params_.state_interfaces){
            state_interfaces_config.names.push_back(joint + "/" + interface_name);
        }
    }

    return state_interfaces_config;
}

controller_interface::CallbackReturn HyyController::on_configure(const rclcpp_lifecycle::State & previous_state){

	auto move_callback = std::bind(&HyyController::robotmove_command_callback, this, std::placeholders::_1, std::placeholders::_2);
	hyyMoveSrv = get_node()->create_service<hyyMoveMsg>("~/hyyRobotMoveSrv", move_callback);
	if(!hyyMoveSrv){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service hyyRobotMoveSrv.");
    }else{
		RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/hyyRobotMoveSrv is now ready.");
	}

	auto grip_callback = std::bind(&HyyController::robotgrip_command_callback, this, std::placeholders::_1, std::placeholders::_2);
    hyyGripSrv = get_node()->create_service<hyyGripMsg>("~/hyyRobotGripSrv", grip_callback);
    if(!hyyGripSrv){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service hyyGripSrv.");
    }else{
		RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/hyyRobotGripSrv is now ready.");
    }

	auto movedata_callback = std::bind(&HyyController::robotmovedata_command_callback, this, std::placeholders::_1, std::placeholders::_2);
    hyyMoveDataSrv = get_node()->create_service<hyyMoveDataMsg>("~/hyyRobotMoveDataSrv", movedata_callback);
    if(!hyyMoveDataSrv){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service hyyMoveDataSrv.");
    }else{
		RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/hyyRobotMoveDataSrv is now ready.");
    }

	auto io_callback = std::bind(&HyyController::robotio_command_callback, this, std::placeholders::_1, std::placeholders::_2);
    hyyIoSrv = get_node()->create_service<hyyIoMsg>("~/hyyRobotIoSrv", io_callback);
    if(!hyyIoSrv){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service hyyIoSrv.");
    }else{
		RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/hyyRobotIoSrv is now ready.");
    }

	auto generalcontrol_callback = std::bind(&HyyController::robotgeneralcontrol_command_callback, this, std::placeholders::_1, std::placeholders::_2);
    hyyGeneralControlSrv = get_node()->create_service<hyyGeneralControlMsg>("~/hyyRobotGeneralControlSrv", generalcontrol_callback);
    if(!hyyGeneralControlSrv){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to create service hyyGeneralControlSrv.");
    }else{
		RCLCPP_INFO_STREAM(get_node()->get_logger(), "Service " << get_node()->get_name() << "/hyyRobotGeneralControlSrv is now ready.");
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HyyController::on_activate(const rclcpp_lifecycle::State & previous_state){

	if (0 != HYYRobotBase::OpenCtrlInput(component_name_))
	{
        RCLCPP_ERROR(get_node()->get_logger(), "OpenCtrlInput failed.");
        return controller_interface::CallbackReturn::ERROR;
    }

    HYYRobotBase::setMoveThread(1);
    start_controller = true;

    return controller_interface::CallbackReturn::SUCCESS;

}

controller_interface::CallbackReturn HyyController::on_deactivate(const rclcpp_lifecycle::State & previous_state){

    start_controller = false;
    if (0 != HYYRobotBase::CloseCtrlInput(component_name_))
    {
		RCLCPP_ERROR(get_node()->get_logger(), "CloseCtrlInput failed.");
		return controller_interface::CallbackReturn::ERROR;
	}
    HYYRobotBase::setMoveThread(0);

    return controller_interface::CallbackReturn::SUCCESS;

}

controller_interface::return_type HyyController::update(const rclcpp::Time & time, const rclcpp::Duration & period){

	if (start_controller)
	{
		double position[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		HYYRobotBase::GetGroupCtrlTarget(component_name_, position, NULL2);
		for (auto index = 0ul; index < command_interfaces_.size(); ++index){
			command_interfaces_[index].set_value(position[index]);
		}
	}
	return controller_interface::return_type::OK;
}

void HyyController::robotmove_command_callback(const std::shared_ptr<hyy_message::srv::Robotmove::Request> req,
                                    std::shared_ptr<hyy_message::srv::Robotmove::Response> res){

    using namespace HYYRobotBase;
    using namespace std;
	if (start_controller){
        if (!req->type.empty()){
			
            string _moveA = "moveA";
            string _moveJ = "moveJ";
            string _moveL = "moveL";
            string _moveC = "moveC";
            string _state = "move_state";

            robjoint *rjoint = NULL;
            robpose *rpose = NULL;
            robpose *rpose_mid = NULL;
            speed *rspeed = NULL;
            zone *rzone = NULL;
            tool *rtool = NULL;
            wobj *rwobj = NULL;

            if (_state != req->type){
				RCLCPP_INFO_STREAM(get_node()->get_logger(), "Recieve new target......");
				if (!req->velocity.empty())
                { 
                    double v = velocity_data_type(req->velocity.c_str());
                    if (v < 0){
                        if (0 != getspeed(req->velocity.c_str(), &rspeed_)){
                            RCLCPP_WARN(get_node()->get_logger(), "Get speed failed, use default.");
                            init_speed(&rspeed_, joint_speed, dof_, 1, 100, 0.1, 1);
                            rspeed = &rspeed_;
                        }else{
                            rspeed = &rspeed_;
							RCLCPP_INFO_STREAM(get_node()->get_logger(), "Use speed " << req->velocity);
                        }
                    }else{
						RCLCPP_INFO_STREAM(get_node()->get_logger(), "Use speed percentage " << v * 100 << "%");
						for (int i = 0; i < dof_; i++){
							joint_speed[i] = v;
						}
						init_speed(&rspeed_, joint_speed, dof_, 0, v, v, 0);
						rspeed = &rspeed_;
					}
				}else{
                    RCLCPP_WARN(get_node()->get_logger(), "Speed data was empty, use default.");
                    init_speed(&rspeed_, joint_speed, dof_, 1, 100, 0.1, 1);
                    rspeed = &rspeed_;
                }

				if (!req->zone.empty())
				{
                    if (0 != getzone(req->zone.c_str(), &rzone_)){
                        RCLCPP_WARN(get_node()->get_logger(), "Get zone failed, use default.");
                        init_zone(&rzone_, 0, 0);
                        rzone = &rzone_;
                    }else{
						rzone = &rzone_;
						RCLCPP_INFO_STREAM(get_node()->get_logger(), "Use zone " << req->zone);
					}
                }else{
					RCLCPP_WARN(get_node()->get_logger(), "Zone data was empty, use default.");
					init_zone(&rzone_, 0, 0);
					rzone = &rzone_;
				}

				if (!req->tool.empty())
				{
                    if (0 != gettool(req->tool.c_str(), &rtool_)){
                        RCLCPP_WARN(get_node()->get_logger(), "Get tool failed, use default.");
						init_tool(&rtool_, default_tool_frame, 1, &default_payload);
						rtool = &rtool_;
					}else{
						RCLCPP_INFO_STREAM(get_node()->get_logger(), "Use tool " << req->tool);
						rtool = &rtool_;
					}
				}else{
					RCLCPP_WARN(get_node()->get_logger(), "Tool data was empty, use default.");
					init_tool(&rtool_, default_tool_frame, 1, &default_payload);
					rtool = &rtool_;
				}


				if (!req->wobj.empty())
				{
					if (0 != getwobj(req->wobj.c_str(), &rwobj_)){
						RCLCPP_WARN(get_node()->get_logger(), "Get wobj failed, use default.");
						init_wobj(&rwobj_, default_userframe, default_workframe, default_addframe, 0, 0, 0);
						rwobj = &rwobj_;
					}else{
						RCLCPP_INFO_STREAM(get_node()->get_logger(), "Use wobj " << req->wobj);
						rwobj = &rwobj_;
					}
				}else{
					RCLCPP_WARN(get_node()->get_logger(), "Wobj data was empty, use default.");
					init_wobj(&rwobj_, default_userframe, default_workframe, default_addframe, 0, 0, 0);
					rwobj = &rwobj_;
				}

			}
			if (_moveA == req->type)
			{
				if (!req->target.empty()){
					if (dof_ != static_cast<int>(req->target.size())){
						RCLCPP_ERROR_STREAM(get_node()->get_logger(), "MoveA: target dof error: " << req->target.size());
						res->result = ERR_MOVETERGET_DOF;
						return;
					}else{
						init_robjoint(&rjoint_, req->target.data(), dof_);
						rjoint = &rjoint_;
					}
				}else{
					if (!req->targetstr.empty()){
						if (0 == getrobjoint(req->targetstr.c_str(), &rjoint_)){
							rjoint = &rjoint_;
						}else{
							RCLCPP_ERROR(get_node()->get_logger(), "MoveA: get robjoint from targetstr error.");
							res->result = ERR_GETMOVETARGET;
							return;
						}
					}else{
						RCLCPP_ERROR(get_node()->get_logger(), "MoveA: targetstr isn't exist.");
						res->result = ERR_MOVETERGET_EMPTY;
						return;
					}
					
				}
				if (if_additionaxis){
					res->result = MultiMoveAdd(rjoint, rspeed, rzone, rtool, rwobj, component_index_);
				}else{
					res->result = MultiMoveA(rjoint, rspeed, rzone, rtool, rwobj, component_index_);
				}

				if (0 != res->result){
					RCLCPP_INFO_STREAM(get_node()->get_logger(), "result: " << res->result);
					RCLCPP_ERROR(get_node()->get_logger(), "MoveA: perform moveA error.");
					return;
				}

			}else if (_moveJ == req->type){
				if (!req->target.empty())
				{
					if (6 != req->target.size()){
						RCLCPP_ERROR(get_node()->get_logger(), "MoveJ: target dimensions error.");
						res->result = ERR_MOVETERGET_DOF;
						return;
					}else{
						init_robpose(&rpose_, &(req->target[0]), &(req->target[3]));
						rpose = &rpose_;
					}
				}else{
					if (!req->targetstr.empty()){
						if (0 == getrobpose(req->targetstr.c_str(), &rpose_)){
							rpose = &rpose_;
						}else{
							RCLCPP_ERROR(get_node()->get_logger(), "MoveJ: get robpose from targetstr error.");
							res->result = ERR_GETMOVETARGET;
							return;
						}
					}else{
						RCLCPP_ERROR(get_node()->get_logger(), "MoveJ: targetstr isn't exist.");
						res->result = ERR_MOVETERGET_EMPTY;
						return;
					}
				}
				res->result = MultiMoveJ(rpose, rspeed, rzone, rtool, rwobj, component_index_);
				if (0 != res->result){
					RCLCPP_INFO_STREAM(get_node()->get_logger(), "result: " << res->result);
					RCLCPP_ERROR(get_node()->get_logger(), "MoveJ: perform moveJ error.");
					return;
				}
			
			}else if (_moveL == req->type)
			{
				res->result = 0;
				if (!req->target.empty()){
					if (6 != req->target.size()){
						RCLCPP_ERROR(get_node()->get_logger(), "MoveL: target dimensions error.");
						res->result = ERR_MOVETERGET_DOF;
						return;
					}else{
						init_robpose(&rpose_, &(req->target[0]), &(req->target[3]));
						rpose = &rpose_;
					}
				}else{
					if (!req->targetstr.empty())
					{
						if (0 == getrobpose(req->targetstr.c_str(), &rpose_)){
							rpose = &rpose_;
						}else{
							RCLCPP_ERROR(get_node()->get_logger(), "MoveL: get robpose from targetstr error.");
							res->result = ERR_GETMOVETARGET;
							return;
						}
					}else{
						RCLCPP_ERROR(get_node()->get_logger(), "MoveL: targetstr isn't exist.");
						res->result = ERR_MOVETERGET_EMPTY;
						return;
					}
				}
				res->result = MultiMoveL(rpose, rspeed, rzone, rtool, rwobj, component_index_);
				if (0 != res->result)
				{
					RCLCPP_INFO_STREAM(get_node()->get_logger(), "result: " << res->result);
					RCLCPP_ERROR(get_node()->get_logger(), "MoveL: perform moveL error.");
					return;
				}
			}else if (_moveC == req->type){
				if (!req->target.empty()){
					if (12 != req->target.size()){
						RCLCPP_ERROR(get_node()->get_logger(), "MoveC: target dimensions error.");
						res->result = ERR_MOVETERGET_DOF;
						return;
					}else{
						init_robpose(&rpose_, &(req->target[0]), &(req->target[3]));
						rpose = &rpose_;
						init_robpose(&rpose_mid_, &(req->target[6]), &(req->target[9]));
						rpose_mid = &rpose_mid_;
					}
				}else{
					if (!req->targetstr.empty())
					{
						int n = req->targetstr.find(",");
						std::string _target = req->targetstr.substr(0, n);
						std::string _target_mid = req->targetstr.substr(n + 1);
						if (_target.empty() || _target_mid.empty())
						{
							RCLCPP_ERROR(get_node()->get_logger(), "MoveC: get target and target_mid from targetstr error.");
							res->result = ERR_DATASPLIT;
							return;
						}
						if (0 == getrobpose(_target.c_str(), &rpose_)){
							rpose = &rpose_;
						}else{
							RCLCPP_ERROR(get_node()->get_logger(), "MoveC: get robpose from target error.");
							res->result = ERR_GETMOVETARGET;
							return;
						}
						if (0 == getrobpose(_target_mid.c_str(), &rpose_mid_)){
							rpose_mid = &rpose_mid_;
						}else{
							RCLCPP_ERROR(get_node()->get_logger(), "MoveC:get robpose_mid from target_mid error.");
							res->result = ERR_GETMOVETARGET;
							return;
						}
					}else{
						RCLCPP_ERROR(get_node()->get_logger(), "MoveC: targetstr isn't exist.");
						res->result = ERR_MOVETERGET_EMPTY;
						return;
					}
				}
				res->result = MultiMoveC(rpose, rpose_mid, rspeed, rzone, rtool, rwobj, component_index_);
				if (0 != res->result)
				{
					RCLCPP_INFO_STREAM(get_node()->get_logger(), "result: " << res->result);
					RCLCPP_ERROR(get_node()->get_logger(), "MoveC: perform moveC error.");
					return;
				}

			}else if (_state == req->type){
				if(if_additionaxis){
					res->result = get_addition_move_state(component_index_);
				}else{
					res->result = get_robot_move_state(component_index_);
				}
				return;
			}else{
				RCLCPP_ERROR(get_node()->get_logger(), "Move: robot command isn't exist.");
				res->result = ERR_TYPE;
				return;
			}
		}else{
			RCLCPP_ERROR(get_node()->get_logger(), "Move: move type is empty, please check.");
			res->result = ERR_TYPE_EMPTY;
			return;
		}
	}else{
		RCLCPP_ERROR(get_node()->get_logger(), "Move: please start controller.");
		res->result = ERR_CONTROLLER_INACTIVE;
		return;
	}

}

void HyyController::robotio_command_callback(const std::shared_ptr<hyy_message::srv::Robotio::Request> req,
                                std::shared_ptr<hyy_message::srv::Robotio::Response> res){

	if (start_controller)
	{
		std::string DI = "DI";
		std::string DO = "DO";
		if (DI == req->type){
			res->result = HYYRobotBase::GetDi(req->index);
			RCLCPP_INFO_STREAM(get_node()->get_logger(), "type:" << req->type << "; index:" << req->index << "; value:" << res->result);
		}else if(DO == req->type){
			int value = req->value ? 1 : 0;
			HYYRobotBase::SetDo(req->index, value);
			res->result = 1;
			RCLCPP_INFO_STREAM(get_node()->get_logger(), "type:" << req->type << "; index:" << req->index << "; value:" << value);
		}else{
			res->result = ERR_TYPE;
			return;
		}
	}else{
		RCLCPP_ERROR(get_node()->get_logger(), "Io : please start controller");
		res->result = ERR_CONTROLLER_INACTIVE;
		return;
	}

}

void HyyController::robotmovedata_command_callback(const std::shared_ptr<hyy_message::srv::Robotmovedata::Request> req,
                                std::shared_ptr<hyy_message::srv::Robotmovedata::Response> res){

	using namespace std;
	using namespace HYYRobotBase;
	if (start_controller){
		if (!req->type.empty()){
      		RCLCPP_INFO(get_node()->get_logger(), "Start processing robotdata");
			string _cartesian = "cartesian";
			string _joint = "joint";
			string _cur_cartesian = "cartesian_current";
			string _cur_joint = "joint_current";
			if (_cartesian == req->type)
			{
				robpose rpose;
				if (0 != getrobpose(req->datastr.c_str(), &rpose)){
					RCLCPP_ERROR(get_node()->get_logger(), "Movedata: get robpose failed");
					res->result.clear();
					return;
				}else{
					res->result.clear();
					res->result.push_back(rpose.xyz[0]);
					res->result.push_back(rpose.xyz[1]);
					res->result.push_back(rpose.xyz[2]);
					res->result.push_back(rpose.kps[0]);
					res->result.push_back(rpose.kps[1]);
					res->result.push_back(rpose.kps[2]);
					RCLCPP_INFO_STREAM(get_node()->get_logger(), "Movedata: send cartesian pose: " << req->datastr);
				}
			}else if (_joint == req->type){
				robjoint rjoint;
				if (0 != getrobjoint(req->datastr.c_str(), &rjoint)){
					res->result.clear();
					RCLCPP_ERROR(get_node()->get_logger(), "Movedata: get robjoint failed");
					return;
				}else{
					if (dof_ != rjoint.dof){
						res->result.clear();
						RCLCPP_ERROR(get_node()->get_logger(), "Movedata: data dimensions error\n");
						return;
					}
					res->result.clear();
					for (int i = 0; i < rjoint.dof; i++){
						res->result.push_back(rjoint.angle[i]);
					}
					RCLCPP_INFO_STREAM(get_node()->get_logger(), "Movedata: send joint position: " << req->datastr);
				}
			}else if (_cur_cartesian == req->type){
				tool *rtool = NULL;
				wobj *rwobj = NULL;
				if (!req->tool.empty()){
					if (0 != gettool(req->tool.c_str(), &rtool_)){
						RCLCPP_WARN(get_node()->get_logger(), "Movedata: get tool failed, use default.");
						init_tool(&rtool_, default_tool_frame, 1, &default_payload);
						rtool = &rtool_;				
					}else{
						rtool = &rtool_;
					}
				}
				if (!req->wobj.empty())
				{
					if (0 != getwobj(req->wobj.c_str(), &rwobj_)){
						RCLCPP_WARN(get_node()->get_logger(), "Movedata: get wobj failed, use default.");
						init_wobj(&rwobj_, default_userframe, default_workframe, default_addframe, 0, 0, 0);
						rwobj = &rwobj_;
					}else{
						rwobj = &rwobj_;
					}
				}
				double pospose[6];
				if (0 != robot_getCartesian(rtool, rwobj, pospose, component_index_)){
					res->result.clear();
					RCLCPP_ERROR(get_node()->get_logger(), "Movedata: can't get current cartesian pos.");
					return;
				}
				res->result.clear();
				res->result.push_back(pospose[0]);
				res->result.push_back(pospose[1]);
				res->result.push_back(pospose[2]);
				res->result.push_back(pospose[3]);
				res->result.push_back(pospose[4]);
				res->result.push_back(pospose[5]);
				RCLCPP_INFO(get_node()->get_logger(), "Movedata: send current cartesian point");
			}else if (_cur_joint == req->type){
				double joint[10];
				memset(joint, 0, sizeof(joint));
				if(if_additionaxis){
					GetCurrentAdditionJoint(joint, component_index_);
				}else{
					robot_getJoint(joint, component_index_);
				}
				res->result.clear();
				for (int i = 0; i < dof_; i++){
					res->result.push_back(joint[i]);
				}
				RCLCPP_INFO(get_node()->get_logger(), "Movedata: send current joint point");
			}else{
				res->result.clear();
				RCLCPP_ERROR(get_node()->get_logger(), "Movedata: data type isn't exist.");
				return;
			}
		}else{
			res->result.clear();
			RCLCPP_ERROR(get_node()->get_logger(), "Movedata: movedata type is empty, please check.");
			return;
		}
	}else{
		res->result.clear();
		RCLCPP_ERROR(get_node()->get_logger(), "Movedata: please start controller");
		return;
	}

}

void HyyController::robotgrip_command_callback(const std::shared_ptr<hyy_message::srv::Robotgrip::Request> req,
                                std::shared_ptr<hyy_message::srv::Robotgrip::Response> res){

	using namespace HYYRobotBase;
	if (start_controller)
	{
		int ret = 0;
		if (req->value > 1) {
			// Create Grip
			ret = CreateGrip2(req->type.c_str(), req->type.c_str()); 
			RCLCPP_INFO(get_node()->get_logger(),"Grip: create grip");
		}else if (req->value < 0) {
			// Destroy Grip
			ret = DestroyGrip(req->type.c_str());
			RCLCPP_INFO(get_node()->get_logger(),"Grip: destroy grip");
		}else {
			// control Grip
			ret = ControlGrip(req->type.c_str(), req->value);
			RCLCPP_INFO(get_node()->get_logger(),"Grip: control grip");
		}
		res->result = ret;
		if (ret != 0){
			RCLCPP_ERROR(get_node()->get_logger(), "Grip: execute grip failed");
			return;
		}
	}else{
		RCLCPP_ERROR(get_node()->get_logger(), "Grip: please start controller");
		res->result = ERR_CONTROLLER_INACTIVE;
		return;
	}
}

void HyyController::robotgeneralcontrol_command_callback(const std::shared_ptr<hyyGeneralControlMsg::Request> req,
								std::shared_ptr<hyyGeneralControlMsg::Response> res){

	using namespace HYYRobotBase;
	using namespace std;
	if (start_controller)
	{
		if (!req->type.empty()){
			string _deviceStop = "deviceStop";
			string _robotStop = "robotStop";
			string _addaxisStop = "addaxisStop";
			if(req->type == _deviceStop){
				DeviceStopRun();
				RCLCPP_INFO(get_node()->get_logger(), "robotcontrol: detect DeviceStopRun.");
				res->result = 0;
				return;
			}else if(req->type == _robotStop){
				for (int i = 0; i < req->robotindex.size(); i++){
					RobotStopRun(req->robotindex.at(i));
				}
				RCLCPP_INFO(get_node()->get_logger(), "robotcontrol: detect RobotStopRun.");
				res->result = 0;
				return;
			}else if(req->type == _addaxisStop){
				for (int i = 0; i < req->addaxisindex.size(); i++){
					AdditionStopRun(req->addaxisindex.at(i));
				}
				RCLCPP_INFO(get_node()->get_logger(), "robotcontrol: detect AdditionStopRun.");
				res->result = 0;
				return;
			}else{
				RCLCPP_ERROR(get_node()->get_logger(), "robotcontrol: control type isn't exist.");
				res->result = ERR_TYPE;
				return;
			}
		}else{
			RCLCPP_ERROR(get_node()->get_logger(), "robotcontrol: control type is empty, please check.");
			res->result = ERR_TYPE_EMPTY;
			return;
		}
	}

}

double HyyController::velocity_data_type(const std::string& velocity)
{
	if (velocity.find("%") == velocity.length() - 1)
	{
		double ret = std::stod(velocity.substr(0, velocity.length() - 1));
		if (ret < 0.0){
			RCLCPP_WARN(get_node()->get_logger(), "Speed percentage data is out of range(0-100), use value 1%%");
			return 0.01;
		}else if (ret > 100){
			RCLCPP_WARN(get_node()->get_logger(), "Speed percentage data is out of range(0-100), use value 100%%");
			return 1;
		}else{
			// RCLCPP_INFO_STREAM(get_node()->get_logger(), "Use speed percentage " << ret << "%");
			return ret * 0.01;
		}
	}else{
		return ERR_PERSPEEDFORMAT;
	}
}

}  // namespace hyy_controller

PLUGINLIB_EXPORT_CLASS(hyy_controller::HyyController, controller_interface::ControllerInterface)
