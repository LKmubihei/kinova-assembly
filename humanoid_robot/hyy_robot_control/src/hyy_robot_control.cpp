#include "hyy_robot_control/hyy_robot_control.h"

std::atomic<bool> stop(false);

namespace hyy_robot_control{

HyyRobotControl::HyyRobotControl(std::shared_ptr<rclcpp::Node> node) :
init_flag(false),
block_flag(true)
{
    node_ = node;
	moveReq = std::make_shared<hyyMoveMsg::Request>();
	ioReq = std::make_shared<hyyIoMsg::Request>();
	moveDataReq = std::make_shared<hyyMoveDataMsg::Request>();
	gripReq = std::make_shared<hyyGripMsg::Request>();

    SetangleReq = std::make_shared<hyySetangleMsg::Request>();
    SetposReq = std::make_shared<hyySetposMsg::Request>();
    SetspeedReq = std::make_shared<hyySetspeedMsg::Request>();
    SetforceReq = std::make_shared<hyySetforceMsg::Request>();
    GetangleactReq = std::make_shared<hyyGetangleactMsg::Request>();
    GetanglesetReq = std::make_shared<hyyGetanglesetMsg::Request>();
    GetposactReq = std::make_shared<hyyGetposactMsg::Request>();
    GetpossetReq = std::make_shared<hyyGetpossetMsg::Request>();
    GetspeedsetReq = std::make_shared<hyyGetspeedsetMsg::Request>();
    GetforceactReq = std::make_shared<hyyGetforceactMsg::Request>();
    GetforcesetReq = std::make_shared<hyyGetforcesetMsg::Request>();
    GetcurrentactReq = std::make_shared<hyyGetcurrentactMsg::Request>();
    GeterrorReq = std::make_shared<hyyGeterrorMsg::Request>();
    GettempReq = std::make_shared<hyyGettempMsg::Request>();
	empty.resize(6);

    SetGripPosReq = std::make_shared<hyyGripMsg::Request>();
    SetGripSpeedReq = std::make_shared<hyyGripMsg::Request>();
    SetGripForceReq = std::make_shared<hyyGripMsg::Request>();
    GripMoveReq = std::make_shared<hyyGripMsg::Request>();
    ActivateGripReq = std::make_shared<hyyGripMsg::Request>();
    DeactivateGripReq = std::make_shared<hyyGripMsg::Request>();
    GetGripStatusReq = std::make_shared<hyyGripMsg::Request>();
    GetGripPosReq = std::make_shared<hyyGripMsg::Request>();
}

HyyRobotControl::HyyRobotControl(){}

bool HyyRobotControl::init(std::string controller_name, int type)
{

    if (type == 1) {

		SetangleSrvName_ = "/" + controller_name + "/Setangle";
		SetposSrvName_ = "/" + controller_name + "/Setpos";
		SetspeedSrvName_ = "/" + controller_name + "/Setspeed";
		SetforceSrvName_ = "/" + controller_name + "/Setforce";
		GetangleactSrvName_ = "/" + controller_name + "/Getangleact";
		GetanglesetSrvName_ = "/" + controller_name + "/Getangleset";
		GetposactSrvName_ = "/" + controller_name + "/Getposact";
		GetpossetSrvName_ = "/" + controller_name + "/Getposset";
		GetspeedsetSrvName_ = "/" + controller_name + "/Getspeedset";
		GetforceactSrvName_ = "/" + controller_name + "/Getforceact";
		GetforcesetSrvName_ = "/" + controller_name + "/Getforceset";
		GetcurrentactSrvName_ = "/" + controller_name + "/Getcurrentact";
		GeterrorSrvName_ = "/" + controller_name + "/Geterror";
		GettempSrvName_ = "/" + controller_name + "/Gettemp";

		hyySetangleClient = node_->create_client<hyySetangleMsg>(SetangleSrvName_);
		hyySetposClient = node_->create_client<hyySetposMsg>(SetposSrvName_);
		hyySetspeedClient = node_->create_client<hyySetspeedMsg>(SetspeedSrvName_);
		hyySetforceClient = node_->create_client<hyySetforceMsg>(SetforceSrvName_);
		hyyGetangleactClient = node_->create_client<hyyGetangleactMsg>(GetangleactSrvName_);
		hyyGetanglesetClient = node_->create_client<hyyGetanglesetMsg>(GetanglesetSrvName_);
		hyyGetposactClient = node_->create_client<hyyGetposactMsg>(GetposactSrvName_);
		hyyGetpossetClient = node_->create_client<hyyGetpossetMsg>(GetpossetSrvName_);
		hyyGetspeedsetClient = node_->create_client<hyyGetspeedsetMsg>(GetspeedsetSrvName_);
		hyyGetforceactClient = node_->create_client<hyyGetforceactMsg>(GetforceactSrvName_);
		hyyGetforcesetClient = node_->create_client<hyyGetforcesetMsg>(GetforcesetSrvName_);
		hyyGetcurrentactClient = node_->create_client<hyyGetcurrentactMsg>(GetcurrentactSrvName_);
		hyyGeterrorClient = node_->create_client<hyyGeterrorMsg>(GeterrorSrvName_);
		hyyGettempClient = node_->create_client<hyyGettempMsg>(GettempSrvName_);
		
		SetGripPosSrvName_ = "/" + controller_name + "/ControlGripper";
		SetGripSpeedSrvName_ = "/" + controller_name + "/ControlGripper";
		SetGripForceSrvName_ = "/" + controller_name + "/ControlGripper";
		GripMoveSrvName_ = "/" + controller_name + "/ControlGripper";
		ActivateGripSrvName_ = "/" + controller_name + "/ControlGripper";
		DeactivateGripSrvName_ = "/" + controller_name + "/ControlGripper";
		GetGripStatusSrvName_ = "/" + controller_name + "/ControlGripper";
		GetGripPosSrvName_ = "/" + controller_name + "/ControlGripper";

		hyySetGripPosClient = node_->create_client<hyyGripMsg>(SetGripPosSrvName_);
		hyySetGripSpeedClient = node_->create_client<hyyGripMsg>(SetGripSpeedSrvName_);
		hyySetGripForceClient = node_->create_client<hyyGripMsg>(SetGripForceSrvName_);
		hyyGripMoveClient = node_->create_client<hyyGripMsg>(GripMoveSrvName_);
		hyyActivateGripClient = node_->create_client<hyyGripMsg>(ActivateGripSrvName_);
		hyyDeactivateGripClient = node_->create_client<hyyGripMsg>(DeactivateGripSrvName_);
		hyyGetGripStatusClient = node_->create_client<hyyGripMsg>(GetGripStatusSrvName_);
		hyyGetGripPosClient = node_->create_client<hyyGripMsg>(GetGripPosSrvName_);

		std::vector<rclcpp::ClientBase::SharedPtr> clients = {
			hyySetangleClient, hyySetposClient, hyySetspeedClient, hyySetforceClient,
			hyyGetangleactClient, hyyGetanglesetClient, hyyGetposactClient, hyyGetpossetClient,
			hyyGetspeedsetClient, hyyGetforceactClient, hyyGetforcesetClient, hyyGetcurrentactClient,
			hyyGeterrorClient, hyyGettempClient, hyySetGripPosClient ,hyySetGripSpeedClient, 
			hyySetGripForceClient, hyyGripMoveClient, hyyActivateGripClient, hyyDeactivateGripClient,
			hyyGetGripStatusClient, hyyGetGripPosClient
		};

		for (const auto &client : clients){
			if (!client) {
				RCLCPP_ERROR(node_->get_logger(), "Failed to create one of the service clients.");
				return false;
			}
		}

	} else {

		MoveSrvName_ = "/" + controller_name + "/hyyRobotMoveSrv";
		IoSrvName_ = "/" + controller_name + "/hyyRobotIoSrv";
		MoveDataSrvName_ = "/" + controller_name + "/hyyRobotMoveDataSrv";
		GripSrvName_ = "/" + controller_name + "/hyyRobotGripSrv";

		robotMoveClient = node_->create_client<hyyMoveMsg>(MoveSrvName_);
		robotIoClient = node_->create_client<hyyIoMsg>(IoSrvName_);
		robotMoveDataClient = node_->create_client<hyyMoveDataMsg>(MoveDataSrvName_);
		robotGripClient = node_->create_client<hyyGripMsg>(GripSrvName_);

		std::vector<rclcpp::ClientBase::SharedPtr> clients = {robotMoveClient, robotIoClient, robotMoveDataClient, robotGripClient};

		for (const auto &client : clients){
			if (!client) {
				RCLCPP_ERROR(node_->get_logger(), "Failed to create one of the service clients.");
				return false;
			}
		}

	}
	sleep(5);
	RCLCPP_INFO(node_->get_logger(), "All service clients created successfully.");

	init_flag = true;
	return true;
}

int HyyRobotControl::moveA(const std::string& target,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotMoveClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotMoveClient->get_service_name());
		return -1;
	}
	
	moveReq->type = "moveA";
	moveReq->targetstr = target;
	moveReq->target.clear();
	moveReq->velocity = velocity;
	moveReq->zone = zone;
	moveReq->tool = tool;
	moveReq->wobj = wobj;

	auto res = robotMoveClient->async_send_request(moveReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		// RCLCPP_INFO_STREAM(node_->get_logger(), robotMoveClient->get_service_name()<< " response: " << res.get()->result);
		return wait_move_finish(res);
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "moveA: failed to call service " << robotMoveClient->get_service_name());
		return -1;
	}
}

int HyyRobotControl::moveA(std::vector<double>& target,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotMoveClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotMoveClient->get_service_name());
		return -1;
	}
	moveReq->type = "moveA";
	moveReq->target = target;
	moveReq->targetstr.clear();
	moveReq->velocity = velocity;
	moveReq->zone = zone;
	moveReq->tool = tool;
	moveReq->wobj = wobj;

	auto res = robotMoveClient->async_send_request(moveReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		// RCLCPP_INFO_STREAM(node_->get_logger(), robotMoveClient->get_service_name()<< " response: " << res.get()->result);
		return wait_move_finish(res);
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "moveA: failed to call service " << robotMoveClient->get_service_name());
		return -1;
	}
}

int HyyRobotControl::moveJ(const std::string& target,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotMoveClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotMoveClient->get_service_name());
		return -1;
	}
	moveReq->type = "moveJ";
	moveReq->targetstr = target;
	moveReq->target.clear();
	moveReq->velocity = velocity;
	moveReq->zone = zone;
	moveReq->tool = tool;
	moveReq->wobj = wobj;

	auto res = robotMoveClient->async_send_request(moveReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		// RCLCPP_INFO_STREAM(node_->get_logger(), robotMoveClient->get_service_name() << " response: " << res.get()->result);
		return wait_move_finish(res);
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "moveJ: failed to call service " << robotMoveClient->get_service_name());
		return -1;
	}
}

int HyyRobotControl::moveJ(std::vector<double>& target,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotMoveClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotMoveClient->get_service_name());
		return -1;
	}
	moveReq->type = "moveJ";
	moveReq->target = target;
	moveReq->targetstr.clear();
	moveReq->velocity = velocity;
	moveReq->zone = zone;
	moveReq->tool = tool;
	moveReq->wobj = wobj;

	auto res = robotMoveClient->async_send_request(moveReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		// RCLCPP_INFO_STREAM(node_->get_logger(), robotMoveClient->get_service_name() << " response: " << res.get()->result);
		return wait_move_finish(res);
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "moveJ: failed to call service " << robotMoveClient->get_service_name());
		return -1;
	}
}

int HyyRobotControl::moveL(const std::string& target,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotMoveClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotMoveClient->get_service_name());
		return -1;
	}
	moveReq->type = "moveL";
	moveReq->targetstr = target;
	moveReq->target.clear();
	moveReq->velocity = velocity;
	moveReq->zone = zone;
	moveReq->tool = tool;
	moveReq->wobj = wobj;
	auto res = robotMoveClient->async_send_request(moveReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		// RCLCPP_INFO_STREAM(node_->get_logger(), robotMoveClient->get_service_name() << " response: " << res.get()->result);
		return wait_move_finish(res);
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "moveL: failed to call service " << robotMoveClient->get_service_name());
		return -1;
	}
}

int HyyRobotControl::moveL(std::vector<double>& target,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotMoveClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotMoveClient->get_service_name());
		return -1;
	}
	moveReq->type = "moveL";
	moveReq->target = target;
	moveReq->targetstr.clear();
	moveReq->velocity = velocity;
	moveReq->zone = zone;
	moveReq->tool = tool;
	moveReq->wobj = wobj;

	auto res = robotMoveClient->async_send_request(moveReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		// RCLCPP_INFO_STREAM(node_->get_logger(), robotMoveClient->get_service_name() << " response: " << res.get()->result);
		return wait_move_finish(res);
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "moveL: failed to call service " << robotMoveClient->get_service_name());
		return -1;
	}
}

int HyyRobotControl::moveC(const std::string& target,const std::string& target_mid,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotMoveClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotMoveClient->get_service_name());
		return -1;
	}
	moveReq->type = "moveC";
	moveReq->targetstr = target + "," + target_mid;
	moveReq->target.clear();
	moveReq->velocity = velocity;
	moveReq->zone = zone;
	moveReq->tool = tool;
	moveReq->wobj = wobj;

	auto res = robotMoveClient->async_send_request(moveReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		// RCLCPP_INFO_STREAM(node_->get_logger(), robotMoveClient->get_service_name() << " response: " << res.get()->result);
		return wait_move_finish(res);
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "moveC: failed to call service " << robotMoveClient->get_service_name());
		return -1;
	}
}

int HyyRobotControl::moveC(std::vector<double>& target,std::vector<double>& target_mid,const std::string& velocity,const std::string& zone,const std::string& tool,const std::string& wobj)
{

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotMoveClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotMoveClient->get_service_name());
		return -1;
	}

	moveReq->type = "moveC";
	moveReq->target = target;
	for (size_t i = 0; i < target_mid.size(); i++){
		moveReq->target.push_back(target_mid[i]);
	}
	moveReq->targetstr.clear();
	moveReq->velocity = velocity;
	moveReq->zone = zone;
	moveReq->tool = tool;
	moveReq->wobj = wobj;

	auto res = robotMoveClient->async_send_request(moveReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		// RCLCPP_INFO_STREAM(node_->get_logger(), robotMoveClient->get_service_name() << " response: " << res.get()->result);
		return wait_move_finish(res);
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "moveC: failed to call service " << robotMoveClient->get_service_name());
		return -1;
	}
}

int HyyRobotControl::ask_status(){

	moveReq->type = "move_state";
	int state = 0;
	const int max_retries = 10;  // Maximum number of retries
	int retry_count = 0;        // Current retry attempt
	while (true) {
        if (stop.load()) {
            RCLCPP_INFO(node_->get_logger(), "Signal caught, exiting ask_status...");
            state = -1;
            break;
        }
		// Send the request to get the current state of the robot
		auto res = robotMoveClient->async_send_request(moveReq);
		// Wait for the response from the service
		if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
			state = res.get()->result;  // Obtain the result once
			if (state == 0) {
				// If state is 0, movement has successfully finished
				RCLCPP_INFO_STREAM(node_->get_logger(), "robot move finished, state = " << state);
				break; // Exit the loop
			} else if (state > 0) {
				// If state is positive, robot is still moving
				RCLCPP_INFO_STREAM(node_->get_logger(), "robot is moving, state = " << state);
				usleep(100000);  // Sleep for a short time before next query
			} else {
				// If state is negative, an error occurred
				RCLCPP_INFO_STREAM(node_->get_logger(), "robot move error, state = " << state);
				break;  // Exit the loop due to error
			}
		} else {
			// If the service call failed and retry count has not exceeded maximum retries
			if (retry_count < max_retries) {
				retry_count++;  // Increment retry count
				RCLCPP_WARN(node_->get_logger(), "Failed to receive response from service, retrying (%d/%d)", retry_count, max_retries);
				usleep(100000);  // Wait a bit before retrying
			} else {
				// Maximum retries reached, log an error and exit loop
				state = -1;
				RCLCPP_ERROR(node_->get_logger(), "Failed to receive response from service after multiple attempts.");
				break;
			}
		}
	}
	return state;
}

int HyyRobotControl::wait_move_finish(rclcpp::Client<hyy_robot_control::hyyMoveMsg>::FutureAndRequestId &resp)
{
    if (block_flag) {
        moveReq->type = "move_state";
        int state = 0;
        const int max_retries = 10;  // Maximum number of retries
        int retry_count = 0;        // Current retry attempt

        while (true) {
            // Send the request to get the current state of the robot
            auto res = robotMoveClient->async_send_request(moveReq);
            // Wait for the response from the service
            if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
                state = res.get()->result;  // Obtain the result once

                if (state == 0) {
                    // If state is 0, movement has successfully finished
					RCLCPP_INFO_STREAM(node_->get_logger(), "robot move finished, state = " << state);
					break; // Exit the loop
				} else if (state > 0) {
                    // If state is positive, robot is still moving
                    RCLCPP_INFO_STREAM(node_->get_logger(), "robot is moving, state = " << state);
                    usleep(100000);  // Sleep for a short time before next query
                } else {
                    // If state is negative, an error occurred
                    RCLCPP_INFO_STREAM(node_->get_logger(), "robot move error, state = " << state);
                    break;  // Exit the loop due to error
                }
            } else {
                // If the service call failed and retry count has not exceeded maximum retries
                if (retry_count < max_retries) {
                    retry_count++;  // Increment retry count
                    RCLCPP_WARN(node_->get_logger(), "Failed to receive response from service, retrying (%d/%d)", retry_count, max_retries);
                    usleep(100000);  // Wait a bit before retrying
                } else {
                    // Maximum retries reached, log an error and exit loop
                    RCLCPP_ERROR(node_->get_logger(), "Failed to receive response from service after multiple attempts.");
                    break;
                }
            }
        }
        return state;  // Return the last known state
    } else {
        // If not blocking, return the immediate result of the service call
        return resp.get()->result;
    }
}

void HyyRobotControl::isblock(bool value)
{
	block_flag = value;
}

int HyyRobotControl::getDI(int index)
{
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotIoClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotIoClient->get_service_name());
		return -1;
	}
	ioReq->type = "DI";
	ioReq->index = index;
	auto res = robotIoClient->async_send_request(ioReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		int result_ = res.get()->result;
		RCLCPP_INFO_STREAM(node_->get_logger(), "get DI(" << ioReq->index << ") successfully, value: " << result_);
		return result_;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "getDI: failed to call service " << robotIoClient->get_service_name());
		return -1;
	}
}

int HyyRobotControl::setDO(int index, bool value)
{
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotIoClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotIoClient->get_service_name());
		return -1;
	}
	ioReq->type = "DO";
	ioReq->index = index;
	ioReq->value = value;
	auto res = robotIoClient->async_send_request(ioReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		RCLCPP_INFO_STREAM(node_->get_logger(), "set DO(" << ioReq->index << ") successfully, value: " << ioReq->value);
		return res.get()->result;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "setDO: failed to call service " << robotIoClient->get_service_name());
		return -1;
	}
}

std::vector<double> HyyRobotControl::get_joint_data(const std::string& name)
{
	std::vector<double> empty_data;
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty_data;
	}

	if (!robotMoveDataClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotMoveDataClient->get_service_name());
		return empty_data;
	}

	moveDataReq->type = "joint";
	moveDataReq->datastr = name;
	moveDataReq->tool.clear();
	moveDataReq->wobj.clear();
	auto res = robotMoveDataClient->async_send_request(moveDataReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		RCLCPP_INFO_STREAM(node_->get_logger(), "Success in get joint data \" " << moveDataReq->datastr << " \"");
		return res.get()->result;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "get_joint_data: failed to call service " << robotMoveDataClient->get_service_name());
		return empty_data;
	}
}

std::vector<double> HyyRobotControl::get_cartesian_data(const std::string& name)
{
	std::vector<double> empty_data;
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty_data;
	}

	if (!robotMoveDataClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotMoveDataClient->get_service_name());
		return empty_data;
	}

	moveDataReq->type = "cartesian";
	moveDataReq->datastr = name;
	moveDataReq->tool.clear();
	moveDataReq->wobj.clear();
	auto res = robotMoveDataClient->async_send_request(moveDataReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		RCLCPP_INFO_STREAM(node_->get_logger(), "Success in get cartesian data \" " << moveDataReq->datastr << " \"");
		return res.get()->result;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "get_cartesian_data: failed to call service " << robotMoveDataClient->get_service_name());
		return empty_data;
	}
}

std::vector<double> HyyRobotControl::get_joint_current(void)
{
	std::vector<double> empty_data;
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty_data;
	}
	moveDataReq->type = "joint_current";
	moveDataReq->datastr.clear();
	moveDataReq->tool.clear();
	moveDataReq->wobj.clear();
	auto res = robotMoveDataClient->async_send_request(moveDataReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		RCLCPP_INFO_STREAM(node_->get_logger(), "Success in get current robot joint data");
		return res.get()->result;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "get_joint_current: failed to call service " << robotMoveDataClient->get_service_name());
		return empty_data;
	}
}

std::vector<double> HyyRobotControl::get_cartesian_current(const std::string& tool,const std::string& wobj)
{
	std::vector<double> empty_data;
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty_data;
	}
	moveDataReq->type = "cartesian_current";
	moveDataReq->datastr.clear();
	moveDataReq->tool = tool;
	moveDataReq->wobj = wobj;
	auto res = robotMoveDataClient->async_send_request(moveDataReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		RCLCPP_INFO_STREAM(node_->get_logger(), "Success in get current robot cartesian data");
		return res.get()->result;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "get_cartesian_current: failed to call service " << robotMoveDataClient->get_service_name());
		return empty_data;
	}
}

std::vector<double> HyyRobotControl::offs(std::vector<double>& target, double x, double y, double z, double k, double p, double s)
{
	std::vector<double> of;
	of.clear();
	if (target.empty()){
		RCLCPP_INFO(node_->get_logger(), "offs: target data empty");
		return of;
	}
	of.push_back(target[0] + x);
	of.push_back(target[1] + y);
	of.push_back(target[2] + z);

	double R0[3][3];
	double R1[3][3];
	double Rres[3][3];
	double rpy[3] = {k, p, s};
	double kps[3] = {target[3], target[4], target[5]};
	rpy2tr(kps, R0, 2);
	rpy2tr(rpy, R1, 2);
	Rmulti(R1, R0, Rres);
	tr2rpy(Rres, kps, 2);
	of.push_back(kps[0]);
	of.push_back(kps[1]);
	of.push_back(kps[2]);

	return of;
}

std::vector<double> HyyRobotControl::offsrel(std::vector<double>& target, double x, double y, double z, double k, double p, double s)
{
	std::vector<double> of;
	of.clear();
	if (target.empty())
	{
		RCLCPP_INFO(node_->get_logger(), "offsrel: target data empty");
		return of;
	}
	double R0[3][3];
	double R1[3][3];
	double Rres[3][3];
	double rpy[3] = {k, p, s};
	double kps[3] = {target[3], target[4], target[5]};
	rpy2tr(kps, R0, 2);
	rpy2tr(rpy, R1, 2);
	Rmulti(R0, R1, Rres);
	tr2rpy(Rres, kps, 2);

	double _pos[3];
	double pos[3]={x,y,z};
	RMultVec(R0, pos, _pos);

	of.push_back(target[0]+_pos[0]);
	of.push_back(target[1]+_pos[1]);
	of.push_back(target[2]+_pos[2]);
	of.push_back(kps[0]);
	of.push_back(kps[1]);
	of.push_back(kps[2]);

	return of;
}

int HyyRobotControl::grip_create(const std::string& name)
{

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotGripClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotGripClient->get_service_name());
		return -1;
	}

	gripReq->type = name;
	gripReq->value = 2;

	auto res = robotGripClient->async_send_request(gripReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		auto result_ = res.get()->result;
		RCLCPP_INFO(node_->get_logger(), "grip_create received response: %d", result_);
		return result_;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "grip_create: failed to call service " << robotGripClient->get_service_name());
		return -1;
	}

}

int HyyRobotControl::grip_control(const std::string& name, double position)
{

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotGripClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotGripClient->get_service_name());
		return -1;
	}

	gripReq->type = name;
	gripReq->value = position;
	auto res = robotGripClient->async_send_request(gripReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		auto result_ = res.get()->result;
		RCLCPP_INFO(node_->get_logger(), "grip_control received response: %d", result_);
		return result_;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "grip_control: failed to call service " << robotGripClient->get_service_name());
		return -1;
	}
}

int HyyRobotControl::grip_destroy(const std::string& name)
{

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!robotGripClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", robotGripClient->get_service_name());
		return -1;
	}

	gripReq->type = name;
	gripReq->value = -1;
	auto res = robotGripClient->async_send_request(gripReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		auto result_ = res.get()->result;
		RCLCPP_INFO(node_->get_logger(), "grip_destroy received response: %d", result_);
		return result_;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "grip_destroy: failed to call service " << robotGripClient->get_service_name());
		return -1;
	}
}

void HyyRobotControl::tr2rpy(double R[3][3], double* rpy, int flag)
{
	double eps = 0.000001;
	double s = 0.0;
	double c = 0.0;
	double cp = 0.0;
	//double Q = 0.0;
	double m=0.0;

	//matlab 四元数
	double qs=0;
	double kx=0;
	double ky=0;
	double kz=0;
	double kx1=0;
	double ky1=0;
	double kz1=0;
	int add=0;
	double nm=0;
	switch (flag)
	{
	case 0://XYZ order
		if (fabs(R[2][2])<eps&&fabs(R[1][2])<eps)
		{
			//singularity
			rpy[0] = 0;//roll is zero
			rpy[1] =R_PI/2;// atan2(R[0][2], R[2][2]);//pitch
			rpy[2] = atan2(R[1][0], R[1][1]);//yaw is sum of roll+yaw

		}
		else
		{
			rpy[0] = atan2(-R[1][2], R[2][2]);//roll
			s = sin(rpy[0]);
			c = cos(rpy[0]);
			rpy[1] = atan2(R[0][2], c*R[2][2] - s*R[1][2]);//pitch
			rpy[2] = atan2(-R[0][1], R[0][0]);//yaw

		}
		break;
	case 1://ZYX order
		if (fabs(R[0][0])<eps&&fabs(R[1][0])<eps)
		{
			//singularity
			rpy[0] = 0;//roll is zero
			rpy[1] = atan2(-R[2][0], R[0][0]);//pitch
			rpy[2] = atan2(-R[1][2], R[1][1]);//yaw is sum of roll+yaw
		}
		else
		{
			rpy[0] = atan2(R[1][0], R[0][0]);//roll
			s = sin(rpy[0]);
			c = cos(rpy[0]);
			rpy[1] = atan2(-R[2][0], c*R[0][0] + s*R[1][0]);//pitch
			rpy[2] = atan2(s*R[0][2] - c*R[1][2], c*R[1][1] - s*R[0][1]);//yaw

		}
		break;
	case 2://XYZ HLHN J.CRAIG method
		rpy[1] = atan2(-R[2][0],sqrt(R[0][0]*R[0][0]+R[1][0]*R[1][0])); //pitch
		if (fabs(fabs(rpy[1]) - R_PI / 2.0) < eps)
		{
			if (rpy[1] > 0) //pi/2
			{
				rpy[1] = R_PI / 2.0;
				rpy[2] = 0.0;
				rpy[0] = atan2(R[0][1],R[1][1]);
			}
			else//-pi/2
			{
				rpy[1] = -R_PI / 2.0;
				rpy[2] = 0.0;
				rpy[0] = -atan2(R[0][1], R[1][1]);
			}
		}
		else
		{
			cp = cos(rpy[1]);
			rpy[2] = atan2(R[1][0]/cp,R[0][0]/cp);
			rpy[0] = atan2(R[2][1]/cp, R[2][2]/cp);
		}
		break;
	case 3://��Чת�ᷨ  //orocos
		cp=(R[0][0]+R[1][1]+R[2][2]-1)/2.0;
		if (cp>1-(eps*(1e-6)))
		{
        	rpy[0] = 0;
        	rpy[1] = 0;
        	rpy[2] = 0;
		}else if (cp<-1+(eps*(1e-6)))
		{
			rpy[0] = sqrt( (R[0][0]+1.0)/2);
			rpy[1] = sqrt( (R[1][1]+1.0)/2);
			rpy[2] = sqrt( (R[2][2]+1.0)/2);
			if ( R[0][2]< 0) rpy[0] =-rpy[0] ;
			if ( R[2][1]< 0) rpy[1] =-rpy[1] ;
			if ( rpy[0]*rpy[1]*R[0][1] < 0) rpy[0]=-rpy[0];  // this last line can be necessary when z is 0
			// z always >= 0
			// if z equal to zero
			rpy[0] = rpy[0]*R_PI;
			rpy[1] = rpy[1]*R_PI;
			rpy[2] = rpy[2]*R_PI;
		}
		else
		{
//			Q=acos(cp);
//
//	    	rpy[0] = Q*0.5*(R[2][1]-R[1][2])/sin(Q);
//	    	rpy[1] = Q*0.5*(R[0][2]-R[2][0])/sin(Q);
//	    	rpy[2] = Q*0.5*(R[1][0]-R[0][1])/sin(Q);


			 double angle;
			double mod_axis;
			double axisx, axisy, axisz;
			axisx = R[2][1]-R[1][2];
			axisy = R[0][2]-R[2][0];
			axisz = R[1][0]-R[0][1];
			mod_axis = sqrt(axisx*axisx+axisy*axisy+axisz*axisz);
			angle = atan2(mod_axis/2,cp);
			rpy[0] = angle*(R[2][1]-R[1][2])/mod_axis;
			rpy[1] = angle*(R[0][2]-R[2][0])/mod_axis;
			rpy[2] = angle*(R[1][0]-R[0][1])/mod_axis;
		}


		break;
	case 4://单位四元数
			m=0.5*sqrt(1+R[0][0]+R[1][1]+R[2][2]);
			if (fabs(m)<eps)
			{
				RCLCPP_ERROR(node_->get_logger(), "quaternion no solution!");
				break;
			}
			rpy[3]=m;
			rpy[0]=(R[2][1]-R[1][2])/(4*rpy[3]);
			rpy[1]=(R[0][2]-R[2][0])/(4*rpy[3]);
			rpy[2]=(R[1][0]-R[0][1])/(4*rpy[3]);
			break;
	case 5: //matlab 机器人工具箱
		 qs = sqrt((R[0][0]+R[1][1]+R[2][2])+1)/2.0;
		 kx = R[2][1] - R[1][2];   // Oz - Ay
		 ky =R[0][2] - R[2][0];   //Ax - Nz
		 kz = R[1][0] - R[0][1];   // Ny - Ox

		 if ((R[0][0]>=R[1][1]) && (R[0][0]>= R[2][2]) )
		 {
				kx1 = R[0][0]- R[1][1] - R[2][2] + 1; // Nx - Oy - Az + 1
				ky1 = R[1][0] + R[0][1];          // Ny + Ox
				kz1 = R[2][0]+ R[0][2];          // Nz + Ax
				if (kx >= 0)
				{
					add=1;
				}
				else
				{
					add=0;
				}
		 }
		 else if  (R[1][1]>= R[2][2])
		 {
		        kx1 = R[1][0]+ R[0][1];          //Ny + Ox
		        ky1 = R[1][1]- R[0][0]- R[2][2] + 1; // Oy - Nx - Az + 1
		        kz1 = R[2][1] + R[1][2];          // Oz + Ay
				if (ky >= 0)
				{
					add=1;
				}
				else
				{
					add=0;
				}
		 }
		else
		{
				kx1 = R[2][0] + R[0][2];          // Nz + Ax
				ky1 = R[2][1]+ R[1][2];          //Oz + Ay
				kz1 = R[2][2] - R[0][0] - R[1][1]+ 1; // Az - Nx - Oy + 1
				if (kz >= 0)
				{
					add=1;
				}
				else
				{
					add=0;
				}
		}
		 if( add)
		 {
				 kx = kx + kx1;
				 ky = ky + ky1;
				 kz = kz + kz1;
		 }
		 else
		 {
				 kx = kx - kx1;
				 ky = ky - ky1;
				 kz = kz - kz1;
		 }
		 nm=sqrt(kx*kx+ky*ky+kz*kz);
		 if (nm<0.000001)
		 {
			 	 rpy[3]=1;rpy[0]=0;rpy[1]=0;rpy[2]=0;

		 }
		 else
		 {
			 	 s=sqrt(1-qs*qs)/nm;
			 	rpy[3]=qs;rpy[0]=s*kx;rpy[1]=s*ky;rpy[2]=s*kz;

		 }
		break;
	default:
		break;
	}

}

void HyyRobotControl::rpy2tr(double* rpy, double R[3][3], int flag)
{
	double sx = 0;
	double cx = 0;
	double sy = 0;
	double cy = 0;
	double sz = 0;
	double cz = 0;

	double Q = 0;
	double k[3];
	double v=0;

	/*
	Rx = [
	1   0    0
	0   cx  -sx
	0   sx   cx
	];
	Ry = [
	cy  0   sy
	0   1   0
	-sy  0   cy
	];
	Rz = [
	cz  -sz  0
	sz   cz  0
	0    0   1
	];
	*/
	switch (flag)
	{
	case 0://XYZ order
		sx = sin(rpy[0]);
		cx = cos(rpy[0]);
		sy = sin(rpy[1]);
		cy = cos(rpy[1]);
		sz = sin(rpy[2]);
		cz = cos(rpy[2]);
		//R=Rx*Ry*Rz
		R[0][0] = cy*cz; R[0][1] = -cy*sz; R[0][2] = sy;
		R[1][0] = cx*sz + cz*sx*sy; R[1][1] = cx*cz - sx*sy*sz; R[1][2] = -cy*sx;
		R[2][0] = sx*sz - cx*cz*sy; R[2][1] = cz*sx + cx*sy*sz; R[2][2] = cx*cy;
		break;
	case 1://ZYX order
		sx = sin(rpy[2]);
		cx = cos(rpy[2]);
		sy = sin(rpy[1]);
		cy = cos(rpy[1]);
		sz = sin(rpy[0]);
		cz = cos(rpy[0]);
		//R=Rz*Ry*Rx
		R[0][0] = cy*cz; R[0][1] = cz*sx*sy - cx*sz; R[0][2] = sx*sz + cx*cz*sy;
		R[1][0] = cy*sz; R[1][1] = cx*cz + sx*sy*sz; R[1][2] = cx*sy*sz - cz*sx;
		R[2][0] = -sy; R[2][1] = cy*sx; R[2][2] = cx*cy;
		break;
	case 2://XYZ HLHN J.CRAIG method
		sx = sin(rpy[0]);
		cx = cos(rpy[0]);
		sy = sin(rpy[1]);
		cy = cos(rpy[1]);
		sz = sin(rpy[2]);
		cz = cos(rpy[2]);
		//R=Rz*Ry*Rx
		R[0][0] = cy*cz; R[0][1] = cz*sx*sy - cx*sz; R[0][2] = sx*sz + cx*cz*sy;
		R[1][0] = cy*sz; R[1][1] = cx*cz + sx*sy*sz; R[1][2] = cx*sy*sz - cz*sx;
		R[2][0] = -sy; R[2][1] = cy*sx; R[2][2] = cx*cy;
		break;
	case 3://��Чת�ᷨ
		Q=sqrt(rpy[0]*rpy[0]+rpy[1]*rpy[1]+rpy[2]*rpy[2]);
		if (Q<1e-10)
		{
	        R[0][0] = 1;	   R[0][1] = 0;	 R[0][2] = 0;
	        R[1][0] = 0;    R[1][1] = 1;   R[1][2] = 0;
	        R[2][0] = 0;	   R[2][1] = 0;   R[2][2] = 1;
        	//Rdebug("rpy2tr no solution\n");
        	break;
		}
        k[0]=rpy[0]/Q;
        k[1]=rpy[1]/Q;
        k[2]=rpy[2]/Q;
        sx=sin(Q);cx=cos(Q);v=1-cx;
        R[0][0] = k[0]*k[0]*v + cx;			R[0][1] = k[0]*k[1]*v - k[2]*sx;	R[0][2] = k[0]*k[2]*v + k[1]*sx;
        R[1][0] = k[0]*k[1]*v + k[2]*sx;	R[1][1] = k[1]*k[1]*v + cx;			R[1][2] = k[1]*k[2]*v - k[0]*sx;
        R[2][0] = k[0]*k[2]*v - k[1]*sx;	R[2][1] = k[1]*k[2]*v + k[0]*sx;	R[2][2] = k[2]*k[2]*v + cx;
		break;
	case 4://单位四元数
			R[0][0] = 1-2*rpy[1]*rpy[1]-2*rpy[2]*rpy[2]; R[0][1] = 2*(rpy[0]*rpy[1]-rpy[2]*rpy[3]); R[0][2] = 2*(rpy[0]*rpy[2]+rpy[1]*rpy[3]);
			R[1][0] = 2*(rpy[0]*rpy[1]+rpy[2]*rpy[3]); R[1][1] = 1-2*rpy[0]*rpy[0]-2*rpy[2]*rpy[2]; R[1][2] = 2*(rpy[1]*rpy[2]-rpy[0]*rpy[3]);
			R[2][0] = 2*(rpy[0]*rpy[2]-rpy[1]*rpy[3]); R[2][1] = 2*(rpy[1]*rpy[2]+rpy[0]*rpy[3]); R[2][2] = 1-2*rpy[0]*rpy[0]-2*rpy[1]*rpy[1];
			break;
	case 5: //matlab 机器人工具箱
		    R[0][0] = 1-2*rpy[1]*rpy[1]-2*rpy[2]*rpy[2]; R[0][1] = 2*(rpy[0]*rpy[1]-rpy[2]*rpy[3]); R[0][2] = 2*(rpy[0]*rpy[2]+rpy[1]*rpy[3]);
			R[1][0] = 2*(rpy[0]*rpy[1]+rpy[2]*rpy[3]); R[1][1] = 1-2*rpy[0]*rpy[0]-2*rpy[2]*rpy[2]; R[1][2] = 2*(rpy[1]*rpy[2]-rpy[0]*rpy[3]);
			R[2][0] = 2*(rpy[0]*rpy[2]-rpy[1]*rpy[3]); R[2][1] = 2*(rpy[1]*rpy[2]+rpy[0]*rpy[3]); R[2][2] = 1-2*rpy[0]*rpy[0]-2*rpy[1]*rpy[1];
		break;
	default:
		break;
	}

}

void HyyRobotControl::Rmulti(double R0[3][3],double R1[3][3],double Rres[3][3])
{
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			Rres[i][j] = 0;
			for (int k = 0; k < 3; k++){
				Rres[i][j] = Rres[i][j] + R0[i][k] * R1[k][j];
			}
		}
	}
}

void HyyRobotControl::RMultVec(double(*R)[3], double* v, double * vres)
{
	vres[0] = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
	vres[1] = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
	vres[2] = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
}

bool HyyRobotControl::hand_SetAngle(std::vector<int> angle, const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return false;
	}
	if (!hyySetangleClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyySetangleClient->get_service_name());
		return false;
	}
	SetangleReq->status = "set_angle";
	SetangleReq->hand_id = hand_id;
	SetangleReq->angle.clear();
	for (int i = 0; i < 6; i++){
		SetangleReq->angle.push_back(angle[i]);
	}

	auto res = hyySetangleClient->async_send_request(SetangleReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		usleep(250000);
		std::vector<int> position = hand_GetAngleAct();
		return res.get()->angle_accepted;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyySetangleClient->get_service_name());
		return false;
	}
}

bool HyyRobotControl::hand_fullopen(){
	std::vector<int> angle_ = {1000, 1000, 1000, 1000, 1000, 1000};
	return hand_SetAngle(angle_);
}

bool HyyRobotControl::hand_SetForce(std::vector<int> force, const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return false;
	}
	if (!hyySetforceClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyySetforceClient->get_service_name());
		return false;
	}
	SetforceReq->status = "set_force";
	SetforceReq->hand_id = hand_id;
	SetforceReq->force.clear();
	for (int i = 0; i < 6; i++){
		SetforceReq->force.push_back(force[i]);
	}
	auto res = hyySetforceClient->async_send_request(SetforceReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->force_accepted;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyySetforceClient->get_service_name());
		return false;
	}

	return true;
}

bool HyyRobotControl::hand_SetPos(std::vector<int> pos, const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return false;
	}
	if (!hyySetposClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyySetposClient->get_service_name());
		return false;
	}
	SetposReq->status = "set_pos";
	SetposReq->hand_id = hand_id;
	SetposReq->pos.clear();
	for (int i = 0; i < 6; i++){
		SetposReq->pos.push_back(pos[i]);
	}
	auto res = hyySetposClient->async_send_request(SetposReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		usleep(250000);
		std::vector<int> position = hand_GetAngleAct();
		return res.get()->pos_accepted;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyySetposClient->get_service_name());
		return false;
	}
}

bool HyyRobotControl::hand_SetSpeed(std::vector<int> speed, const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return false;
	}
	if (!hyySetspeedClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyySetspeedClient->get_service_name());
		return false;
	}
	SetspeedReq->status = "set_speed";
	SetspeedReq->hand_id = hand_id;
	SetspeedReq->speed.clear();
	for (int i = 0; i < 6; i++){
		SetspeedReq->speed.push_back(speed[i]);
	}
	auto res = hyySetspeedClient->async_send_request(SetspeedReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->speed_accepted;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyySetspeedClient->get_service_name());
		return false;
	}

	return true;

}

std::vector<int> HyyRobotControl::hand_GetAngleAct(const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty;
	}
	if (!hyyGetangleactClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGetangleactClient->get_service_name());
		return empty;
	}
	GetangleactReq->status = "get_angleact";
	GetangleactReq->hand_id = hand_id;

	auto res = hyyGetangleactClient->async_send_request(GetangleactReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->curangleact;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyyGetangleactClient->get_service_name());
		return empty;
	}

}

std::vector<int> HyyRobotControl::hand_GetAngleSet(const int hand_id){

	
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty;
	}
	if (!hyyGetanglesetClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGetanglesetClient->get_service_name());
		return empty;
	}
	GetanglesetReq->status = "get_angleset";
	GetanglesetReq->hand_id = hand_id;

	auto res = hyyGetanglesetClient->async_send_request(GetanglesetReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->curangleset;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyyGetanglesetClient->get_service_name());
		return empty;
	}

}

std::vector<int> HyyRobotControl::hand_GetCurrentAct(const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty;
	}
	if (!hyyGetcurrentactClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGetcurrentactClient->get_service_name());
		return empty;
	}
	GetcurrentactReq->status = "get_currentact";
	GetcurrentactReq->hand_id = hand_id;

	auto res = hyyGetcurrentactClient->async_send_request(GetcurrentactReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->curcurrent;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyyGetcurrentactClient->get_service_name());
		return empty;
	}

}

std::vector<int> HyyRobotControl::hand_GetError(const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty;
	}
	if (!hyyGeterrorClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGeterrorClient->get_service_name());
		return empty;
	}
	GeterrorReq->status = "get_error";
	GeterrorReq->hand_id = hand_id;

	auto res = hyyGeterrorClient->async_send_request(GeterrorReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->error;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyyGeterrorClient->get_service_name());
		return empty;
	}

}

std::vector<int> HyyRobotControl::hand_GetForceAct(const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty;
	}
	if (!hyyGetforceactClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGetforceactClient->get_service_name());
		return empty;
	}
	GetforceactReq->status = "get_forceact";
	GetforceactReq->hand_id = hand_id;

	auto res = hyyGetforceactClient->async_send_request(GetforceactReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->curforceact;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyyGetforceactClient->get_service_name());
		return empty;
	}

}

std::vector<int> HyyRobotControl::hand_GetForceSet(const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty;
	}
	if (!hyyGetforcesetClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGetforcesetClient->get_service_name());
		return empty;
	}
	GetforcesetReq->status = "get_forceset";
	GetforcesetReq->hand_id = hand_id;

	auto res = hyyGetforcesetClient->async_send_request(GetforcesetReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->curforceset;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyyGetforcesetClient->get_service_name());
		return empty;
	}

}

std::vector<int> HyyRobotControl::hand_GetPosAct(const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty;
	}
	if (!hyyGetposactClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGetposactClient->get_service_name());
		return empty;
	}
	GetposactReq->status = "get_posact";
	GetposactReq->hand_id = hand_id;

	auto res = hyyGetposactClient->async_send_request(GetposactReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->curposact;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyyGetposactClient->get_service_name());
		return empty;
	}

}

std::vector<int> HyyRobotControl::hand_GetPosSet(const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty;
	}
	if (!hyyGetpossetClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGetpossetClient->get_service_name());
		return empty;
	}
	GetpossetReq->status = "get_posset";
	GetpossetReq->hand_id = hand_id;

	auto res = hyyGetpossetClient->async_send_request(GetpossetReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->curposset;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyyGetpossetClient->get_service_name());
		return empty;
	}

}

std::vector<int> HyyRobotControl::hand_GetSpeedSet(const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty;
	}
	if (!hyyGetspeedsetClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGetspeedsetClient->get_service_name());
		return empty;
	}
	GetspeedsetReq->status = "get_speedset";
	GetspeedsetReq->hand_id = hand_id;

	auto res = hyyGetspeedsetClient->async_send_request(GetspeedsetReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->curspeedset;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyyGetspeedsetClient->get_service_name());
		return empty;
	}

}

std::vector<int> HyyRobotControl::hand_GetTemp(const int hand_id){

	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return empty;
	}
	if (!hyyGettempClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGettempClient->get_service_name());
		return empty;
	}
	GettempReq->status = "get_temp";
	GettempReq->hand_id = hand_id;

	auto res = hyyGettempClient->async_send_request(GettempReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->temp;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "hand: failed to call service " << hyyGettempClient->get_service_name());
		return empty;
	}

}

bool HyyRobotControl::Gripper_SetPos(const int pos){
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return false;
	}
	if (!hyySetGripPosClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyySetGripPosClient->get_service_name());
		return false;
	}
	SetGripPosReq->type = "set_position";
	SetGripPosReq->value = pos;

	auto res = hyySetGripPosClient->async_send_request(SetGripPosReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->result;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "Gripper: failed to call service " << hyySetGripPosClient->get_service_name());
		return false;
	}

}

bool HyyRobotControl::Gripper_SetSpeed(const int speed){
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return false;
	}
	if (!hyySetGripSpeedClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyySetGripSpeedClient->get_service_name());
		return false;
	}
	SetGripSpeedReq->type = "set_speed";
	SetGripSpeedReq->value = speed;

	auto res = hyySetGripSpeedClient->async_send_request(SetGripSpeedReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->result;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "Gripper: failed to call service " << hyySetGripSpeedClient->get_service_name());
		return false;
	}
}

bool HyyRobotControl::Gripper_SetForce(const int force){
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return false;
	}
	if (!hyySetGripForceClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyySetGripForceClient->get_service_name());
		return false;
	}
	SetGripForceReq->type = "set_force";
	SetGripForceReq->value = force;

	auto res = hyySetGripForceClient->async_send_request(SetGripForceReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->result;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "Gripper: failed to call service " << hyySetGripForceClient->get_service_name());
		return false;
	}
}

bool HyyRobotControl::Gripper_Move(){
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return false;
	}
	if (!hyyGripMoveClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGripMoveClient->get_service_name());
		return false;
	}
	GripMoveReq->type = "goto";

	auto res = hyyGripMoveClient->async_send_request(GripMoveReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->result;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "Gripper: failed to call service " << hyyGripMoveClient->get_service_name());
		return false;
	}
}

bool HyyRobotControl::Gripper_goto(const int pos){
	Gripper_SetPos(pos);
	Gripper_Move();
	return true;
}

bool HyyRobotControl::Gripper_fullopen(){
	Gripper_goto(0);
	return true;
}

bool HyyRobotControl::Gripper_fullclose(){
	Gripper_goto(255);
	return true;
}

bool HyyRobotControl::Gripper_Activate(){
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return false;
	}
	if (!hyyActivateGripClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyActivateGripClient->get_service_name());
		return false;
	}
	ActivateGripReq->type = "activate";

	auto res = hyyActivateGripClient->async_send_request(ActivateGripReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->result;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "Gripper: failed to call service " << hyyActivateGripClient->get_service_name());
		return false;
	}
}

bool HyyRobotControl::Gripper_Deactivate(){
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return false;
	}
	if (!hyyDeactivateGripClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyDeactivateGripClient->get_service_name());
		return false;
	}
	DeactivateGripReq->type = "deactivate";

	auto res = hyyDeactivateGripClient->async_send_request(DeactivateGripReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->result;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "Gripper: failed to call service " << hyyDeactivateGripClient->get_service_name());
		return false;
	}
}

bool HyyRobotControl::Gripper_initialize(){
	Gripper_Deactivate();
	sleep(1);
	Gripper_Activate();
	sleep(1);
	Gripper_SetForce(255);
	Gripper_SetSpeed(255);
	sleep(2);
	RCLCPP_INFO(node_->get_logger(), "Gripper: initialize success.");
	return true;
}

int HyyRobotControl::Gripper_GetStatus(){
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!hyyGetGripStatusClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGetGripStatusClient->get_service_name());
		return -1;
	}
	GetGripStatusReq->type = "isactive";

	auto res = hyyGetGripStatusClient->async_send_request(GetGripStatusReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return (res.get()->result & 1);
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "Gripper: failed to call service " << hyyGetGripStatusClient->get_service_name());
		return -1;
	}
}

int HyyRobotControl::Gripper_GetPos(){
	if (!init_flag){
		RCLCPP_ERROR(node_->get_logger(), "please init()");
		return -1;
	}
	if (!hyyGetGripPosClient->wait_for_service(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", hyyGetGripPosClient->get_service_name());
		return -1;
	}
	GetGripPosReq->type = "get_position";

	auto res = hyyGetGripPosClient->async_send_request(GetGripPosReq);
	if (res.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
		return res.get()->result;
	} else {
		RCLCPP_ERROR_STREAM(node_->get_logger(), "Gripper: failed to call service " << hyyGetGripPosClient->get_service_name());
		return -1;
	}
}

}