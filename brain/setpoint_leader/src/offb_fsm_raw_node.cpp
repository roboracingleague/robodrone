#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetModeRequest.h>
#include <iostream>
#include <sys/time.h>
#include <fstream>
using namespace std;

//vel
#include <geometry_msgs/TwistStamped.h>

#include <setpoint_leader/robocars_attitude.h>
#include <setpoint_leader/robocars_mission.h>
#include <setpoint_leader/robocars_debug.h>

#include "offb_fsm.hpp" 

// // Flight types
// enum TypeMasks {
// MASK_POSITION_VELOCITY =0b0000111111000000,
// MASK_POSITION =         0b0000111111111000,
// MASK_VELOCITY =         0b0000111111000111,
// MASK_TAKEOFF_POSITION = 0b0001111111111000,
// MASK_TAKEOFF =          0b0001111111111111,
// MASK_LAND_VELOCITY =    0b0010111111000111,
// MASK_LAND_POSITION =    0b0010111111111000,
// MASK_LAND =             0b0010111111111111,
// MASK_LOITER_POSITION =  0b0100111111111000,
// MASK_IDLE_POSITION =    0b1000111111111000,
// };

// 1000 1111 1111 1000
// 0100 0000 0000 0000

MissionRosInterface * mri;

float MissionStateMachine::destX=0.0f;
float MissionStateMachine::destY=0.0f;
float MissionStateMachine::destZ=0.0f;
float MissionStateMachine::destMode=0.0f;

int LandedVehicleCount = 0;

bool offBoardMode = true;
string RCFlightMode = "MANUAL";

bool pauseActive=false;
float prevDestX=0;
float prevDestY=0;
float prevDestZ=0;


class onWaitConnect;
class onWaitMission;
class onRecordMission;
class onStreamIdlePosition;
class onSwitchOffBoard;
class onArm;
class onDisarm;
class onMission;
class onReachDeparture;
class onGoing;
class onLand;
class onLoiter;
class onFreestyle;
class onStopOffBoard;

class onWaitConnect
: public MissionStateMachine
{
    public:
        onWaitConnect() : MissionStateMachine("onWaitConnect") {};

    private:

        void entry(void) override {
            MissionStateMachine::entry();
            destX=0;
            destY=0;
            destZ=0;
        };
  
        void react(ConnectedEvent const & e) override { 
            MissionStateMachine::react(e);
            transit<onWaitMission>();
        };

        void react(DisconnectedEvent const & e) override { 
            MissionStateMachine::react(e);
            transit<onWaitConnect>();
        };

        void react(EnteringOffboardEvent const & e) override { 
        };
        void react(LeavingOffboardEvent const & e) override { 
        };
        void react(ArmedEvent const & e) override { 
        };
        void react(DisarmedEvent const & e) override { 
        };
        void react(SwitchModeEvent const & e) override { 
        };
        void react(NewMissionEvent const & e) override { 
            ROS_INFO("state %s : NewMissionEvent received in bad state",getStateName());
        };

        void react(TickEvent const & e) override { 
        };

};

class onRecordMission
: public MissionStateMachine
{
    public:
        onRecordMission() : MissionStateMachine("onRecordMission") {};

    private:

        string filename="";

        void entry(void) override {
            MissionStateMachine::entry();
            mri->clearRecordedMission();
        };

        void react(RecordCurrentPositionEvent const & e) override { 
            MissionStateMachine::react(e);
            //record position
            mri->recordCurrentPosition(); 
        };

        void react(WaitMissionModeEvent const & e) override { 
            MissionStateMachine::react(e);
            //End of record -- TODO
            mri->saveRecordedMission(filename);
            transit<onWaitMission>();
        };

        void react(ConnectedEvent const & e) override { 
        };

        void react(DisconnectedEvent const & e) override { 
            MissionStateMachine::react(e);
            transit<onWaitConnect>();
        };

        void react(EnteringOffboardEvent const & e) override { 
        };
        void react(LeavingOffboardEvent const & e) override { 
        };
        void react(ArmedEvent const & e) override { 
        };
        void react(DisarmedEvent const & e) override { 
        };
        void react(NewMissionEvent const & e) override { 
            ROS_INFO("state %s : NewMissionEvent received in bad state",getStateName());
        };

        void react(TickEvent const & e) override { 
        };

};

class onWaitMission
: public MissionStateMachine
{
public:
    onWaitMission() : MissionStateMachine("onWaitMission") {};

private:

        void entry(void) override {
            mri->clearMission();
            MissionStateMachine::entry();
            ROS_INFO("state %s : pauseActive = %s",getStateName(), pauseActive ? "true" : "false");
            if (! pauseActive) {
                destX=0;
                destY=0;
                destZ=0;
            }
            ROS_INFO("state %s : destX:%f, destY:%f, destZ:%f",getStateName(), destX, destY, destZ);
        };

        void react(DisarmedEvent const & e) override { 
            //nothing
        };

        void react(ArmedEvent const & e) override { 
            //MissionStateMachine::react(e);
            //transit<onDisarm>();
        };

        void react(DisconnectedEvent const & e) override { 
            MissionStateMachine::react(e);
            transit<onWaitConnect>();
        };

        void react(ConnectedEvent const & e) override { 
        };

        void react(EnteringOffboardEvent const & e) override { 
            // ignore
        }

        void react(LeavingOffboardEvent const & e) override { 
        };

        void react (RecordMissionModeEvent const & e) override {
            MissionStateMachine::react(e);
            transit<onRecordMission>();
        }

        void react(WaitMissionModeEvent const & e) override { 
        };

        void react (NewMissionEvent const & e) override {
            MissionStateMachine::react(e);
            if (offBoardMode) {
                mri->getNextWaypoint(destX, destY, destZ, destMode);
                transit<onSwitchOffBoard>();
            }
        };

        void react (PauseBackMissionEvent const & e) override {
            MissionStateMachine::react(e);
            if ( pauseActive && offBoardMode) {
                pauseActive=false;
                transit<onSwitchOffBoard>();
            }
        };

        void react (ExecuteMissionRecordedEvent const & e) override {
            //execute system command to load mission recorded in mission topic
            MissionStateMachine::react(e);
            if (offBoardMode) {
                //if (mri->loadMissionRecorded(lap_numbers)) {
                if (mri->loadMissionRecorded(1)) {
                    ROS_INFO("loadMissionRecorded() error");
                }
            }
        };

        void react(TickEvent const & e) override { 
            MissionStateMachine::react(e);
            mri->sendRobocarsAttitude();
        };

};


class onStreamIdlePosition
: public MissionStateMachine
{
    public:
        onStreamIdlePosition() : MissionStateMachine("onStreamIdlePosition") {};
        onStreamIdlePosition(const char * subStateName) : MissionStateMachine(subStateName) {};

    protected:
        virtual void react(TickEvent                      const & e) override { 
            mri->sendIdlePosition(destX, destY, destZ);
         };
        virtual void react(ConnectedEvent                 const & e) override { 
            // Just ignore
         };
        virtual void react(DisconnectedEvent              const & e) override { 
            MissionStateMachine::react(e);
            transit<onWaitConnect>();
         };
        virtual void react(EnteringOffboardEvent          const & e) override { 
            MissionStateMachine::react(e);
         };
        virtual void react(LeavingOffboardEvent           const & e) override { 
            MissionStateMachine::react(e);
        };
        virtual void react(ArmedEvent                     const & e) override { 
            // Just ignore
         };
        virtual void react(DisarmedEvent                  const & e) override { 
            MissionStateMachine::react(e);
         };
        virtual void react(NewMissionEvent                const & e) override { 
            MissionStateMachine::react(e);
         };

        virtual void entry(void) { 
            MissionStateMachine::entry();
        };  

};

class onSwitchOffBoard
: public onStreamIdlePosition
{
    public:
        onSwitchOffBoard() : onStreamIdlePosition("onSwitchOffBoard") {};

    private:
        ros::Time last_request;

        void entry(void) override {
            onStreamIdlePosition::entry();
            mri->startOffBoardMode();
            last_request = ros::Time::now();            
        };

        void react(EnteringOffboardEvent const & e) override { 
            onStreamIdlePosition::react(e);
            transit<onArm>();            
        };

        void react(SwitchModeEvent const & e) override { 
            if (!offBoardMode) {
                transit<onStopOffBoard>();            
            }
        };

        void react(LeavingOffboardEvent const & e) override { 
            // ignore
        }

        void react(DisarmedEvent const & e) override { 
            // just ignore
        }

        void react(TickEvent const & e) override { 
            onStreamIdlePosition::react(e);

            if(ros::Time::now() - last_request > ros::Duration(5.0))
            {
                mri->startOffBoardMode();
                last_request = ros::Time::now();
            }
        };

};

class onStopOffBoard
: public onStreamIdlePosition
{
    public:
        onStopOffBoard() : onStreamIdlePosition("onStopOffBoard") {};

    private:
        ros::Time last_request;
        string mode = "MANUAL";

        void entry(void) override {
            onStreamIdlePosition::entry();
            mode = "MANUAL";
            if (!offBoardMode) {
                mode=RCFlightMode;
            }
            mri->stopOffBoardMode(mode);
            last_request = ros::Time::now();
        };

        void react(LeavingOffboardEvent const & e) override {
            onStreamIdlePosition::react(e);
            transit<onWaitMission>();
        }

        void react(DisarmedEvent const & e) override { 
            // just ignore
        }

        void react(TickEvent const & e) override { 
            onStreamIdlePosition::react(e);

            if(ros::Time::now() - last_request > ros::Duration(5.0))
            {
                mri->stopOffBoardMode(mode);
                last_request = ros::Time::now();
            }
        };
};

class onArm
: public onStreamIdlePosition
{
    public:
        onArm() : onStreamIdlePosition("onArm") {};

    private:
        ros::Time last_request;

        void entry(void) override {
            onStreamIdlePosition::entry();
            if (!mri->dry_run ) {
                mri->armVehicule();
                last_request = ros::Time::now();
            }
            else
            {
                //dry run mode
                ROS_INFO("dry_run active !");
                transit<onGoing>();     
            }
            
        };

        void react(ArmedEvent const & e) override { 
            onStreamIdlePosition::react(e);
            transit<onGoing>();            
        };

        void react(EnteringOffboardEvent const & e) override { 
            // ignore
        };

        void react(SwitchModeEvent const & e) override { 
            if (!offBoardMode) {
                transit<onDisarm>();
            }
        };

        void react(LeavingOffboardEvent const & e) override { 
            MissionStateMachine::react(e);
            transit<onWaitMission>();
        };

        void react(TickEvent const & e) override { 
            onStreamIdlePosition::react(e);
            if(ros::Time::now() - last_request > ros::Duration(5.0))
            {
                mri->armVehicule();
                last_request = ros::Time::now();
            }
        };

};


class onMission
: public MissionStateMachine
{
    public:
        onMission() : MissionStateMachine("onMission") {};
        onMission(const char * subStateName) : MissionStateMachine(subStateName) {};

    protected:

        void entry(void) override {
            MissionStateMachine::entry();
        };

        virtual void react(ConnectedEvent                 const & e) override { 
            // Just ignore
         };

        void react(DisconnectedEvent const & e) override { 
            // cleanup mission here 
            MissionStateMachine::react(e);
        };

        virtual void react(EnteringOffboardEvent          const & e) override { 
            // Just ignore
        };
        virtual void react(LeavingOffboardEvent           const & e) override { 
            MissionStateMachine::react(e);
        };
        virtual void react(ArmedEvent                     const & e) override { 
            // Just ignore
         };
        virtual void react(DisarmedEvent                  const & e) override { 
            MissionStateMachine::react(e);
         };
        virtual void react(NewMissionEvent                const & e) override { 
            MissionStateMachine::react(e);
         };
        virtual void react (CancelMissionEvent            const & e) override {
            transit<onWaitMission>();
        };
        virtual void react (SwitchModeEvent            const & e) override {
            MissionStateMachine::react(e);
            if (!offBoardMode) {
                //switch to flight mode without landing nor disarming
                transit<onStopOffBoard>();
            }
        };
        virtual void react (LocalPositionAcquiredEvent    const & e) override {
        }
        void react(TickEvent const & e) override { 
            MissionStateMachine::react(e);
            mri->sendRobocarsAttitude();
        };
};


class onGoing : public onMission {
    public:
        onGoing() : onMission("onGoing") {};

    private:

        void entry(void) override {
            onMission::entry();
            mri->sendMissionOrder(MASK_POSITION,destX, destY, destZ);
        };

        void react(DisconnectedEvent const & e) override { 
            onMission::react(e);
            transit<onWaitConnect>();
        };

        virtual void react(EnteringOffboardEvent          const & e) override { 
            // Just ignore
        };
        virtual void react(LeavingOffboardEvent           const & e) override { 
            onMission::react(e);
            transit<onWaitMission>();
        };
        virtual void react(ArmedEvent                     const & e) override { 
            // Just ignore
         };
        virtual void react(DisarmedEvent                  const & e) override { 
            onMission::react(e);
            transit<onWaitMission>();
         };
        virtual void react(NewMissionEvent                const & e) override { 
            onMission::react(e);
         };

        virtual void react (LocalPositionAcquiredEvent    const & e) override {

        };

        virtual void react(Pause1MissionEvent                const & e) override { 
            //drone must loiter on its current position
            pauseActive = true;
            transit<onLoiter>();
         };
        
        virtual void react(Pause2MissionEvent                const & e) override { 
            //drone must land and disarm on its current position
            pauseActive = true;
            transit<onLand>();
         };

        void react(TickEvent const & e) override { 
            onMission::react(e);
            if (mri->processWaypoint(destX, destY, destZ)) {
                // Waypoint on going 
                mri->sendMissionOrderVelocity(MASK_POSITION,destX, destY, destZ);
            } else {
                float prev_destMode = destMode;
                // We have not reached our next stop, let's pop the next WayPoint and send it right now ot autopilot
                if (mri->getNextWaypoint(destX, destY, destZ, destMode)) {
                    // New waypoint
                    if (prev_destMode == 1.0) {
                        transit<onFreestyle>();
                    } else {
                        mri->sendMissionOrderVelocity(MASK_POSITION,destX, destY, destZ);
                    }
                } else {
                    // End of mission  
                    ROS_INFO("state %s : no new WayPoint",getStateName());
                    transit<onLand>();
                }
            }
        };
};

class onFreestyle : public onMission {
    public:
        onFreestyle() : onMission("onFreestyle") {};

    private:

        void entry() override {
            //Front flip by default
            onMission::entry();
            mri->doFrontflip();
            transit<onGoing>();
        };

};

class onLoiter : public onMission {
    public:
        onLoiter() : onMission("onLoiter") {};

    private:
        float curdestX;
        float curdestY;
        float curdestZ;

        void entry(void) override {
            onMission::entry();
            mri->getCurrentPosition(curdestX, curdestY, curdestZ);
            mri->sendMissionOrder(MASK_POSITION,curdestX, curdestY, curdestZ);
        };

        void react(DisconnectedEvent const & e) override { 
            onMission::react(e);
            transit<onWaitConnect>();
        };

        virtual void react(EnteringOffboardEvent          const & e) override { 
            // Just ignore
        };
        virtual void react(LeavingOffboardEvent           const & e) override { 
            onMission::react(e);
            transit<onWaitConnect>();
        };
        virtual void react(ArmedEvent                     const & e) override { 
            // Just ignore
         };
        virtual void react(DisarmedEvent                  const & e) override { 
            onMission::react(e);
            transit<onWaitMission>();
         };
        virtual void react(NewMissionEvent                const & e) override { 
            onMission::react(e);
         };

        virtual void react (LocalPositionAcquiredEvent    const & e) override {

        };

        virtual void react(Pause1MissionEvent                const & e) override { 
            //Ignore
         };
        
        virtual void react(Pause2MissionEvent                const & e) override { 
            //drone must land on its current position
            pauseActive = true;
            transit<onLand>();
         };

         virtual void react(PauseBackMissionEvent                const & e) override { 
            pauseActive = false;
            transit<onGoing>();
         };

        void react(TickEvent const & e) override { 
            onMission::react(e);
            mri->sendMissionOrder(MASK_POSITION,curdestX, curdestY, curdestZ);
        };
};


class onLand : public onMission {
    public:
        onLand() : onMission("onLand") {};

    private:
        float curdestX;
        float curdestY;
        float curdestZ;

        void entry(void) override {
            onMission::entry();
            LandedVehicleCount=0;
            mri->getCurrentPosition(curdestX, curdestY, curdestZ);
            mri->sendMissionOrderVelocity(MASK_POSITION, curdestX, curdestY, curdestZ, true);
            
        };

        virtual void react(ConnectedEvent                 const & e) override { 
            // Just ignore
         };

        void react(DisconnectedEvent const & e) override { 
            onMission::react(e);
            transit<onWaitConnect>();
        };

        virtual void react(ArmedEvent                     const & e) override { 
            // Just ignore
         };

        virtual void react(DisarmedEvent                  const & e) override { 
            if (!mri->dry_run ) {
                onMission::react(e);
                transit<onWaitMission>();
            }
         };
        virtual void react(NewMissionEvent                const & e) override { 
            onMission::react(e);
         };

        virtual void react (LocalPositionAcquiredEvent    const & e) override {
             onMission::react(e);
        };

        void react(TickEvent const & e) override { 
            onMission::react(e);

            if (!mri->isVehicleLanded()) {
                //landing in progress
                mri->sendMissionOrderVelocity(MASK_POSITION, curdestX, curdestY, curdestZ, true);
            } else {
                // vehicule landed
                ROS_INFO("state %s : vehicle landed",getStateName());
                transit<onDisarm>();
                }
        };
};

class onDisarm
: public onMission
{
    public:
        onDisarm() : onMission("onDisarm") {};

    private:
        ros::Time last_request;

        void entry(void) override {
            onMission::entry();
            if (!mri->dry_run ) {
                mri->disarmVehicule();
                last_request = ros::Time::now();
            }
            else
            {
                //dry run mode
                ROS_INFO("dry_run active !");
                transit<onStopOffBoard>();     
            }
            
        };

        void react(DisarmedEvent const & e) override { 
            onMission::react(e);
            transit<onStopOffBoard>();            
        };

        void react(EnteringOffboardEvent const & e) override { 
            // ignore
        };

        void react(LeavingOffboardEvent const & e) override { 
            MissionStateMachine::react(e);
            transit<onWaitMission>();
        };

        void react(TickEvent const & e) override { 
            onMission::react(e);
            if(ros::Time::now() - last_request > ros::Duration(5.0))
            {
                mri->disarmVehicule();
                last_request = ros::Time::now();
            }
        };

};

FSM_INITIAL_STATE(MissionStateMachine, onWaitConnect)



void MissionRosInterface::state_cb(const mavros_msgs::State::ConstPtr& msg){

    //ROS_INFO("state_cb msg.mode (%d %d %d %s)", msg->connected, msg->armed, msg->guided, msg->mode.c_str());
    std::string offboard("OFFBOARD");

    current_state = *msg;
    if (current_state.connected) {
        send_event(ConnectedEvent());
    } else {
        send_event(DisconnectedEvent());
    }
    if (current_state.armed) {
        send_event(ArmedEvent());
    } else {
        send_event(DisarmedEvent());
    }
    if (current_state.mode != offboard) {
        send_event(LeavingOffboardEvent());        
    } else {
        send_event(EnteringOffboardEvent());        
    }
}

void MissionRosInterface::rcin_cb(const mavros_msgs::RCIn::ConstPtr& rcin_msg){
    mavros_msgs::RCIn previous_rcin = current_rcin;
    current_rcin = *rcin_msg;
    if ( previous_rcin.channels.size() == 0 ) { //first time this method is called
       if ( current_rcin.channels[rcin_mode_channel-1] < 1300 ) {
            send_event(RecordMissionModeEvent());
        }
        else
        {
            if ( current_rcin.channels[rcin_mode_channel-1] < 1600 ) {
                send_event(WaitMissionModeEvent());
            } else {
                send_event(ExecuteMissionRecordedEvent());
            }
        } 
        if ( current_rcin.channels[rcin_pause_mission_channel-1] < 1300 ) {
            send_event(PauseBackMissionEvent());
        }
        else
        {
            if ( current_rcin.channels[rcin_pause_mission_channel-1] < 1600 ) {
                send_event(Pause1MissionEvent());
            } else {
                send_event(Pause2MissionEvent());
            }
        } 
        if ( current_rcin.channels[rcin_switchMode_channel-1] > 1300 ) {
            if (offBoardMode) {
                offBoardMode=false;
            } else {
                offBoardMode=true;
            }
            ROS_INFO("rcin_cb - switchMode (offboard/flightmode) detected - offBoardMode = %s", offBoardMode ? "true" : "false");
            send_event(SwitchModeEvent());
        }
        if ( current_rcin.channels[rcin_flightMode_channel-1] < 1300 ) {
            RCFlightMode="ALTCTL";
        } 
        else
        {
            if ( current_rcin.channels[rcin_flightMode_channel-1] < 1600 ) {
                RCFlightMode="STABILIZED";
            } else {
                RCFlightMode="ACRO";
            }
        }
        return;
    }
    if ( current_rcin.channels[rcin_record_position_channel-1] != previous_rcin.channels[rcin_record_position_channel-1] ) {
        // channel switch to record position
        ROS_INFO("rcin_cb - switch record detected - position recorded : x = %f, y = %f; z = %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
        send_event(RecordCurrentPositionEvent());
    }
    if ( current_rcin.channels[rcin_mode_channel-1] != previous_rcin.channels[rcin_mode_channel-1] ) {
        // channel switch mode
        ROS_INFO("rcin_cb - switch mode detected - value = %d", current_rcin.channels[rcin_mode_channel-1]);
        if ( current_rcin.channels[rcin_mode_channel-1] < 1300 ) {
            send_event(RecordMissionModeEvent());
        }
        else
        {
            if ( current_rcin.channels[rcin_mode_channel-1] < 1600 ) {
                send_event(WaitMissionModeEvent());
            } else {
                send_event(ExecuteMissionRecordedEvent());
            }
        }
    }
    if ( current_rcin.channels[rcin_pause_mission_channel-1] != previous_rcin.channels[rcin_pause_mission_channel-1] ) {
        // channel switch mode
        ROS_INFO("rcin_cb - switch pause_mission detected - value = %d", current_rcin.channels[rcin_pause_mission_channel-1]);
        if ( current_rcin.channels[rcin_pause_mission_channel-1] < 1300 ) {
            send_event(PauseBackMissionEvent());
        }
        else
        {
            if ( current_rcin.channels[rcin_pause_mission_channel-1] < 1600 ) {
                send_event(Pause1MissionEvent());
            } else {
                send_event(Pause2MissionEvent());
            }
        }
    }
    if ( current_rcin.channels[rcin_switchMode_channel-1] != previous_rcin.channels[rcin_switchMode_channel-1] ) {
        // channel switch mode
        if ( current_rcin.channels[rcin_switchMode_channel-1] > 1300 ) {
            if (offBoardMode) {
                offBoardMode=false;
            } else {
                offBoardMode=true;
            }
            ROS_INFO("rcin_cb - switchMode (offboard/flightmode) detected - offBoardMode = %s", offBoardMode ? "true" : "false");
            send_event(SwitchModeEvent());
        }
    }
    if ( current_rcin.channels[rcin_flightMode_channel-1] != previous_rcin.channels[rcin_flightMode_channel-1] ) {
        if ( current_rcin.channels[rcin_flightMode_channel-1] < 1300 ) {
            RCFlightMode="POSITION";
        } 
        else
        {
            if ( current_rcin.channels[rcin_flightMode_channel-1] < 1600 ) {
                RCFlightMode="STABILIZED";
            } else {
                RCFlightMode="ACCRO";
            }
        }
    }
}

void MissionRosInterface::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg){
    current_pose = *pose_msg;
    send_event(LocalPositionAcquiredEvent(pose_msg));        
}

void MissionRosInterface::velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& velocity_msg)
{
    float x_vel_2 = pow(velocity_msg->twist.linear.x,2.0);
    float y_vel_2 = pow(velocity_msg->twist.linear.y,2.0);
    float z_vel_2 = pow(velocity_msg->twist.linear.z,2.0);
    current_vel = sqrt(x_vel_2+y_vel_2+z_vel_2);
    current_velocity = *velocity_msg;
}

void MissionRosInterface::vfr_hud_cb (const mavros_msgs::VFR_HUD::ConstPtr& vfr_hud_msg)
{
     current_vfr_hud = *vfr_hud_msg;
}

void MissionRosInterface::mission_cb(const setpoint_leader::robocars_mission::ConstPtr& mission_msg){
    
    setpoint_leader::robocars_mission current_mission;
    current_mission = *mission_msg;
    //current_mission_cpy = current_mission;
    // two first waypoints are departure and arrival.

    // If mission is empty, cancel ongoing mission.
    if (current_mission.path.size()==0) {
        ROS_INFO("MAVROS: Received CANCEL mission");
        send_event(CancelMissionEvent());
        return;
    }

    departurePos.pose.position.x = current_mission.departure.x_lat;
    departurePos.pose.position.y = current_mission.departure.y_long;
    departurePos.pose.position.z = current_mission.departure.z_alt;
    
    arrivalPos.pose.position.x = current_mission.arrival.x_lat;
    arrivalPos.pose.position.y = current_mission.arrival.y_long;
    arrivalPos.pose.position.z = current_mission.arrival.z_alt;

    ROS_INFO("MAVROS: Received new mission with %ld waypoints", current_mission.path.size());
    ROS_INFO("MAVROS: Pickup position : %f, %f", departurePos.pose.position.x, departurePos.pose.position.y);
    ROS_INFO("MAVROS: Delivery position : %f, %f", arrivalPos.pose.position.x, arrivalPos.pose.position.y);
    for (int i=0; i<current_mission.path.size();i++) {
        waypoints.push_back(current_mission.path[i]);
    }
    send_event(NewMissionEvent());
}

void MissionRosInterface::startOffBoardMode (void) {
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ROS_INFO("MAVROS: Request OFFBOARD mode");
    int res = mri->set_mode_client.call(offb_set_mode);
    ROS_INFO("OFFBOARD request sent (%d)", res);
}

void MissionRosInterface::stopOffBoardMode (string mode) {
    offb_set_mode.request.custom_mode = mode;
    ROS_INFO("MAVROS: Request %s mode", mode.c_str());
    int res = mri->set_mode_client.call(offb_set_mode);
    ROS_INFO("%s mode request sent (%d)", mode.c_str(), res);
}

void MissionRosInterface::startManualMode (void) {
    offb_set_mode.request.custom_mode = "";
    offb_set_mode.request.base_mode = mavros_msgs::SetModeRequest::MAV_MODE_MANUAL_DISARMED;
    ROS_INFO("MAVROS: Request MANUAL mode");
    int res = mri->set_mode_client.call(offb_set_mode);
//    ROS_INFO("OFFBOARD request sent (%d)", res);
}

void MissionRosInterface::armVehicule (void) {
    arm_cmd.request.value = true;
    ROS_INFO("MAVROS: Arming Vehicule");
    int res = mri->arming_client.call(arm_cmd);
    ROS_INFO("Aming request sent (%d)", res);
}

void MissionRosInterface::disarmVehicule (void) {
    arm_cmd.request.value = false;
    ROS_INFO("MAVROS: Disarming Vehicule");
    int res = arming_client.call(arm_cmd);
    ROS_INFO("Disaming request sent (%d)", res);
}

void MissionRosInterface::checkOffBoardResponse (void) {
    ROS_INFO("MAVROS: OFFBOARD response (%d)", offb_set_mode.response.mode_sent);
}

void MissionRosInterface::checkCurrentState (void) {
    ROS_INFO("MAVROS: current_state.mode (%s)", current_state.mode.c_str());
}

void MissionRosInterface::checkArmingResponse (void) {
    ROS_INFO("MAVROS: current_state.mode (%d)", arm_cmd.response.success);
}

void MissionRosInterface::sendIdlePosition (float destX, float destY, float destZ) {

    mavros_msgs::PositionTarget rawMsg;

    // Feed mavros_msgs::PositionTarget message
    rawMsg.header.stamp = ros::Time::now();
    rawMsg.header.seq=pos_seq;
    rawMsg.header.frame_id = "enu_world";

    rawMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // rawMsg.type_mask = MASK_IDLE_POSITION;
    // rawMsg.type_mask = MASK_POSITION_VELOCITY;
    rawMsg.type_mask = MASK_IDLE_POSITION;
    
    //switchMode = !switchMode;

    rawMsg.position.x = 0.0f; // destX;
    rawMsg.position.y = 0.0f; // destY;
    rawMsg.position.z = 0.0f; // destZ;
    rawMsg.velocity.x = 0.0f;
    rawMsg.velocity.y = 0.0f;
    rawMsg.velocity.z = 0.0f;
    rawMsg.acceleration_or_force.x = 0.0f;
    rawMsg.acceleration_or_force.y = 0.0f;
    rawMsg.acceleration_or_force.z = 0.0f;
    rawMsg.yaw = getDestYawAngle(0.0, 0.0);
    rawMsg.yaw_rate = 0.5f;

    //ROS_INFO("sendMissionPosition: Request IDLE");

    // Publish message
    local_raw_pub.publish(rawMsg);

    pos_seq++;
    //sendBrakeVelocity();
        
}

void MissionRosInterface::sendMissionPosition (float destX, float destY, float destZ) {

    mavros_msgs::PositionTarget rawMsg;
    rawMsg.header.stamp = ros::Time::now();
    rawMsg.header.seq=pos_seq;
    rawMsg.header.frame_id = "enu_world";
    rawMsg.type_mask = MASK_POSITION;

    rawMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    rawMsg.position.x = destX;
    rawMsg.position.y = destY;

    //rawMsg.position.x = 50.0f;
    //rawMsg.position.y = 50.0f;

    rawMsg.position.z = destZ;
    rawMsg.velocity.x = 0.0f;
    rawMsg.velocity.y = 0.0f;
    rawMsg.velocity.z = 0.0f;
    rawMsg.acceleration_or_force.x = 0.0f;
    rawMsg.acceleration_or_force.y = 0.0f;
    rawMsg.acceleration_or_force.z = 0.0f;
    rawMsg.yaw = getDestYawAngle(destX, destY);
    rawMsg.yaw_rate = 0.05f;

    //ROS_INFO("sendMissionPosition: Request POSITION %f %f %08x", destX, destY, rawMsg.type_mask);

    // Publish message
    //if (motion_mode){
    local_raw_pub.publish(rawMsg);

    pos_seq++;
}


void MissionRosInterface::sendVelocityOrder( float vX, float vY, float vZ) {

    mavros_msgs::PositionTarget rawMsg;

    rawMsg.header.stamp = ros::Time::now();
    rawMsg.header.seq=pos_seq;
    rawMsg.header.frame_id = "enu_world";
    // rawMsg.type_mask = MASK_VELOCITY;
    rawMsg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
	               mavros_msgs::PositionTarget::IGNORE_PY |
		       mavros_msgs::PositionTarget::IGNORE_PZ |  
		       mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ |
                       mavros_msgs::PositionTarget::FORCE;

    rawMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    //rawMsg.position.x = 0.0f;
    //rawMsg.position.y = 0.0f;
    //rawMsg.position.z = 0.0f;
    rawMsg.velocity.x = vX;
    rawMsg.velocity.y = vY;
    rawMsg.velocity.z = vZ;
    //rawMsg.acceleration_or_force.x = NAN;
    //rawMsg.acceleration_or_force.y = NAN;
    //rawMsg.acceleration_or_force.z = NAN;
    rawMsg.yaw = 1.5708f;
    rawMsg.yaw_rate = 0.05f;

    //ROS_INFO("sendMissionPosition: Request POSITION %f %f %08x", destX, destY, rawMsg.type_mask);

    // Publish message
    local_raw_pub.publish(rawMsg);

    pos_seq++;
}

void MissionRosInterface::sendMissionOrderVelocity(TypeMasks action, float destX, float destY, float destZ, bool landingMode) {

    mavros_msgs::PositionTarget rawMsg;

    rawMsg.header.stamp = ros::Time::now();
    rawMsg.header.seq=pos_seq;
    rawMsg.header.frame_id = "enu_world";
    float Vx, Vy, Vz;

    float vel=refVelocity;

    float deltaX = destX-current_pose.pose.position.x;
    float deltaY = destY-current_pose.pose.position.y;
    float deltaZ = destZ-current_pose.pose.position.z;
    float distance = sqrt(pow(deltaX,2)+pow(deltaY,2)+pow(deltaZ,2));
    float distanceXY = sqrt(pow(deltaX,2)+pow(deltaY,2));

    /*
    //calcul de l'angle entre la direction du drone et la direction voulue
    float a_drone = deltaX*current_velocity.twist.linear.x + deltaY*current_velocity.twist.linear.y + deltaZ*current_velocity.twist.linear.z;
    float b_drone = sqrt((pow(deltaX,2)+pow(deltaY,2)+pow(deltaZ,2))*(pow(current_velocity.twist.linear.x,2)+pow(current_velocity.twist.linear.y,2)+pow(current_velocity.twist.linear.z,2)));
    float angle=0;
    if (b_drone != 0.0) {
        angle = acos(a_drone/b_drone);
    }

    float adjust = (distance / dst_vel_treshold) * cos(angle);
    */
    float adjust = distance / dst_vel_treshold;
    if ( adjust > 1) {
            adjust=1.0+(adjust/10);
    }
    if (adjust < 1) {
        adjust=1;
    }
    vel=adjust*vel;

    /*if (distance < dst_vel_treshold and angle > 3.14/4) {
        vel=vel*distance/(dst_vel_treshold);
        ROS_INFO("vel=%f", vel);
    }*/
    Vx = vel*deltaX/distance;
    Vy = vel*deltaY/distance;
    if ( landingMode ) {
        Vz = landing_velocity;
    }
    else
    {
        Vz = vel*deltaZ/(distance);
    }
    
    
    /*
    ROS_INFO("---------------");
    ROS_INFO(" currX=%f, currY=%f, currZ=%f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    ROS_INFO(" destX=%f, destY=%f, destZ=%f", destX, destY, destZ);
    ROS_INFO("vel.x=%f, vel.y=%f, vel.z=%f", current_velocity.twist.linear.x, current_velocity.twist.linear.y, current_velocity.twist.linear.z);
    ROS_INFO("deltax=%f, deltaY=%f, deltaZ=%f", deltaX, deltaY, deltaZ);
    ROS_INFO("a_drone=%f, b_drone=%f, angle=%f, vel=%f", a_drone, b_drone, angle, vel);
    ROS_INFO("Vx=%f, Vy=%f, Vz=%f", Vx, Vy, Vz);
    */
    rawMsg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
                       mavros_msgs::PositionTarget::IGNORE_PY |
                       mavros_msgs::PositionTarget::IGNORE_PZ |
                       mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ ;
                       //mavros_msgs::PositionTarget::IGNORE_YAW |
		       //mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    rawMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    //ROS_INFO("deltaX=%f, deltaY=%f, deltaZ=%f, distance=%f, Vx=%f, Vy=%f, Vz=%f", deltaX, deltaY, deltaZ, distance, Vx, Vy, Vz);

    /*rawMsg.position.x = destX;
    rawMsg.position.y = destY;
    rawMsg.position.z = destZ;*/
    rawMsg.velocity.x = Vx;
    rawMsg.velocity.y = Vy;
    rawMsg.velocity.z = Vz;
    /*rawMsg.acceleration_or_force.x = 0.0;
    rawMsg.acceleration_or_force.y = 0.0;
    rawMsg.acceleration_or_force.z = 0.0;*/
    // set yaw toward destination
    if ( landingMode ) {
        rawMsg.yaw = getCurrentYaw();
        rawMsg.yaw_rate = 0.00f;
    }
    else
    {
        rawMsg.yaw = getDestYawAngle(destX, destY);
        rawMsg.yaw_rate = 0.05f;
    }
    

    //ROS_INFO("sendMissionPosition: Request POSITION %f %f %08x", destX, destY, rawMsg.type_mask);

    // Publish message
    local_raw_pub.publish(rawMsg);

    pos_seq++;
}

void MissionRosInterface::accZ(float z) {
        //ROS_INFO("curZ:%f; current_pose.pose.position.z:%f", curZ, current_pose.pose.position.z);
        mavros_msgs::PositionTarget rawMsg;
        rawMsg.header.stamp = ros::Time::now();
        rawMsg.header.seq=pos_seq;
        rawMsg.header.frame_id = "enu_world";
        float Ax, Ay, Az;

        rawMsg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
                        mavros_msgs::PositionTarget::IGNORE_PY |
                        mavros_msgs::PositionTarget::IGNORE_PZ |
                        mavros_msgs::PositionTarget::IGNORE_VX | 
                        mavros_msgs::PositionTarget::IGNORE_VY |
                        mavros_msgs::PositionTarget::IGNORE_VZ ;
                        //mavros_msgs::PositionTarget::IGNORE_YAW |
                //mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        rawMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        rawMsg.acceleration_or_force.x=0;
        rawMsg.acceleration_or_force.y=0;
        rawMsg.acceleration_or_force.z=z;
        local_raw_pub.publish(rawMsg);
        pos_seq++;
}

void MissionRosInterface::doFrontflip(void) {
    float curX = current_pose.pose.position.x;
    float curY = current_pose.pose.position.y;
    float curZ = current_pose.pose.position.z;

    //Accelerate verticaly
    while (current_pose.pose.position.z < curZ + 0.3f) {
        mri->accZ(40);
        ros::spinOnce();
    }

    //pitch acceleration for front flipping
    ROS_INFO("Pitching...");
    bool pitch=true;
    bool reverse=false;
    float trust=1.0;
    while (pitch) {
        EulerAngles curAngle = mri->getCurrentEulerAngles();
        ROS_INFO("curAngle.pitch:%f; curAngle.roll:%f", curAngle.pitch, curAngle.roll);
        if ( ! reverse ) {
            if ( abs(curAngle.pitch) > M_PI*0.9f || abs(curAngle.roll) > M_PI*0.9f) {
                reverse = true;
                trust=0.1;
            }
        }
        if ( reverse and (abs(curAngle.pitch) < M_PI*0.1 || abs(curAngle.roll) < M_PI*0.1)) {
            pitch=false;
        }
        mavros_msgs::AttitudeTarget rawMsg;
        rawMsg.header.stamp = ros::Time::now();
        rawMsg.header.seq=pos_seq;
        rawMsg.header.frame_id = "enu_world";

        rawMsg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE ;
                           //mavros_msgs::AttitudeTarget::IGNORE_THRUST ;

        rawMsg.body_rate.x=0;
        rawMsg.body_rate.y=30;
        rawMsg.body_rate.z=0;
        rawMsg.thrust=trust;
        attitude_raw_pub.publish(rawMsg);

        pos_seq++;
        ros::spinOnce();
    }
    mavros_msgs::AttitudeTarget rawMsg;
        rawMsg.header.stamp = ros::Time::now();
        rawMsg.header.seq=pos_seq;
        rawMsg.header.frame_id = "enu_world";

        rawMsg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE ;
                           //mavros_msgs::AttitudeTarget::IGNORE_THRUST ;

        rawMsg.body_rate.x=0;
        rawMsg.body_rate.y=1;
        rawMsg.body_rate.z=0;
        rawMsg.thrust=0.6;
        attitude_raw_pub.publish(rawMsg);

        pos_seq++;
        ros::spinOnce();

}

void MissionRosInterface::sendMissionOrder(TypeMasks action, float destX, float destY, float destZ) {

    mavros_msgs::PositionTarget rawMsg;

    rawMsg.header.stamp = ros::Time::now();
    rawMsg.header.seq=pos_seq;
    rawMsg.header.frame_id = "enu_world";
    //rawMsg.type_mask = action;
    rawMsg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                       mavros_msgs::PositionTarget::IGNORE_VY |
                       mavros_msgs::PositionTarget::IGNORE_VZ |
                       mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ ;
                       //mavros_msgs::PositionTarget::IGNORE_YAW |
		       //mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    rawMsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    rawMsg.position.x = destX;
    rawMsg.position.y = destY;
    rawMsg.position.z = destZ;
    //rawMsg.velocity.x = NAN;
    //rawMsg.velocity.y = NAN;
    //rawMsg.velocity.z = NAN;
    //rawMsg.acceleration_or_force.x = NAN;
    //rawMsg.acceleration_or_force.y = NAN;
    //rawMsg.acceleration_or_force.z = NAN;
    rawMsg.yaw = 1.5708f;
    rawMsg.yaw_rate = 0.1f;

    //ROS_INFO("sendMissionPosition: Request POSITION %f %f %08x", destX, destY, rawMsg.type_mask);

    // Publish message
    local_raw_pub.publish(rawMsg);

    pos_seq++;
}


void MissionRosInterface::sendRobocarsAttitude () {
    setpoint_leader::robocars_attitude att_msg;
    att_msg.pose.position.x = current_pose.pose.position.x;
    att_msg.pose.position.y = current_pose.pose.position.y;
    att_msg.pose.position.z = current_pose.pose.position.z;
    att_msg.pose.orientation.x = current_pose.pose.orientation.x;
    att_msg.pose.orientation.y = current_pose.pose.orientation.y;
    att_msg.pose.orientation.z = current_pose.pose.orientation.z;
    att_msg.pose.orientation.w = current_pose.pose.orientation.w;
    att_msg.velocity = current_vel;
    att_msg.heading = current_vfr_hud.heading;
    robocars_attitude_pub.publish (att_msg);
}

void MissionRosInterface::sendDebug (float destX, float destY, float destZ, float dst) {
    setpoint_leader::robocars_debug debug_msg;
    ros::Time tnow=ros::Time::now();
    ros::Time tpose(current_pose.header.stamp.sec, current_pose.header.stamp.nsec);
    ros::Duration tdelta=tnow-tpose;
    debug_msg.header.stamp.sec = tnow.sec; 
    debug_msg.header.stamp.nsec = tnow.nsec; 
    debug_msg.pose_seq = current_pose.header.seq;
    debug_msg.pose_stamp.sec = tpose.sec;
    debug_msg.pose_stamp.nsec = tpose.nsec;
    debug_msg.delta_stamp.sec = tdelta.sec;
    debug_msg.delta_stamp.nsec = tdelta.nsec;
    debug_msg.cx=current_pose.pose.position.x;
    debug_msg.cy=current_pose.pose.position.y;
    debug_msg.cz=current_pose.pose.position.z;
    debug_msg.dx=destX;
    debug_msg.dy=destY;
    debug_msg.dz=destZ;
    debug_msg.dst=dst;
    events_pub.publish (debug_msg);
}

void MissionRosInterface::printCurrentPosition() {
    ROS_INFO("Current Position : x = %f, y = %f, z = %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
}

void MissionRosInterface::getCurrentPosition(float & destX, float & destY, float & destZ) {
    destX = current_pose.pose.position.x;
    destY = current_pose.pose.position.y;
    destZ = current_pose.pose.position.z;
}

MissionRosInterface::EulerAngles MissionRosInterface::getCurrentEulerAngles() {
    Quaternion qua;
    qua.x = current_pose.pose.orientation.x;
    qua.y = current_pose.pose.orientation.y;
    qua.z = current_pose.pose.orientation.z;
    qua.w = current_pose.pose.orientation.w;
    return ToEulerAngles(qua);
} 

double MissionRosInterface::getCurrentYaw() {

    EulerAngles ea;
    ea = mri->getCurrentEulerAngles();
    return ea.yaw;
}

float MissionRosInterface::getDestYawAngle(float destX, float destY) {
    float Y = destY - current_pose.pose.position.y;
    float X = destX - current_pose.pose.position.x;
    float YAW = getCurrentYaw();
    float PI = 3.14159265;
    float tmp_res = 0;
    float res = 0;

    //ROS_INFO("getDestYawAngle - X = %f; Y= = %f; YAW = %f", X, Y, YAW);
    if ( sqrt(pow(Y, 2.0) + pow(X, 2.0)) < dst_yaw_thresh ) {
        //ROS_INFO("getDestYawAngle - < dst_yaw_thresh, return %f ", YAW);
        return YAW;
    }
    if ( X == 0 ) {
        if ( Y > 0 ) {
            tmp_res = PI/2;
        }
        else
        {
            tmp_res = -PI/2;
        }
        //ROS_INFO("getDestYawAngle - X ==0, return %f", tmp_res);
        return tmp_res;
    }
    tmp_res = atan(abs(Y/X));
    res = tmp_res;
    if ( X < 0 and Y < 0 ) {
        res = - (PI - tmp_res);
    }
    if ( X < 0 and Y > 0 ) {
        res = PI - tmp_res;
    }
    if ( X > 0 and Y < 0 ) {
        res = -tmp_res;
    }
    //ROS_INFO("getDestYawAngle - return %f", res);
    return res;
}

float MissionRosInterface::getDestinationdistance(float ros_px, float ros_py, float ros_pz, float x, float y, float z) {
    float dist_to_target_x_2 = pow((current_pose.pose.position.x - ros_px),2.0);
    float dist_to_target_y_2 = pow((current_pose.pose.position.y - ros_py),2.0);
    float dist_to_target_z_2 = pow((current_pose.pose.position.z - ros_pz),2.0);
    float dist = sqrt(dist_to_target_x_2+dist_to_target_y_2+dist_to_target_z_2);
    return dist;
}

int MissionRosInterface::isDestinationReachedInXSecond(float second, float ros_px, float ros_py, float ros_pz) {
    
    float x_projected = current_pose.pose.position.x; //+ current_velocity.twist.linear.x*second;
    float y_projected = current_pose.pose.position.x; //+ current_velocity.twist.linear.y*second;
    float z_projected = current_pose.pose.position.x; //+ current_velocity.twist.linear.z*second;

    if ( isDestinationReached(ros_px, ros_py, ros_pz, x_projected, y_projected, z_projected)) {
        return 1;    
    }
    else
    {
        return 0;
    }
}


int MissionRosInterface::isDestinationReached(float ros_px, float ros_py, float ros_pz, float x=0, float y=0, float z=0) {
    if (x == 0 and y == 0 and z == 0) {
        x = current_pose.pose.position.x;
        y = current_pose.pose.position.y;
        z = current_pose.pose.position.z;
    }

    float dist = getDestinationdistance (ros_px, ros_py, ros_pz, x, y, z);
    //ROS_INFO("isDestinationReached: distance = %f", dist);

    if ( dist < dst_thresh ){
        return 1;
    } else {
        return 0;
    }
}


int MissionRosInterface::isVehicleLanded() {
    //if vehicle has a z velocity near to 0 for a long time (X iterations) while in landing state, it's considered as landed
    float curVelZ = current_velocity.twist.linear.z;

    if ( abs(curVelZ) < 0.1f ) {
        LandedVehicleCount++;
        //ROS_INFO("isVehicleLanded : increment LandedVehicleCount = %d", LandedVehicleCount);
    } else {
        //ROS_INFO("isVehicleLanded : curVelZ = %f", curVelZ);
        LandedVehicleCount=0;
    }
    if ( LandedVehicleCount > 100 ) {
        return 1;
    }
    else
    {
        return 0;
    }

}

int MissionRosInterface::isDepartureReached() {
    int ret = isDestinationReached (departurePos.pose.position.x, departurePos.pose.position.y, departurePos.pose.position.z);
    //ROS_INFO("isDepartureReached : ret = %d; departure : x= %f, y=%f, z=%f; current : x=%f, y=%f, z=%f", ret, departurePos.pose.position.x, departurePos.pose.position.y, departurePos.pose.position.z, current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    return ret;
}

int MissionRosInterface::getNextWaypoint (float & destX, float & destY, float & destZ, float & destMode) {

  if (waypoints.size()>0){
      mavros_msgs::Waypoint wp = waypoints.front();

      destX = wp.x_lat;
      destY = wp.y_long;
      destZ = wp.z_alt;
      destMode = wp.param1;

      waypoints.erase(waypoints.begin());
      ROS_INFO("getNextWaypoint : next waypoint is x=%f y=%f z=%f, mode=%f",destX, destY, destZ, destMode);
      ROS_INFO("getNextWaypoint : %ld waypoints remaining",waypoints.size());
      return 1;
  }    
  return 0;
}

int MissionRosInterface::processWaypoint (float & destX, float & destY, float & destZ) {

    if (isDestinationReachedInXSecond (0.5, destX, destY, destZ)) {
        // Waypoint done
        return 0;            
    }
    // Waypoint in going
    return 1;
}

void MissionRosInterface::clearMission (void) {
    waypoints.clear();
}

void MissionRosInterface::recordCurrentPosition(void) {
    float X = current_pose.pose.position.x;
    float Y = current_pose.pose.position.y;
    float Z = current_pose.pose.position.z;

    mavros_msgs::Waypoint wp;
    wp.frame=4;
    wp.command=16;
    wp.is_current=true;
    wp.autocontinue=true;
    wp.param1=0.0;
    wp.param2=0.0;
    wp.param3=0.0;
    wp.param4=0.0;
    wp.x_lat=X;
    wp.y_long=Y;
    wp.z_alt=Z;

    if ( recorded_mission.path.size() == 0 ) {
        // First position to record
        recorded_mission.departure = wp;
    }
    recorded_mission.path.push_back(wp);
    recorded_mission.arrival = recorded_mission.path.back();
}

void MissionRosInterface::saveRecordedMission(string filename) {
    auto waypoint2text = [](mavros_msgs::Waypoint wp, bool path) 
    {
        string txt="";
        if ( path ) {
            txt = "- frame: " + std::to_string(wp.frame) + "\n";
        }
        else
        {
            txt = "  frame: " + std::to_string(wp.frame) + "\n";
        }
        txt += "  command: " + std::to_string(wp.command) + "\n";
        string is_cur = wp.is_current ? "true" : "false";
        txt += "  is_current: " + is_cur + "\n";
        string autocontinue = wp.autocontinue ? "true" : "false";
        txt += "  autocontinue: " + autocontinue + "\n";
        txt += "  param1: " + std::to_string(wp.param1) + "\n";
        txt += "  param2: " + std::to_string(wp.param2) + "\n";
        txt += "  param3: " + std::to_string(wp.param3) + "\n";
        txt += "  param4: " + std::to_string(wp.param4) + "\n";
        txt += "  x_lat: " + std::to_string(wp.x_lat) + "\n";
        txt += "  y_long: " + std::to_string(wp.y_long) + "\n";
        txt += "  z_alt: " + std::to_string(wp.z_alt) + "\n";
        return txt;
    };

    if ( recorded_mission.path.size() > 0 ) {
        //mission recorded to save
        ofstream myfile;
        myfile.open(mission_recorded_filename);
        myfile << "departure:\n";
        myfile << waypoint2text(recorded_mission.departure, false);
        myfile << "arrival:\n";
        myfile << waypoint2text(recorded_mission.arrival, false);
        myfile << "path:\n";
        for (int i=0; i<recorded_mission.path.size(); i++) {
            myfile << waypoint2text(recorded_mission.path[i], true);
        }
        myfile.close();
    }
}

void MissionRosInterface::clearRecordedMission (void) {
    recorded_waypoints.clear();
    recorded_mission.path.clear();
}

int MissionRosInterface::loadMissionRecorded (int lap_nb) {
    string cmd = mission_script + " " + mission_recorded_filename + " " + to_string(lap_nb) + "&";
    return system(cmd.c_str());
}


int main(int argc, char **argv)
{
    int loopCnt=0;
    ros::init(argc, argv, "offb_fsm_raw_node");

    mri = new MissionRosInterface;

    fsm_list::start();

    // wait for FCU connection
    ros::Rate rate(50.0);
    while(ros::ok()){
        ros::spinOnce();
        //ros::spin();
        if ((++loopCnt)%100 == 0) {
            mri->updateParam();
        }
        send_event (TickEvent());
        rate.sleep();
    }
}
