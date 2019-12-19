#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/VFR_HUD.h>
#include "std_msgs/String.h"
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>

#include <setpoint_leader/robocars_attitude.h>
#include <setpoint_leader/robocars_mission.h>
#include <setpoint_leader/robocars_debug.h>
#include <setpoint_leader/robocars_vehicle_status.h>


struct BaseEvent : tinyfsm::Event
{
    public:
        BaseEvent(const char * evtName) : _evtName(evtName) {};
        const char * getEvtName() const { return _evtName; };
    private:
        const char *  _evtName;
};

struct TickEvent                    : BaseEvent { public: TickEvent() : BaseEvent("TickEvent") {}; };
struct ConnectedEvent               : BaseEvent { public: ConnectedEvent() : BaseEvent("ConnectedEvent") {}; };
struct EnteringOffboardEvent        : BaseEvent { public: EnteringOffboardEvent() : BaseEvent("EnteringOffboardEvent") {}; };
struct RecordCurrentPositionEvent   : BaseEvent { public: RecordCurrentPositionEvent() : BaseEvent("RecordCurrentPositionEvent") {}; };
struct RecordMissionModeEvent       : BaseEvent { public: RecordMissionModeEvent() : BaseEvent("RecordMissionModeEvent") {}; };
struct Pause1MissionEvent           : BaseEvent { public: Pause1MissionEvent() : BaseEvent("Pause1MissionEvent") {}; };
struct PauseBackMissionEvent        : BaseEvent { public: PauseBackMissionEvent() : BaseEvent("PauseBackMissionEvent") {}; };
struct Pause2MissionEvent           : BaseEvent { public: Pause2MissionEvent() : BaseEvent("Pause2MissionEvent") {}; };
struct WaitMissionModeEvent         : BaseEvent { public: WaitMissionModeEvent() : BaseEvent("WaitMissionModeEvent") {}; };
struct LeavingOffboardEvent         : BaseEvent { public: LeavingOffboardEvent() : BaseEvent("LeavingOffboardEvent") {}; };
struct ArmedEvent                   : BaseEvent { public: ArmedEvent() : BaseEvent("ArmedEvent") {}; };
struct DisarmedEvent                : BaseEvent { public: DisarmedEvent() : BaseEvent("DisarmedEvent") {}; };
struct DisconnectedEvent            : BaseEvent { public: DisconnectedEvent() : BaseEvent("DisconnectedEvent") {}; };
struct InitSeedDoneEvent            : BaseEvent { public: InitSeedDoneEvent() : BaseEvent("InitSeedDoneEvent") {}; };
struct NewMissionEvent              : BaseEvent { public: NewMissionEvent() : BaseEvent("NewMissionEvent") {}; };
struct ExecuteMissionRecordedEvent  : BaseEvent { public: ExecuteMissionRecordedEvent() : BaseEvent("ExecuteMissionRecordedEvent") {}; };
struct CancelMissionEvent           : BaseEvent { public: CancelMissionEvent() : BaseEvent("CancelMissionEvent") {}; };
struct MaintenanceEnterManualEvent  : BaseEvent { public: MaintenanceEnterManualEvent() : BaseEvent("MaintenanceEnterManualEvent") {}; };
struct MaintenanceEnterAutoEvent    : BaseEvent { public: MaintenanceEnterAutoEvent() : BaseEvent("MaintenanceEnterAutoEvent") {}; };
struct LocalPositionAcquiredEvent   : BaseEvent { public: 
    LocalPositionAcquiredEvent(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) : pose_msg(*pose_msg), BaseEvent("LocalPositionAcquiredEvent") {};
    geometry_msgs::PoseStamped pose_msg; 
    };

class MissionStateMachine
: public tinyfsm::Fsm<MissionStateMachine>
{
    public:
        MissionStateMachine(const char * stateName) : _stateName(stateName), tinyfsm::Fsm<MissionStateMachine>::Fsm() { 
            ROS_INFO("MissionStateMachine: State created: %s", _stateName);      
        };
        const char * getStateName() const { return _stateName; };

    public:
        static float destX;
        static float destY;
        static float destZ;
        static float destMode;

    public:
        /* default reaction for unhandled events */
        void react(BaseEvent const & ev) { 
            ROS_INFO("state %s: unexpected event %s reveived", getStateName(), ev.getEvtName());      
        };

        virtual void react(TickEvent                      const & e) { /*logEvent(e);*/ };
        virtual void react(ConnectedEvent                 const & e) { logEvent(e); };
        virtual void react(DisconnectedEvent              const & e) { logEvent(e); };
        virtual void react(EnteringOffboardEvent          const & e) { logEvent(e); };
        virtual void react(RecordCurrentPositionEvent     const & e) { logEvent(e); };
        virtual void react(RecordMissionModeEvent         const & e) { logEvent(e); };
        virtual void react(Pause1MissionEvent             const & e) { logEvent(e); };
        virtual void react(Pause2MissionEvent             const & e) { logEvent(e); };
        virtual void react(PauseBackMissionEvent          const & e) { logEvent(e); };
        virtual void react(WaitMissionModeEvent           const & e) { logEvent(e); };
        virtual void react(LeavingOffboardEvent           const & e) { logEvent(e); };
        virtual void react(ArmedEvent                     const & e) { logEvent(e); };
        virtual void react(DisarmedEvent                  const & e) { logEvent(e); };
        virtual void react(NewMissionEvent                const & e) { logEvent(e); };
        virtual void react(ExecuteMissionRecordedEvent    const & e) { logEvent(e); };
        virtual void react(CancelMissionEvent             const & e) { logEvent(e); };
        virtual void react(MaintenanceEnterAutoEvent      const & e) { logEvent(e); };
        virtual void react(MaintenanceEnterManualEvent    const & e) { logEvent(e); };
        virtual void react(LocalPositionAcquiredEvent     const & e) { };

        virtual void entry(void) { 
            ROS_INFO("State %s: entering", getStateName()); 
        };  
        void         exit(void)  { };  /* no exit actions */

    private:
        const char *  _stateName ="NoName";
        void logEvent(BaseEvent const & e) {
            ROS_INFO("State %s: event %s", getStateName(), e.getEvtName());
        }
};

typedef tinyfsm::FsmList<MissionStateMachine> fsm_list;

template<typename E>
void send_event(E const & event)
{
  fsm_list::template dispatch<E>(event);
}

// Flight types
enum TypeMasks {
MASK_POSITION_VELOCITY =0b0000111111000000,
MASK_POSITION =         0b0000111111111000,
MASK_VELOCITY =         0b0000111111000111,
MASK_TAKEOFF_POSITION = 0b0001111111111000,
MASK_TAKEOFF_TO_Z =     0b0001111111111000,
MASK_TAKEOFF =          0b0001110111111111,
MASK_LAND_VELOCITY =    0b0010111111000111,
MASK_LAND_POSITION =    0b0010110111111000,
MASK_LAND =             0b0010110111111111,
MASK_LOITER_POSITION =  0b0100111111111000,
MASK_IDLE_POSITION =    0b1000110111111000
};


class MissionRosInterface
{
    public :
        MissionRosInterface() : pos_seq(1) {
            updateParam();
            state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, &MissionRosInterface::state_cb, this);
            rcin_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 1, &MissionRosInterface::rcin_cb, this);
            pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &MissionRosInterface::pose_cb, this);
            velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 1, &MissionRosInterface::velocity_cb, this);
            vfr_hud_sub = nh.subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud", 1, &MissionRosInterface::vfr_hud_cb, this);
            //mission_sub = nh.subscribe<mavros_msgs::WaypointList>("robocar/mission", 1, &MissionRosInterface::mission_cb, this);
            mission_sub = nh.subscribe<setpoint_leader::robocars_mission>("robocar/mission", 1, &MissionRosInterface::mission_cb, this);
            //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
            //raw
            // publication de la demande sur /mavros/setpoint_raw/local
            // https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/setpoint_raw.cpp
            // fera les conversion ENU - NED et publiera sur /mavros/setpoint_raw/target_local
            // qui sera envoyée sur mavlink au format set_position_target_local_ned
            local_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
            robocars_attitude_pub = nh.advertise<setpoint_leader::robocars_attitude>("/robocar/attitude", 10);
            robocars_at_pickup_pub = nh.advertise<setpoint_leader::robocars_vehicle_status>("/robocar/at_pickup", 10);
            robocars_at_dest_pub = nh.advertise<setpoint_leader::robocars_vehicle_status>("/robocar/at_dest", 10);
            robocars_on_duty_pub = nh.advertise<setpoint_leader::robocars_vehicle_status>("/robocar/on_duty", 10);
            events_pub = nh.advertise<setpoint_leader::robocars_debug>("robocar/debug", 10);

            // MAVLINK_MSG_ID_LOCAL_POSITION_NED can indeed be used in conjonction with
            // MAVLINK_MSG_ID_ATTITUDE to know whether it has completed the requested movement.
        };

        void updateParam() {
            float new_dst_thresh=1.0f;
            float new_refVelocity=1.0f;
            float new_dst_vel_treshold=1.0f;
            bool new_dry_run=false;
            string new_mission_recorded_filename="";
            string new_mission_script="";
            if ((nh.getParam("dst_thresh", new_dst_thresh)) && (new_dst_thresh != dst_thresh))
            {
                dst_thresh = new_dst_thresh;
                ROS_INFO("updateParam : new destination detection threshold set to %f",dst_thresh);
            }
            if ((nh.getParam("refVelocity", new_refVelocity)) && (new_refVelocity != refVelocity))
            {
                refVelocity = new_refVelocity;
                ROS_INFO("updateParam : new refVelocity detection threshold set to %f",refVelocity);
            }
            if ((nh.getParam("dst_vel_treshold", new_dst_vel_treshold)) && (new_dst_vel_treshold != dst_vel_treshold))
            {
                dst_vel_treshold = new_dst_vel_treshold;
                ROS_INFO("updateParam : new dst_vel_treshold  set to %f",dst_vel_treshold);
            }
            if ((nh.getParam("dryrun", new_dry_run)) && (new_dry_run != dry_run))
            {
                dry_run = new_dry_run;
                ROS_INFO("updateParam : new dry_run set to %s",dry_run ? "true" : "false");
            }
            if ((nh.getParam("mission_recorded_filename", new_mission_recorded_filename)) && (new_mission_recorded_filename != mission_recorded_filename))
            {
                mission_recorded_filename = new_mission_recorded_filename;
                ROS_INFO("updateParam : new mission_recorded_filename set to %s",mission_recorded_filename.c_str());
            }
            if ((nh.getParam("mission_script", new_mission_script)) && (new_mission_script != mission_script))
            {
                mission_script = new_mission_script;
                ROS_INFO("updateParam : new mission_script set to %s",mission_script.c_str());
            }
        }

        struct Quaternion {
            double w, x, y, z;
        };

        struct EulerAngles {
            double roll, pitch, yaw;
        };
        EulerAngles ToEulerAngles(Quaternion q) {
            EulerAngles angles;

            // roll (x-axis rotation)
            double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
            double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
            angles.roll = std::atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            double sinp = 2 * (q.w * q.y - q.z * q.x);
            if (std::abs(sinp) >= 1)
                angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            else
                angles.pitch = std::asin(sinp);

            // yaw (z-axis rotation)
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            angles.yaw = std::atan2(siny_cosp, cosy_cosp);

            return angles;
        }
        float getDestYawAngle(float destX, float destY);

        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void rcin_cb(const mavros_msgs::RCIn::ConstPtr& msg);
        void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
        void mission_cb(const setpoint_leader::robocars_mission::ConstPtr& mission_msg);
        void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& velocity_msg);
        void vfr_hud_cb (const mavros_msgs::VFR_HUD::ConstPtr& vfr_hud_msg);
        void startOffBoardMode (void);
        void stopOffBoardMode(void);
        void startManualMode (void);
        void armVehicule (void);
        void checkOffBoardResponse (void);
        void checkArmingResponse (void);
        void disarmVehicule (void);
        void checkCurrentState (void);

        void sendIdlePosition (float destX=0.0f, float destY=0.0f, float destZ=0.0f);
        void sendBrakeOrder (float destX=0.0f, float destY=0.0f, float destZ=0.0f);
        void sendMissionPosition (float destX, float destY, float destZ);
        void sendRobocarsAttitude();

        // drone adapted mission orders
        void sendTakeOff(float destX=0.0f, float destY=0.0f, float destZ=0.0f);
        void sendLoiterPosition(float destX=0.0f, float destY=0.0f, float destZ=0.0f);
        void sendLandPosition(float destX=0.0f, float destY=0.0f, float destZ=0.0f);
        void sendMissionOrder(TypeMasks action=MASK_IDLE_POSITION, float destX=0.0f, float destY=0.0f, float destZ=0.0f);
        void sendMissionOrderVelocity(TypeMasks action=MASK_IDLE_POSITION, float destX=0.0f, float destY=0.0f, float dst_treshold=0.5f, bool landingMode=false);
        void doFrontflip(void);

        void sendVelocityOrder(float vX=0.0f, float vY=0.0f, float vZ=0.0f);

        void sendDebug (float destX, float destY, float destZ, float dst);
        int getNextWaypoint (float &destX, float &destY, float &destZ, float &destMode);
        void printCurrentPosition();
        void getCurrentPosition(float &destX, float &destY, float &destZ);
        double getCurrentYaw();
        float getDestinationdistance(float ros_px, float ros_py, float ros_pz, float X, float Y, float z);
        int isDestinationReached (float destX, float destY, float destZ, float X, float Y, float z);
        int isDestinationReachedInXSecond (float second, float destX, float destY, float destZ);
        int isVehicleLanded ();
        int isDepartureReached ();
        int processWaypoint (float &destX, float &destY, float &destZ);
        void clearMission(void);
        void recordCurrentPosition(void);
        void saveRecordedMission(string filename);
        void clearRecordedMission(void);
        int loadMissionRecorded(int lap_nb=1);

        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::Publisher local_raw_pub;
        ros::Publisher robocars_attitude_pub;
        ros::Publisher robocars_at_pickup_pub;
        ros::Publisher robocars_at_dest_pub;
        ros::Publisher robocars_on_duty_pub;
        ros::Publisher events_pub;

        bool dry_run=false;

    private:

        mavros_msgs::State current_state;
        mavros_msgs::RCIn current_rcin;
        int rcin_mode_channel = 10;
        int rcin_record_position_channel = 11;
        int rcin_pause_mission_channel = 12;
        int rcin_seq;
        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        float dst_thresh=0.25f;
        float dst_yaw_thresh=dst_thresh*1.5;
        float dst_vel_treshold = 0.5f;
        float landing_velocity=-0.5f;
        float refVelocity=2.0f;
        string mission_recorded_filename="";
        string mission_script="";

        geometry_msgs::PoseStamped current_pose;
        geometry_msgs::TwistStamped current_velocity;
        float current_vel;
        mavros_msgs::VFR_HUD current_vfr_hud;

        std::vector<mavros_msgs::Waypoint> waypoints;
        std::vector<mavros_msgs::Waypoint> recorded_waypoints;
        setpoint_leader::robocars_mission recorded_mission;

        geometry_msgs::PoseStamped departurePos;
        geometry_msgs::PoseStamped arrivalPos;

        int pos_seq;

        ros::NodeHandle nh;
        ros::Subscriber state_sub;
        ros::Subscriber rcin_sub;
        ros::Subscriber pose_sub;
        ros::Subscriber mission_sub;
        ros::Subscriber velocity_sub;
        ros::Subscriber vfr_hud_sub;

};

