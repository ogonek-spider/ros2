#include <cstdint>
#include <cmath>
#include "twai_proto.h"


struct SpiderMotor {
    SpiderMotor(uint8_t leg_id, uint8_t motor_id): leg_id_(leg_id), motor_id_(motor_id) {}

    uint8_t leg_id_;
    uint8_t motor_id_;

    double angle_setpoint_ = NAN; // [rad]
    double angle_ = 0; // [rad]
    double current_ = NAN;

    double P_ = NAN;
    double Pprev_ = NAN;
    double I_ = NAN;
    double Iprev_ = NAN;
    double D_ = NAN;
    double Dprev_ = NAN;

    bool requested_vars = false;
    bool new_params_values = false;

    double last_angle_setpoint_ = NAN; // to not send each angle each tick  

    void processMsg(CanMessage msg) {
        switch(msg.type) {
            case MessageType::MOTOR_HEARTBEAT:
                angle_ = msg.data.heartbeat.angle;
                current_ = msg.data.heartbeat.current / 10.0;
                if (angle_setpoint_ == NAN) {
                    angle_setpoint_ = angle_;
                    last_angle_setpoint_ = angle_;
                }
            break;
            case MessageType::SET_VAR:
                new_params_values = true;
                switch (msg.data.setvar.name) {
                    case VarName::P:
                        P_ = msg.data.setvar.val;
                    break;
                    case VarName::I:
                        I_ = msg.data.setvar.val;
                    break;
                    case VarName::D:
                        D_ = msg.data.setvar.val;
                    break;
                    default:
                        //do nothing
                    break;
                }
            break;
            default:
            //do nothing
            break;
        }
    }

    std::vector<CanMessage> getMsgsToSend() {
        std::vector<CanMessage> msgs;
        
        MotorAddress addr = {
            .legn = leg_id_,
            .motorn = motor_id_
        };

        if (!std::isnan(angle_setpoint_) && angle_setpoint_ != last_angle_setpoint_) {
            last_angle_setpoint_ = angle_setpoint_;
            msgs.push_back({
                .address = addr,
                .type = MessageType::SET_ANGLE,
                .data = {
                    .angle = {
                        .angle = (float) angle_setpoint_
                    }
                }
            }
            );        
       }
       
       //getvar section (request once after start)
       if (!requested_vars) {
            requested_vars = true;
            CanMessage getvarmsg = {
                .address = addr,
                .type = MessageType::GET_VAR,
            };
            for (auto& vn: {VarName::P, VarName::I, VarName::D}) {
                getvarmsg.data.getvar.name = vn;
                msgs.push_back(getvarmsg);
            }
        }

        //setvar section, send if param changed
        CanMessage setvarmsg = {
            .address = addr,
            .type = MessageType::SET_VAR,
        };

        if (P_ != Pprev_ && !std::isnan(P_)) {
            Pprev_ = P_;
            setvarmsg.data.setvar.name = VarName::P;
            setvarmsg.data.setvar.val = (float) P_;
            msgs.push_back(setvarmsg);
        }
        if (I_ != Iprev_ && !std::isnan(I_)) {
            Iprev_ = I_;
            setvarmsg.data.setvar.name = VarName::I;
            setvarmsg.data.setvar.val = (float) I_;
            msgs.push_back(setvarmsg);        
        }        
        if (D_ != Dprev_ && !std::isnan(D_)) {
            Dprev_ = D_;
            setvarmsg.data.setvar.name = VarName::D;
            setvarmsg.data.setvar.val = (float) D_;
            msgs.push_back(setvarmsg);        
        }

        return msgs;
    }
};