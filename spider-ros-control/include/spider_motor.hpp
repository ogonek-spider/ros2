#include <cstdint>
#include <cmath>
#include "twai_proto.h"


struct SpiderMotor {
    SpiderMotor(uint8_t leg_id, uint8_t motor_id): leg_id_(leg_id), motor_id_(motor_id) {}

    uint8_t leg_id_;
    uint8_t motor_id_;

    double angle_setpoint_ = NAN; // [rad]
    double angle_ = NAN; // [rad]
    double current_ = NAN;

    double last_angle_setpoint_ = NAN; // to not send each angle each tick  

    void processMsg(CanMessage msg) {
        switch(msg.type) {
            case MessageType::MOTOR_HEARTBEAT:
                angle_ = msg.data.heartbeat.angle;
                current_ = msg.data.heartbeat.current;
                if (angle_setpoint_ == NAN) {
                    angle_setpoint_ = angle_;
                    last_angle_setpoint_ = angle_;
                }
            break;
            default:
            //do nothing
            break;
        }
    }

    std::vector<CanMessage> getMsgsToSend() {
        std::vector<CanMessage> msgs;
        if (angle_setpoint_ != NAN && angle_setpoint_ != last_angle_setpoint_) {
            last_angle_setpoint_ = angle_setpoint_;
            msgs.push_back({
                .address = {
                    .legn = leg_id_,
                    .motorn = motor_id_
                },
                .type = MessageType::SET_ANGLE,
                .data = {
                    .angle = {
                        .angle = (float) angle_setpoint_
                    }
                }
            }
            );        
       }
       return msgs;
    }
};