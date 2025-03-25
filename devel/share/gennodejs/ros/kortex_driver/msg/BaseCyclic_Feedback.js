// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BaseFeedback = require('./BaseFeedback.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');

//-----------------------------------------------------------

class BaseCyclic_Feedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frame_id = null;
      this.base = null;
      this.actuators = null;
      this.interconnect = null;
    }
    else {
      if (initObj.hasOwnProperty('frame_id')) {
        this.frame_id = initObj.frame_id
      }
      else {
        this.frame_id = 0;
      }
      if (initObj.hasOwnProperty('base')) {
        this.base = initObj.base
      }
      else {
        this.base = new BaseFeedback();
      }
      if (initObj.hasOwnProperty('actuators')) {
        this.actuators = initObj.actuators
      }
      else {
        this.actuators = [];
      }
      if (initObj.hasOwnProperty('interconnect')) {
        this.interconnect = initObj.interconnect
      }
      else {
        this.interconnect = new InterconnectCyclic_Feedback();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BaseCyclic_Feedback
    // Serialize message field [frame_id]
    bufferOffset = _serializer.uint32(obj.frame_id, buffer, bufferOffset);
    // Serialize message field [base]
    bufferOffset = BaseFeedback.serialize(obj.base, buffer, bufferOffset);
    // Serialize message field [actuators]
    // Serialize the length for message field [actuators]
    bufferOffset = _serializer.uint32(obj.actuators.length, buffer, bufferOffset);
    obj.actuators.forEach((val) => {
      bufferOffset = ActuatorFeedback.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [interconnect]
    bufferOffset = InterconnectCyclic_Feedback.serialize(obj.interconnect, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BaseCyclic_Feedback
    let len;
    let data = new BaseCyclic_Feedback(null);
    // Deserialize message field [frame_id]
    data.frame_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [base]
    data.base = BaseFeedback.deserialize(buffer, bufferOffset);
    // Deserialize message field [actuators]
    // Deserialize array length for message field [actuators]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.actuators = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.actuators[i] = ActuatorFeedback.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [interconnect]
    data.interconnect = InterconnectCyclic_Feedback.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 56 * object.actuators.length;
    length += InterconnectCyclic_Feedback.getMessageSize(object.interconnect);
    return length + 168;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BaseCyclic_Feedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f56809f61aebc2804399d7366b38f65';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 frame_id
    BaseFeedback base
    ActuatorFeedback[] actuators
    InterconnectCyclic_Feedback interconnect
    ================================================================================
    MSG: kortex_driver/BaseFeedback
    
    uint32 active_state_connection_identifier
    uint32 active_state
    float32 arm_voltage
    float32 arm_current
    float32 temperature_cpu
    float32 temperature_ambient
    float32 imu_acceleration_x
    float32 imu_acceleration_y
    float32 imu_acceleration_z
    float32 imu_angular_velocity_x
    float32 imu_angular_velocity_y
    float32 imu_angular_velocity_z
    float32 tool_pose_x
    float32 tool_pose_y
    float32 tool_pose_z
    float32 tool_pose_theta_x
    float32 tool_pose_theta_y
    float32 tool_pose_theta_z
    float32 tool_twist_linear_x
    float32 tool_twist_linear_y
    float32 tool_twist_linear_z
    float32 tool_twist_angular_x
    float32 tool_twist_angular_y
    float32 tool_twist_angular_z
    float32 tool_external_wrench_force_x
    float32 tool_external_wrench_force_y
    float32 tool_external_wrench_force_z
    float32 tool_external_wrench_torque_x
    float32 tool_external_wrench_torque_y
    float32 tool_external_wrench_torque_z
    uint32 fault_bank_a
    uint32 fault_bank_b
    uint32 warning_bank_a
    uint32 warning_bank_b
    float32 commanded_tool_pose_x
    float32 commanded_tool_pose_y
    float32 commanded_tool_pose_z
    float32 commanded_tool_pose_theta_x
    float32 commanded_tool_pose_theta_y
    float32 commanded_tool_pose_theta_z
    ================================================================================
    MSG: kortex_driver/ActuatorFeedback
    
    uint32 command_id
    uint32 status_flags
    uint32 jitter_comm
    float32 position
    float32 velocity
    float32 torque
    float32 current_motor
    float32 voltage
    float32 temperature_motor
    float32 temperature_core
    uint32 fault_bank_a
    uint32 fault_bank_b
    uint32 warning_bank_a
    uint32 warning_bank_b
    ================================================================================
    MSG: kortex_driver/InterconnectCyclic_Feedback
    
    InterconnectCyclic_MessageId feedback_id
    uint32 status_flags
    uint32 jitter_comm
    float32 imu_acceleration_x
    float32 imu_acceleration_y
    float32 imu_acceleration_z
    float32 imu_angular_velocity_x
    float32 imu_angular_velocity_y
    float32 imu_angular_velocity_z
    float32 voltage
    float32 temperature_core
    uint32 fault_bank_a
    uint32 fault_bank_b
    uint32 warning_bank_a
    uint32 warning_bank_b
    InterconnectCyclic_Feedback_tool_feedback oneof_tool_feedback
    ================================================================================
    MSG: kortex_driver/InterconnectCyclic_MessageId
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/InterconnectCyclic_Feedback_tool_feedback
    
    GripperCyclic_Feedback[] gripper_feedback
    ================================================================================
    MSG: kortex_driver/GripperCyclic_Feedback
    
    GripperCyclic_MessageId feedback_id
    uint32 status_flags
    uint32 fault_bank_a
    uint32 fault_bank_b
    uint32 warning_bank_a
    uint32 warning_bank_b
    MotorFeedback[] motor
    ================================================================================
    MSG: kortex_driver/GripperCyclic_MessageId
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/MotorFeedback
    
    uint32 motor_id
    float32 position
    float32 velocity
    float32 current_motor
    float32 voltage
    float32 temperature_motor
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BaseCyclic_Feedback(null);
    if (msg.frame_id !== undefined) {
      resolved.frame_id = msg.frame_id;
    }
    else {
      resolved.frame_id = 0
    }

    if (msg.base !== undefined) {
      resolved.base = BaseFeedback.Resolve(msg.base)
    }
    else {
      resolved.base = new BaseFeedback()
    }

    if (msg.actuators !== undefined) {
      resolved.actuators = new Array(msg.actuators.length);
      for (let i = 0; i < resolved.actuators.length; ++i) {
        resolved.actuators[i] = ActuatorFeedback.Resolve(msg.actuators[i]);
      }
    }
    else {
      resolved.actuators = []
    }

    if (msg.interconnect !== undefined) {
      resolved.interconnect = InterconnectCyclic_Feedback.Resolve(msg.interconnect)
    }
    else {
      resolved.interconnect = new InterconnectCyclic_Feedback()
    }

    return resolved;
    }
};

module.exports = BaseCyclic_Feedback;
