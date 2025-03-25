// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');

//-----------------------------------------------------------

class InterconnectCyclic_Feedback_tool_feedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gripper_feedback = null;
    }
    else {
      if (initObj.hasOwnProperty('gripper_feedback')) {
        this.gripper_feedback = initObj.gripper_feedback
      }
      else {
        this.gripper_feedback = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InterconnectCyclic_Feedback_tool_feedback
    // Serialize message field [gripper_feedback]
    // Serialize the length for message field [gripper_feedback]
    bufferOffset = _serializer.uint32(obj.gripper_feedback.length, buffer, bufferOffset);
    obj.gripper_feedback.forEach((val) => {
      bufferOffset = GripperCyclic_Feedback.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InterconnectCyclic_Feedback_tool_feedback
    let len;
    let data = new InterconnectCyclic_Feedback_tool_feedback(null);
    // Deserialize message field [gripper_feedback]
    // Deserialize array length for message field [gripper_feedback]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.gripper_feedback = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.gripper_feedback[i] = GripperCyclic_Feedback.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.gripper_feedback.forEach((val) => {
      length += GripperCyclic_Feedback.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/InterconnectCyclic_Feedback_tool_feedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '39e2425dcc897a387ad5c442c6119c99';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new InterconnectCyclic_Feedback_tool_feedback(null);
    if (msg.gripper_feedback !== undefined) {
      resolved.gripper_feedback = new Array(msg.gripper_feedback.length);
      for (let i = 0; i < resolved.gripper_feedback.length; ++i) {
        resolved.gripper_feedback[i] = GripperCyclic_Feedback.Resolve(msg.gripper_feedback[i]);
      }
    }
    else {
      resolved.gripper_feedback = []
    }

    return resolved;
    }
};

module.exports = InterconnectCyclic_Feedback_tool_feedback;
