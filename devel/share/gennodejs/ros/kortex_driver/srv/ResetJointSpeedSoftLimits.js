// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControlConfig_ControlModeInformation = require('../msg/ControlConfig_ControlModeInformation.js');

//-----------------------------------------------------------

let JointSpeedSoftLimits = require('../msg/JointSpeedSoftLimits.js');

//-----------------------------------------------------------

class ResetJointSpeedSoftLimitsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input = null;
    }
    else {
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = new ControlConfig_ControlModeInformation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetJointSpeedSoftLimitsRequest
    // Serialize message field [input]
    bufferOffset = ControlConfig_ControlModeInformation.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetJointSpeedSoftLimitsRequest
    let len;
    let data = new ResetJointSpeedSoftLimitsRequest(null);
    // Deserialize message field [input]
    data.input = ControlConfig_ControlModeInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ResetJointSpeedSoftLimitsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ca19a04230874fdc811929f351eb6d06';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ControlConfig_ControlModeInformation input
    
    ================================================================================
    MSG: kortex_driver/ControlConfig_ControlModeInformation
    
    uint32 control_mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetJointSpeedSoftLimitsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ControlConfig_ControlModeInformation.Resolve(msg.input)
    }
    else {
      resolved.input = new ControlConfig_ControlModeInformation()
    }

    return resolved;
    }
};

class ResetJointSpeedSoftLimitsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = new JointSpeedSoftLimits();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetJointSpeedSoftLimitsResponse
    // Serialize message field [output]
    bufferOffset = JointSpeedSoftLimits.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetJointSpeedSoftLimitsResponse
    let len;
    let data = new ResetJointSpeedSoftLimitsResponse(null);
    // Deserialize message field [output]
    data.output = JointSpeedSoftLimits.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += JointSpeedSoftLimits.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ResetJointSpeedSoftLimitsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f263903f71119aabf10f0bac0d210523';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    JointSpeedSoftLimits output
    
    ================================================================================
    MSG: kortex_driver/JointSpeedSoftLimits
    
    uint32 control_mode
    float32[] joint_speed_soft_limits
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetJointSpeedSoftLimitsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = JointSpeedSoftLimits.Resolve(msg.output)
    }
    else {
      resolved.output = new JointSpeedSoftLimits()
    }

    return resolved;
    }
};

module.exports = {
  Request: ResetJointSpeedSoftLimitsRequest,
  Response: ResetJointSpeedSoftLimitsResponse,
  md5sum() { return '7d22c334332447e55a639a3764ac0a17'; },
  datatype() { return 'kortex_driver/ResetJointSpeedSoftLimits'; }
};
