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

let JointAccelerationSoftLimits = require('../msg/JointAccelerationSoftLimits.js');

//-----------------------------------------------------------

class ResetJointAccelerationSoftLimitsRequest {
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
    // Serializes a message object of type ResetJointAccelerationSoftLimitsRequest
    // Serialize message field [input]
    bufferOffset = ControlConfig_ControlModeInformation.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetJointAccelerationSoftLimitsRequest
    let len;
    let data = new ResetJointAccelerationSoftLimitsRequest(null);
    // Deserialize message field [input]
    data.input = ControlConfig_ControlModeInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ResetJointAccelerationSoftLimitsRequest';
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
    const resolved = new ResetJointAccelerationSoftLimitsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ControlConfig_ControlModeInformation.Resolve(msg.input)
    }
    else {
      resolved.input = new ControlConfig_ControlModeInformation()
    }

    return resolved;
    }
};

class ResetJointAccelerationSoftLimitsResponse {
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
        this.output = new JointAccelerationSoftLimits();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetJointAccelerationSoftLimitsResponse
    // Serialize message field [output]
    bufferOffset = JointAccelerationSoftLimits.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetJointAccelerationSoftLimitsResponse
    let len;
    let data = new ResetJointAccelerationSoftLimitsResponse(null);
    // Deserialize message field [output]
    data.output = JointAccelerationSoftLimits.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += JointAccelerationSoftLimits.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ResetJointAccelerationSoftLimitsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '32851ba558aa7f390542826be3ecdab7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    JointAccelerationSoftLimits output
    
    ================================================================================
    MSG: kortex_driver/JointAccelerationSoftLimits
    
    uint32 control_mode
    float32[] joint_acceleration_soft_limits
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetJointAccelerationSoftLimitsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = JointAccelerationSoftLimits.Resolve(msg.output)
    }
    else {
      resolved.output = new JointAccelerationSoftLimits()
    }

    return resolved;
    }
};

module.exports = {
  Request: ResetJointAccelerationSoftLimitsRequest,
  Response: ResetJointAccelerationSoftLimitsResponse,
  md5sum() { return '6dba409e37ab85922f7777c936234d59'; },
  datatype() { return 'kortex_driver/ResetJointAccelerationSoftLimits'; }
};
