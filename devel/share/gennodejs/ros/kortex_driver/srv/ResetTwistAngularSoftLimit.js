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

let TwistAngularSoftLimit = require('../msg/TwistAngularSoftLimit.js');

//-----------------------------------------------------------

class ResetTwistAngularSoftLimitRequest {
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
    // Serializes a message object of type ResetTwistAngularSoftLimitRequest
    // Serialize message field [input]
    bufferOffset = ControlConfig_ControlModeInformation.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetTwistAngularSoftLimitRequest
    let len;
    let data = new ResetTwistAngularSoftLimitRequest(null);
    // Deserialize message field [input]
    data.input = ControlConfig_ControlModeInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ResetTwistAngularSoftLimitRequest';
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
    const resolved = new ResetTwistAngularSoftLimitRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ControlConfig_ControlModeInformation.Resolve(msg.input)
    }
    else {
      resolved.input = new ControlConfig_ControlModeInformation()
    }

    return resolved;
    }
};

class ResetTwistAngularSoftLimitResponse {
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
        this.output = new TwistAngularSoftLimit();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetTwistAngularSoftLimitResponse
    // Serialize message field [output]
    bufferOffset = TwistAngularSoftLimit.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetTwistAngularSoftLimitResponse
    let len;
    let data = new ResetTwistAngularSoftLimitResponse(null);
    // Deserialize message field [output]
    data.output = TwistAngularSoftLimit.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ResetTwistAngularSoftLimitResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1ad82473bf29ed446f8b39f5390816e9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    TwistAngularSoftLimit output
    
    ================================================================================
    MSG: kortex_driver/TwistAngularSoftLimit
    
    uint32 control_mode
    float32 twist_angular_soft_limit
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetTwistAngularSoftLimitResponse(null);
    if (msg.output !== undefined) {
      resolved.output = TwistAngularSoftLimit.Resolve(msg.output)
    }
    else {
      resolved.output = new TwistAngularSoftLimit()
    }

    return resolved;
    }
};

module.exports = {
  Request: ResetTwistAngularSoftLimitRequest,
  Response: ResetTwistAngularSoftLimitResponse,
  md5sum() { return '24d97d0cfe0dd3c18032abd06a33780b'; },
  datatype() { return 'kortex_driver/ResetTwistAngularSoftLimit'; }
};
