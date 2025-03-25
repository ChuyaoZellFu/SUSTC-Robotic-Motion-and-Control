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

let TwistLinearSoftLimit = require('../msg/TwistLinearSoftLimit.js');

//-----------------------------------------------------------

class ResetTwistLinearSoftLimitRequest {
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
    // Serializes a message object of type ResetTwistLinearSoftLimitRequest
    // Serialize message field [input]
    bufferOffset = ControlConfig_ControlModeInformation.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetTwistLinearSoftLimitRequest
    let len;
    let data = new ResetTwistLinearSoftLimitRequest(null);
    // Deserialize message field [input]
    data.input = ControlConfig_ControlModeInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ResetTwistLinearSoftLimitRequest';
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
    const resolved = new ResetTwistLinearSoftLimitRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ControlConfig_ControlModeInformation.Resolve(msg.input)
    }
    else {
      resolved.input = new ControlConfig_ControlModeInformation()
    }

    return resolved;
    }
};

class ResetTwistLinearSoftLimitResponse {
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
        this.output = new TwistLinearSoftLimit();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetTwistLinearSoftLimitResponse
    // Serialize message field [output]
    bufferOffset = TwistLinearSoftLimit.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetTwistLinearSoftLimitResponse
    let len;
    let data = new ResetTwistLinearSoftLimitResponse(null);
    // Deserialize message field [output]
    data.output = TwistLinearSoftLimit.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ResetTwistLinearSoftLimitResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ae0601debf9db4924c32c672f6fe6df5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    TwistLinearSoftLimit output
    
    ================================================================================
    MSG: kortex_driver/TwistLinearSoftLimit
    
    uint32 control_mode
    float32 twist_linear_soft_limit
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetTwistLinearSoftLimitResponse(null);
    if (msg.output !== undefined) {
      resolved.output = TwistLinearSoftLimit.Resolve(msg.output)
    }
    else {
      resolved.output = new TwistLinearSoftLimit()
    }

    return resolved;
    }
};

module.exports = {
  Request: ResetTwistLinearSoftLimitRequest,
  Response: ResetTwistLinearSoftLimitResponse,
  md5sum() { return '277326cfb7b831ac01de5d007b623790'; },
  datatype() { return 'kortex_driver/ResetTwistLinearSoftLimit'; }
};
