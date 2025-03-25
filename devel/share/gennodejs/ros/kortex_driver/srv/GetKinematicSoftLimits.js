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

let KinematicLimits = require('../msg/KinematicLimits.js');

//-----------------------------------------------------------

class GetKinematicSoftLimitsRequest {
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
    // Serializes a message object of type GetKinematicSoftLimitsRequest
    // Serialize message field [input]
    bufferOffset = ControlConfig_ControlModeInformation.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetKinematicSoftLimitsRequest
    let len;
    let data = new GetKinematicSoftLimitsRequest(null);
    // Deserialize message field [input]
    data.input = ControlConfig_ControlModeInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetKinematicSoftLimitsRequest';
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
    const resolved = new GetKinematicSoftLimitsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ControlConfig_ControlModeInformation.Resolve(msg.input)
    }
    else {
      resolved.input = new ControlConfig_ControlModeInformation()
    }

    return resolved;
    }
};

class GetKinematicSoftLimitsResponse {
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
        this.output = new KinematicLimits();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetKinematicSoftLimitsResponse
    // Serialize message field [output]
    bufferOffset = KinematicLimits.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetKinematicSoftLimitsResponse
    let len;
    let data = new GetKinematicSoftLimitsResponse(null);
    // Deserialize message field [output]
    data.output = KinematicLimits.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += KinematicLimits.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetKinematicSoftLimitsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2299657e3a9e44eda02029d1ffd04763';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    KinematicLimits output
    
    ================================================================================
    MSG: kortex_driver/KinematicLimits
    
    uint32 control_mode
    float32 twist_linear
    float32 twist_angular
    float32[] joint_speed_limits
    float32[] joint_acceleration_limits
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetKinematicSoftLimitsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = KinematicLimits.Resolve(msg.output)
    }
    else {
      resolved.output = new KinematicLimits()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetKinematicSoftLimitsRequest,
  Response: GetKinematicSoftLimitsResponse,
  md5sum() { return '8372b01bbf6aa0f718ce44248decf90a'; },
  datatype() { return 'kortex_driver/GetKinematicSoftLimits'; }
};
