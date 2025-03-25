// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

let KinematicLimits = require('../msg/KinematicLimits.js');

//-----------------------------------------------------------

class GetKinematicHardLimitsRequest {
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
        this.input = new Empty();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetKinematicHardLimitsRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetKinematicHardLimitsRequest
    let len;
    let data = new GetKinematicHardLimitsRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetKinematicHardLimitsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fa3403cd5897c9698bc0fdcb2a453fbc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Empty input
    
    ================================================================================
    MSG: kortex_driver/Empty
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetKinematicHardLimitsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetKinematicHardLimitsResponse {
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
    // Serializes a message object of type GetKinematicHardLimitsResponse
    // Serialize message field [output]
    bufferOffset = KinematicLimits.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetKinematicHardLimitsResponse
    let len;
    let data = new GetKinematicHardLimitsResponse(null);
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
    return 'kortex_driver/GetKinematicHardLimitsResponse';
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
    const resolved = new GetKinematicHardLimitsResponse(null);
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
  Request: GetKinematicHardLimitsRequest,
  Response: GetKinematicHardLimitsResponse,
  md5sum() { return '024037c761d64c76474f3dff50b971a3'; },
  datatype() { return 'kortex_driver/GetKinematicHardLimits'; }
};
