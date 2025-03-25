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

let KinematicLimitsList = require('../msg/KinematicLimitsList.js');

//-----------------------------------------------------------

class GetAllKinematicSoftLimitsRequest {
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
    // Serializes a message object of type GetAllKinematicSoftLimitsRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAllKinematicSoftLimitsRequest
    let len;
    let data = new GetAllKinematicSoftLimitsRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetAllKinematicSoftLimitsRequest';
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
    const resolved = new GetAllKinematicSoftLimitsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetAllKinematicSoftLimitsResponse {
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
        this.output = new KinematicLimitsList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAllKinematicSoftLimitsResponse
    // Serialize message field [output]
    bufferOffset = KinematicLimitsList.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAllKinematicSoftLimitsResponse
    let len;
    let data = new GetAllKinematicSoftLimitsResponse(null);
    // Deserialize message field [output]
    data.output = KinematicLimitsList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += KinematicLimitsList.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetAllKinematicSoftLimitsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f318d6cff46e5c002a37e34b98bbff7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    KinematicLimitsList output
    
    ================================================================================
    MSG: kortex_driver/KinematicLimitsList
    
    KinematicLimits[] kinematic_limits_list
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
    const resolved = new GetAllKinematicSoftLimitsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = KinematicLimitsList.Resolve(msg.output)
    }
    else {
      resolved.output = new KinematicLimitsList()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetAllKinematicSoftLimitsRequest,
  Response: GetAllKinematicSoftLimitsResponse,
  md5sum() { return '82a8454ad50f9bddae049fa61b8b3c3d'; },
  datatype() { return 'kortex_driver/GetAllKinematicSoftLimits'; }
};
