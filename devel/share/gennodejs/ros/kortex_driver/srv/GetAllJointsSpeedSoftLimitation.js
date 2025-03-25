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

let JointsLimitationsList = require('../msg/JointsLimitationsList.js');

//-----------------------------------------------------------

class GetAllJointsSpeedSoftLimitationRequest {
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
    // Serializes a message object of type GetAllJointsSpeedSoftLimitationRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAllJointsSpeedSoftLimitationRequest
    let len;
    let data = new GetAllJointsSpeedSoftLimitationRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetAllJointsSpeedSoftLimitationRequest';
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
    const resolved = new GetAllJointsSpeedSoftLimitationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetAllJointsSpeedSoftLimitationResponse {
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
        this.output = new JointsLimitationsList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAllJointsSpeedSoftLimitationResponse
    // Serialize message field [output]
    bufferOffset = JointsLimitationsList.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAllJointsSpeedSoftLimitationResponse
    let len;
    let data = new GetAllJointsSpeedSoftLimitationResponse(null);
    // Deserialize message field [output]
    data.output = JointsLimitationsList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += JointsLimitationsList.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetAllJointsSpeedSoftLimitationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '51e6c4b6163d0d3c0b0ad680758e82f2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    JointsLimitationsList output
    
    ================================================================================
    MSG: kortex_driver/JointsLimitationsList
    
    JointLimitation[] joints_limitations
    ================================================================================
    MSG: kortex_driver/JointLimitation
    
    uint32 joint_identifier
    uint32 type
    float32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAllJointsSpeedSoftLimitationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = JointsLimitationsList.Resolve(msg.output)
    }
    else {
      resolved.output = new JointsLimitationsList()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetAllJointsSpeedSoftLimitationRequest,
  Response: GetAllJointsSpeedSoftLimitationResponse,
  md5sum() { return '80ab5247f79a646a096a6bb5bc451a7d'; },
  datatype() { return 'kortex_driver/GetAllJointsSpeedSoftLimitation'; }
};
