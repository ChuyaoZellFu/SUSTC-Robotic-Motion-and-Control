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

let DesiredSpeeds = require('../msg/DesiredSpeeds.js');

//-----------------------------------------------------------

class GetDesiredSpeedsRequest {
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
    // Serializes a message object of type GetDesiredSpeedsRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetDesiredSpeedsRequest
    let len;
    let data = new GetDesiredSpeedsRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetDesiredSpeedsRequest';
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
    const resolved = new GetDesiredSpeedsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetDesiredSpeedsResponse {
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
        this.output = new DesiredSpeeds();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetDesiredSpeedsResponse
    // Serialize message field [output]
    bufferOffset = DesiredSpeeds.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetDesiredSpeedsResponse
    let len;
    let data = new GetDesiredSpeedsResponse(null);
    // Deserialize message field [output]
    data.output = DesiredSpeeds.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += DesiredSpeeds.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetDesiredSpeedsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd243d49226b26aa713d823fbebf98e15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    DesiredSpeeds output
    
    ================================================================================
    MSG: kortex_driver/DesiredSpeeds
    
    float32 linear
    float32 angular
    float32[] joint_speed
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetDesiredSpeedsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = DesiredSpeeds.Resolve(msg.output)
    }
    else {
      resolved.output = new DesiredSpeeds()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetDesiredSpeedsRequest,
  Response: GetDesiredSpeedsResponse,
  md5sum() { return '1dd665a1d839a65c717e02f2317ca84c'; },
  datatype() { return 'kortex_driver/GetDesiredSpeeds'; }
};
