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

let TorqueOffset = require('../msg/TorqueOffset.js');

//-----------------------------------------------------------

class GetTorqueOffsetRequest {
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
    // Serializes a message object of type GetTorqueOffsetRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetTorqueOffsetRequest
    let len;
    let data = new GetTorqueOffsetRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetTorqueOffsetRequest';
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
    const resolved = new GetTorqueOffsetRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetTorqueOffsetResponse {
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
        this.output = new TorqueOffset();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetTorqueOffsetResponse
    // Serialize message field [output]
    bufferOffset = TorqueOffset.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetTorqueOffsetResponse
    let len;
    let data = new GetTorqueOffsetResponse(null);
    // Deserialize message field [output]
    data.output = TorqueOffset.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetTorqueOffsetResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd3e46a3056af28b0b9b16f6e2c4b15c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    TorqueOffset output
    
    ================================================================================
    MSG: kortex_driver/TorqueOffset
    
    float32 torque_offset
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetTorqueOffsetResponse(null);
    if (msg.output !== undefined) {
      resolved.output = TorqueOffset.Resolve(msg.output)
    }
    else {
      resolved.output = new TorqueOffset()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetTorqueOffsetRequest,
  Response: GetTorqueOffsetResponse,
  md5sum() { return '6ed1621e934f6480be9371fb25fd7ea1'; },
  datatype() { return 'kortex_driver/GetTorqueOffset'; }
};
