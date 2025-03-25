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

let ControlLoop = require('../msg/ControlLoop.js');

//-----------------------------------------------------------

class GetActivatedControlLoopRequest {
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
    // Serializes a message object of type GetActivatedControlLoopRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetActivatedControlLoopRequest
    let len;
    let data = new GetActivatedControlLoopRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetActivatedControlLoopRequest';
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
    const resolved = new GetActivatedControlLoopRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetActivatedControlLoopResponse {
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
        this.output = new ControlLoop();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetActivatedControlLoopResponse
    // Serialize message field [output]
    bufferOffset = ControlLoop.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetActivatedControlLoopResponse
    let len;
    let data = new GetActivatedControlLoopResponse(null);
    // Deserialize message field [output]
    data.output = ControlLoop.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetActivatedControlLoopResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8659f757cac972f595e612f312c1c5a9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ControlLoop output
    
    ================================================================================
    MSG: kortex_driver/ControlLoop
    
    uint32 control_loop
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetActivatedControlLoopResponse(null);
    if (msg.output !== undefined) {
      resolved.output = ControlLoop.Resolve(msg.output)
    }
    else {
      resolved.output = new ControlLoop()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetActivatedControlLoopRequest,
  Response: GetActivatedControlLoopResponse,
  md5sum() { return 'acf43fee1c712ad0288037798483900b'; },
  datatype() { return 'kortex_driver/GetActivatedControlLoop'; }
};
