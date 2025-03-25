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

let CoggingFeedforwardModeInformation = require('../msg/CoggingFeedforwardModeInformation.js');

//-----------------------------------------------------------

class GetCoggingFeedforwardModeRequest {
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
    // Serializes a message object of type GetCoggingFeedforwardModeRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCoggingFeedforwardModeRequest
    let len;
    let data = new GetCoggingFeedforwardModeRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetCoggingFeedforwardModeRequest';
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
    const resolved = new GetCoggingFeedforwardModeRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetCoggingFeedforwardModeResponse {
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
        this.output = new CoggingFeedforwardModeInformation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCoggingFeedforwardModeResponse
    // Serialize message field [output]
    bufferOffset = CoggingFeedforwardModeInformation.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCoggingFeedforwardModeResponse
    let len;
    let data = new GetCoggingFeedforwardModeResponse(null);
    // Deserialize message field [output]
    data.output = CoggingFeedforwardModeInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetCoggingFeedforwardModeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd1f3050013774259b1634ed413b40a8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CoggingFeedforwardModeInformation output
    
    ================================================================================
    MSG: kortex_driver/CoggingFeedforwardModeInformation
    
    uint32 cogging_feedforward_mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCoggingFeedforwardModeResponse(null);
    if (msg.output !== undefined) {
      resolved.output = CoggingFeedforwardModeInformation.Resolve(msg.output)
    }
    else {
      resolved.output = new CoggingFeedforwardModeInformation()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetCoggingFeedforwardModeRequest,
  Response: GetCoggingFeedforwardModeResponse,
  md5sum() { return '54f13ab10b17b140d71d14c7e922bc7e'; },
  datatype() { return 'kortex_driver/GetCoggingFeedforwardMode'; }
};
