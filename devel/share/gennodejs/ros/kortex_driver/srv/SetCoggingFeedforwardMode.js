// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CoggingFeedforwardModeInformation = require('../msg/CoggingFeedforwardModeInformation.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetCoggingFeedforwardModeRequest {
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
        this.input = new CoggingFeedforwardModeInformation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCoggingFeedforwardModeRequest
    // Serialize message field [input]
    bufferOffset = CoggingFeedforwardModeInformation.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCoggingFeedforwardModeRequest
    let len;
    let data = new SetCoggingFeedforwardModeRequest(null);
    // Deserialize message field [input]
    data.input = CoggingFeedforwardModeInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetCoggingFeedforwardModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e07a693eebb20b9cc047af42353e9039';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CoggingFeedforwardModeInformation input
    
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
    const resolved = new SetCoggingFeedforwardModeRequest(null);
    if (msg.input !== undefined) {
      resolved.input = CoggingFeedforwardModeInformation.Resolve(msg.input)
    }
    else {
      resolved.input = new CoggingFeedforwardModeInformation()
    }

    return resolved;
    }
};

class SetCoggingFeedforwardModeResponse {
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
        this.output = new Empty();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCoggingFeedforwardModeResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCoggingFeedforwardModeResponse
    let len;
    let data = new SetCoggingFeedforwardModeResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetCoggingFeedforwardModeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6c43d221c810050f75091660f63b0cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Empty output
    
    ================================================================================
    MSG: kortex_driver/Empty
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetCoggingFeedforwardModeResponse(null);
    if (msg.output !== undefined) {
      resolved.output = Empty.Resolve(msg.output)
    }
    else {
      resolved.output = new Empty()
    }

    return resolved;
    }
};

module.exports = {
  Request: SetCoggingFeedforwardModeRequest,
  Response: SetCoggingFeedforwardModeResponse,
  md5sum() { return '156e5a43117ffd12af553796e457d579'; },
  datatype() { return 'kortex_driver/SetCoggingFeedforwardMode'; }
};
