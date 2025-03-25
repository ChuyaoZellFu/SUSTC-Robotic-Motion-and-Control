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

let CustomDataSelection = require('../msg/CustomDataSelection.js');

//-----------------------------------------------------------

class GetSelectedCustomDataRequest {
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
    // Serializes a message object of type GetSelectedCustomDataRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSelectedCustomDataRequest
    let len;
    let data = new GetSelectedCustomDataRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetSelectedCustomDataRequest';
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
    const resolved = new GetSelectedCustomDataRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetSelectedCustomDataResponse {
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
        this.output = new CustomDataSelection();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSelectedCustomDataResponse
    // Serialize message field [output]
    bufferOffset = CustomDataSelection.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSelectedCustomDataResponse
    let len;
    let data = new GetSelectedCustomDataResponse(null);
    // Deserialize message field [output]
    data.output = CustomDataSelection.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CustomDataSelection.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetSelectedCustomDataResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd715146aac153a78de5215d025bd9be8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CustomDataSelection output
    
    ================================================================================
    MSG: kortex_driver/CustomDataSelection
    
    uint32[] channel
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSelectedCustomDataResponse(null);
    if (msg.output !== undefined) {
      resolved.output = CustomDataSelection.Resolve(msg.output)
    }
    else {
      resolved.output = new CustomDataSelection()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetSelectedCustomDataRequest,
  Response: GetSelectedCustomDataResponse,
  md5sum() { return 'd4420814b14d58afc7b7b98f4fdd7907'; },
  datatype() { return 'kortex_driver/GetSelectedCustomData'; }
};
