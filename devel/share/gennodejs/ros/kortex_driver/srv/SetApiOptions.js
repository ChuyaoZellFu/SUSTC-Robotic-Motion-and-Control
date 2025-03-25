// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ApiOptions = require('../msg/ApiOptions.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetApiOptionsRequest {
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
        this.input = new ApiOptions();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetApiOptionsRequest
    // Serialize message field [input]
    bufferOffset = ApiOptions.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetApiOptionsRequest
    let len;
    let data = new SetApiOptionsRequest(null);
    // Deserialize message field [input]
    data.input = ApiOptions.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetApiOptionsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '11c85b567b00c6eb6cd55fdb8cb9ad1b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ApiOptions input
    
    ================================================================================
    MSG: kortex_driver/ApiOptions
    uint32 timeout_ms
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetApiOptionsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ApiOptions.Resolve(msg.input)
    }
    else {
      resolved.input = new ApiOptions()
    }

    return resolved;
    }
};

class SetApiOptionsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetApiOptionsResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetApiOptionsResponse
    let len;
    let data = new SetApiOptionsResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetApiOptionsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetApiOptionsResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetApiOptionsRequest,
  Response: SetApiOptionsResponse,
  md5sum() { return '11c85b567b00c6eb6cd55fdb8cb9ad1b'; },
  datatype() { return 'kortex_driver/SetApiOptions'; }
};
