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

let BootloaderVersion = require('../msg/BootloaderVersion.js');

//-----------------------------------------------------------

class GetBootloaderVersionRequest {
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
    // Serializes a message object of type GetBootloaderVersionRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetBootloaderVersionRequest
    let len;
    let data = new GetBootloaderVersionRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetBootloaderVersionRequest';
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
    const resolved = new GetBootloaderVersionRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetBootloaderVersionResponse {
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
        this.output = new BootloaderVersion();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetBootloaderVersionResponse
    // Serialize message field [output]
    bufferOffset = BootloaderVersion.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetBootloaderVersionResponse
    let len;
    let data = new GetBootloaderVersionResponse(null);
    // Deserialize message field [output]
    data.output = BootloaderVersion.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetBootloaderVersionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6032ae0c8396ebaaa1f474b29da727c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    BootloaderVersion output
    
    ================================================================================
    MSG: kortex_driver/BootloaderVersion
    
    uint32 bootloader_version
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetBootloaderVersionResponse(null);
    if (msg.output !== undefined) {
      resolved.output = BootloaderVersion.Resolve(msg.output)
    }
    else {
      resolved.output = new BootloaderVersion()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetBootloaderVersionRequest,
  Response: GetBootloaderVersionResponse,
  md5sum() { return 'e674f344ecc5f8e12f3233bf3b7cbeaa'; },
  datatype() { return 'kortex_driver/GetBootloaderVersion'; }
};
