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

let IPv4Settings = require('../msg/IPv4Settings.js');

//-----------------------------------------------------------

class GetIPv4SettingsRequest {
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
    // Serializes a message object of type GetIPv4SettingsRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetIPv4SettingsRequest
    let len;
    let data = new GetIPv4SettingsRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetIPv4SettingsRequest';
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
    const resolved = new GetIPv4SettingsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetIPv4SettingsResponse {
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
        this.output = new IPv4Settings();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetIPv4SettingsResponse
    // Serialize message field [output]
    bufferOffset = IPv4Settings.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetIPv4SettingsResponse
    let len;
    let data = new GetIPv4SettingsResponse(null);
    // Deserialize message field [output]
    data.output = IPv4Settings.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetIPv4SettingsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a7f194a88227d01d6cd4f86594a64691';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    IPv4Settings output
    
    ================================================================================
    MSG: kortex_driver/IPv4Settings
    
    uint32 ipv4_address
    uint32 ipv4_subnet_mask
    uint32 ipv4_default_gateway
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetIPv4SettingsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = IPv4Settings.Resolve(msg.output)
    }
    else {
      resolved.output = new IPv4Settings()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetIPv4SettingsRequest,
  Response: GetIPv4SettingsResponse,
  md5sum() { return 'e77074eb046f4a5b909d72ae5404567e'; },
  datatype() { return 'kortex_driver/GetIPv4Settings'; }
};
