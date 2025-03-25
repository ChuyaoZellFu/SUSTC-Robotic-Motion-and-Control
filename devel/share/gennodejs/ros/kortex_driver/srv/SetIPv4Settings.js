// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let IPv4Settings = require('../msg/IPv4Settings.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetIPv4SettingsRequest {
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
        this.input = new IPv4Settings();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetIPv4SettingsRequest
    // Serialize message field [input]
    bufferOffset = IPv4Settings.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetIPv4SettingsRequest
    let len;
    let data = new SetIPv4SettingsRequest(null);
    // Deserialize message field [input]
    data.input = IPv4Settings.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetIPv4SettingsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0e9803a8341a8bce6029d7ec9bb40beb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    IPv4Settings input
    
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
    const resolved = new SetIPv4SettingsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = IPv4Settings.Resolve(msg.input)
    }
    else {
      resolved.input = new IPv4Settings()
    }

    return resolved;
    }
};

class SetIPv4SettingsResponse {
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
    // Serializes a message object of type SetIPv4SettingsResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetIPv4SettingsResponse
    let len;
    let data = new SetIPv4SettingsResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetIPv4SettingsResponse';
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
    const resolved = new SetIPv4SettingsResponse(null);
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
  Request: SetIPv4SettingsRequest,
  Response: SetIPv4SettingsResponse,
  md5sum() { return '361b42ee5a195ec231ecbbedd2098401'; },
  datatype() { return 'kortex_driver/SetIPv4Settings'; }
};
