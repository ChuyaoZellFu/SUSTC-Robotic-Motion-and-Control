// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let NetworkHandle = require('../msg/NetworkHandle.js');

//-----------------------------------------------------------

let IPv4Configuration = require('../msg/IPv4Configuration.js');

//-----------------------------------------------------------

class GetIPv4ConfigurationRequest {
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
        this.input = new NetworkHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetIPv4ConfigurationRequest
    // Serialize message field [input]
    bufferOffset = NetworkHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetIPv4ConfigurationRequest
    let len;
    let data = new GetIPv4ConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = NetworkHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetIPv4ConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd83837d5994a891dad63c33420a4fcdf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    NetworkHandle input
    
    ================================================================================
    MSG: kortex_driver/NetworkHandle
    
    uint32 type
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetIPv4ConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = NetworkHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new NetworkHandle()
    }

    return resolved;
    }
};

class GetIPv4ConfigurationResponse {
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
        this.output = new IPv4Configuration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetIPv4ConfigurationResponse
    // Serialize message field [output]
    bufferOffset = IPv4Configuration.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetIPv4ConfigurationResponse
    let len;
    let data = new GetIPv4ConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = IPv4Configuration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetIPv4ConfigurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9963fc67e6217a40e85f90b0fd2cca73';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    IPv4Configuration output
    
    ================================================================================
    MSG: kortex_driver/IPv4Configuration
    
    uint32 ip_address
    uint32 subnet_mask
    uint32 default_gateway
    bool dhcp_enabled
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetIPv4ConfigurationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = IPv4Configuration.Resolve(msg.output)
    }
    else {
      resolved.output = new IPv4Configuration()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetIPv4ConfigurationRequest,
  Response: GetIPv4ConfigurationResponse,
  md5sum() { return 'd1ab60af0d1991268832344a0f905c68'; },
  datatype() { return 'kortex_driver/GetIPv4Configuration'; }
};
