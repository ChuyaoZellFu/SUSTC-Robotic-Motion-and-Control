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

let IPv4Information = require('../msg/IPv4Information.js');

//-----------------------------------------------------------

class GetIPv4InformationRequest {
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
    // Serializes a message object of type GetIPv4InformationRequest
    // Serialize message field [input]
    bufferOffset = NetworkHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetIPv4InformationRequest
    let len;
    let data = new GetIPv4InformationRequest(null);
    // Deserialize message field [input]
    data.input = NetworkHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetIPv4InformationRequest';
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
    const resolved = new GetIPv4InformationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = NetworkHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new NetworkHandle()
    }

    return resolved;
    }
};

class GetIPv4InformationResponse {
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
        this.output = new IPv4Information();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetIPv4InformationResponse
    // Serialize message field [output]
    bufferOffset = IPv4Information.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetIPv4InformationResponse
    let len;
    let data = new GetIPv4InformationResponse(null);
    // Deserialize message field [output]
    data.output = IPv4Information.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetIPv4InformationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '61c113c0a52a621255f3611541f34dcc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    IPv4Information output
    
    ================================================================================
    MSG: kortex_driver/IPv4Information
    
    uint32 ip_address
    uint32 subnet_mask
    uint32 default_gateway
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetIPv4InformationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = IPv4Information.Resolve(msg.output)
    }
    else {
      resolved.output = new IPv4Information()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetIPv4InformationRequest,
  Response: GetIPv4InformationResponse,
  md5sum() { return 'cf6aba6a62d331d4b104d99fe292f36c'; },
  datatype() { return 'kortex_driver/GetIPv4Information'; }
};
