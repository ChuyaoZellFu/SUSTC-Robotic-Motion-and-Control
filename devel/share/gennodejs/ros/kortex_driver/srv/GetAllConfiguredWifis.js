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

let WifiConfigurationList = require('../msg/WifiConfigurationList.js');

//-----------------------------------------------------------

class GetAllConfiguredWifisRequest {
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
    // Serializes a message object of type GetAllConfiguredWifisRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAllConfiguredWifisRequest
    let len;
    let data = new GetAllConfiguredWifisRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetAllConfiguredWifisRequest';
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
    const resolved = new GetAllConfiguredWifisRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetAllConfiguredWifisResponse {
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
        this.output = new WifiConfigurationList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAllConfiguredWifisResponse
    // Serialize message field [output]
    bufferOffset = WifiConfigurationList.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAllConfiguredWifisResponse
    let len;
    let data = new GetAllConfiguredWifisResponse(null);
    // Deserialize message field [output]
    data.output = WifiConfigurationList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += WifiConfigurationList.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetAllConfiguredWifisResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3008b2fabc6af5deeb24b1749ebe403a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    WifiConfigurationList output
    
    ================================================================================
    MSG: kortex_driver/WifiConfigurationList
    
    WifiConfiguration[] wifi_configuration_list
    ================================================================================
    MSG: kortex_driver/WifiConfiguration
    
    Ssid ssid
    string security_key
    bool connect_automatically
    ================================================================================
    MSG: kortex_driver/Ssid
    
    string identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetAllConfiguredWifisResponse(null);
    if (msg.output !== undefined) {
      resolved.output = WifiConfigurationList.Resolve(msg.output)
    }
    else {
      resolved.output = new WifiConfigurationList()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetAllConfiguredWifisRequest,
  Response: GetAllConfiguredWifisResponse,
  md5sum() { return '00cc4429e5b474aecc287723cc4eff69'; },
  datatype() { return 'kortex_driver/GetAllConfiguredWifis'; }
};
