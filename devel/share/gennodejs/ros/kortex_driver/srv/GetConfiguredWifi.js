// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Ssid = require('../msg/Ssid.js');

//-----------------------------------------------------------

let WifiConfiguration = require('../msg/WifiConfiguration.js');

//-----------------------------------------------------------

class GetConfiguredWifiRequest {
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
        this.input = new Ssid();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetConfiguredWifiRequest
    // Serialize message field [input]
    bufferOffset = Ssid.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetConfiguredWifiRequest
    let len;
    let data = new GetConfiguredWifiRequest(null);
    // Deserialize message field [input]
    data.input = Ssid.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Ssid.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetConfiguredWifiRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e22d121851e9887ae67065d9db92deab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Ssid input
    
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
    const resolved = new GetConfiguredWifiRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Ssid.Resolve(msg.input)
    }
    else {
      resolved.input = new Ssid()
    }

    return resolved;
    }
};

class GetConfiguredWifiResponse {
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
        this.output = new WifiConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetConfiguredWifiResponse
    // Serialize message field [output]
    bufferOffset = WifiConfiguration.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetConfiguredWifiResponse
    let len;
    let data = new GetConfiguredWifiResponse(null);
    // Deserialize message field [output]
    data.output = WifiConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += WifiConfiguration.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetConfiguredWifiResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f527641f388c149b4396f8a32bb72e7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    WifiConfiguration output
    
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
    const resolved = new GetConfiguredWifiResponse(null);
    if (msg.output !== undefined) {
      resolved.output = WifiConfiguration.Resolve(msg.output)
    }
    else {
      resolved.output = new WifiConfiguration()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetConfiguredWifiRequest,
  Response: GetConfiguredWifiResponse,
  md5sum() { return '5d1a4bbd14ba02b4b58bf8863a4fba51'; },
  datatype() { return 'kortex_driver/GetConfiguredWifi'; }
};
