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

let WifiInformationList = require('../msg/WifiInformationList.js');

//-----------------------------------------------------------

class GetAvailableWifiRequest {
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
    // Serializes a message object of type GetAvailableWifiRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAvailableWifiRequest
    let len;
    let data = new GetAvailableWifiRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetAvailableWifiRequest';
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
    const resolved = new GetAvailableWifiRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetAvailableWifiResponse {
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
        this.output = new WifiInformationList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAvailableWifiResponse
    // Serialize message field [output]
    bufferOffset = WifiInformationList.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAvailableWifiResponse
    let len;
    let data = new GetAvailableWifiResponse(null);
    // Deserialize message field [output]
    data.output = WifiInformationList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += WifiInformationList.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetAvailableWifiResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '88de91381473bf8e58231cfed636243f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    WifiInformationList output
    
    ================================================================================
    MSG: kortex_driver/WifiInformationList
    
    WifiInformation[] wifi_information_list
    ================================================================================
    MSG: kortex_driver/WifiInformation
    
    Ssid ssid
    uint32 security_type
    uint32 encryption_type
    uint32 signal_quality
    int32 signal_strength
    uint32 frequency
    uint32 channel
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
    const resolved = new GetAvailableWifiResponse(null);
    if (msg.output !== undefined) {
      resolved.output = WifiInformationList.Resolve(msg.output)
    }
    else {
      resolved.output = new WifiInformationList()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetAvailableWifiRequest,
  Response: GetAvailableWifiResponse,
  md5sum() { return '7759436e458092e2d8f72d35d5d731f7'; },
  datatype() { return 'kortex_driver/GetAvailableWifi'; }
};
