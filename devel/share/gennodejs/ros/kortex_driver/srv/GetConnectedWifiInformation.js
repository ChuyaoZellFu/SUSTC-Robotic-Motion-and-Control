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

let WifiInformation = require('../msg/WifiInformation.js');

//-----------------------------------------------------------

class GetConnectedWifiInformationRequest {
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
    // Serializes a message object of type GetConnectedWifiInformationRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetConnectedWifiInformationRequest
    let len;
    let data = new GetConnectedWifiInformationRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetConnectedWifiInformationRequest';
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
    const resolved = new GetConnectedWifiInformationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetConnectedWifiInformationResponse {
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
        this.output = new WifiInformation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetConnectedWifiInformationResponse
    // Serialize message field [output]
    bufferOffset = WifiInformation.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetConnectedWifiInformationResponse
    let len;
    let data = new GetConnectedWifiInformationResponse(null);
    // Deserialize message field [output]
    data.output = WifiInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += WifiInformation.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetConnectedWifiInformationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8ba865cd380cb46d3bb247102a4172a0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    WifiInformation output
    
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
    const resolved = new GetConnectedWifiInformationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = WifiInformation.Resolve(msg.output)
    }
    else {
      resolved.output = new WifiInformation()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetConnectedWifiInformationRequest,
  Response: GetConnectedWifiInformationResponse,
  md5sum() { return '38ff400e5249598bc1f7289a3e9733aa'; },
  datatype() { return 'kortex_driver/GetConnectedWifiInformation'; }
};
