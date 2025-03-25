// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let EthernetDeviceIdentification = require('../msg/EthernetDeviceIdentification.js');

//-----------------------------------------------------------

let EthernetConfiguration = require('../msg/EthernetConfiguration.js');

//-----------------------------------------------------------

class GetEthernetConfigurationRequest {
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
        this.input = new EthernetDeviceIdentification();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetEthernetConfigurationRequest
    // Serialize message field [input]
    bufferOffset = EthernetDeviceIdentification.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetEthernetConfigurationRequest
    let len;
    let data = new GetEthernetConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = EthernetDeviceIdentification.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetEthernetConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2afd0c991c8e92ffa9e59279739b28dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    EthernetDeviceIdentification input
    
    ================================================================================
    MSG: kortex_driver/EthernetDeviceIdentification
    
    uint32 device
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetEthernetConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = EthernetDeviceIdentification.Resolve(msg.input)
    }
    else {
      resolved.input = new EthernetDeviceIdentification()
    }

    return resolved;
    }
};

class GetEthernetConfigurationResponse {
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
        this.output = new EthernetConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetEthernetConfigurationResponse
    // Serialize message field [output]
    bufferOffset = EthernetConfiguration.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetEthernetConfigurationResponse
    let len;
    let data = new GetEthernetConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = EthernetConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetEthernetConfigurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '62205af0ba461c567072364c0b0527fe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    EthernetConfiguration output
    
    ================================================================================
    MSG: kortex_driver/EthernetConfiguration
    
    uint32 device
    bool enabled
    uint32 speed
    uint32 duplex
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetEthernetConfigurationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = EthernetConfiguration.Resolve(msg.output)
    }
    else {
      resolved.output = new EthernetConfiguration()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetEthernetConfigurationRequest,
  Response: GetEthernetConfigurationResponse,
  md5sum() { return 'd50fb352c6ad553c74cb37db16e3d609'; },
  datatype() { return 'kortex_driver/GetEthernetConfiguration'; }
};
