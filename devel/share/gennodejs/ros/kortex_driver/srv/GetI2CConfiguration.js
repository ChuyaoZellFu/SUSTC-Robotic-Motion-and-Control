// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let I2CDeviceIdentification = require('../msg/I2CDeviceIdentification.js');

//-----------------------------------------------------------

let I2CConfiguration = require('../msg/I2CConfiguration.js');

//-----------------------------------------------------------

class GetI2CConfigurationRequest {
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
        this.input = new I2CDeviceIdentification();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetI2CConfigurationRequest
    // Serialize message field [input]
    bufferOffset = I2CDeviceIdentification.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetI2CConfigurationRequest
    let len;
    let data = new GetI2CConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = I2CDeviceIdentification.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetI2CConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2afd0c991c8e92ffa9e59279739b28dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    I2CDeviceIdentification input
    
    ================================================================================
    MSG: kortex_driver/I2CDeviceIdentification
    
    uint32 device
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetI2CConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = I2CDeviceIdentification.Resolve(msg.input)
    }
    else {
      resolved.input = new I2CDeviceIdentification()
    }

    return resolved;
    }
};

class GetI2CConfigurationResponse {
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
        this.output = new I2CConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetI2CConfigurationResponse
    // Serialize message field [output]
    bufferOffset = I2CConfiguration.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetI2CConfigurationResponse
    let len;
    let data = new GetI2CConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = I2CConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetI2CConfigurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '22a01e0c2b30e1df5d343f8fbe6a2673';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    I2CConfiguration output
    
    ================================================================================
    MSG: kortex_driver/I2CConfiguration
    
    uint32 device
    bool enabled
    uint32 mode
    uint32 addressing
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetI2CConfigurationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = I2CConfiguration.Resolve(msg.output)
    }
    else {
      resolved.output = new I2CConfiguration()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetI2CConfigurationRequest,
  Response: GetI2CConfigurationResponse,
  md5sum() { return 'd50c46d9bec318dcd7281869a6e5606a'; },
  datatype() { return 'kortex_driver/GetI2CConfiguration'; }
};
