// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SensorIdentifier = require('../msg/SensorIdentifier.js');

//-----------------------------------------------------------

let SensorSettings = require('../msg/SensorSettings.js');

//-----------------------------------------------------------

class GetSensorSettingsRequest {
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
        this.input = new SensorIdentifier();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSensorSettingsRequest
    // Serialize message field [input]
    bufferOffset = SensorIdentifier.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSensorSettingsRequest
    let len;
    let data = new GetSensorSettingsRequest(null);
    // Deserialize message field [input]
    data.input = SensorIdentifier.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetSensorSettingsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd6b665ee68fb76fe11bbb8a3f0942a78';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SensorIdentifier input
    
    ================================================================================
    MSG: kortex_driver/SensorIdentifier
    
    uint32 sensor
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSensorSettingsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = SensorIdentifier.Resolve(msg.input)
    }
    else {
      resolved.input = new SensorIdentifier()
    }

    return resolved;
    }
};

class GetSensorSettingsResponse {
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
        this.output = new SensorSettings();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSensorSettingsResponse
    // Serialize message field [output]
    bufferOffset = SensorSettings.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSensorSettingsResponse
    let len;
    let data = new GetSensorSettingsResponse(null);
    // Deserialize message field [output]
    data.output = SensorSettings.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetSensorSettingsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '26134c3a35535872dc70f8d5256dbeaa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SensorSettings output
    
    ================================================================================
    MSG: kortex_driver/SensorSettings
    
    uint32 sensor
    uint32 resolution
    uint32 frame_rate
    uint32 bit_rate
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSensorSettingsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = SensorSettings.Resolve(msg.output)
    }
    else {
      resolved.output = new SensorSettings()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetSensorSettingsRequest,
  Response: GetSensorSettingsResponse,
  md5sum() { return '16278c9fd5ce95a2e5bda4211043968e'; },
  datatype() { return 'kortex_driver/GetSensorSettings'; }
};
