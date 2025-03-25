// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SafetyHandle = require('../msg/SafetyHandle.js');

//-----------------------------------------------------------

let SafetyConfiguration = require('../msg/SafetyConfiguration.js');

//-----------------------------------------------------------

class GetSafetyConfigurationRequest {
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
        this.input = new SafetyHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSafetyConfigurationRequest
    // Serialize message field [input]
    bufferOffset = SafetyHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSafetyConfigurationRequest
    let len;
    let data = new GetSafetyConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = SafetyHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetSafetyConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28fc5544fa28c41b8590a221a4f4db30';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SafetyHandle input
    
    ================================================================================
    MSG: kortex_driver/SafetyHandle
    
    uint32 identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSafetyConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = SafetyHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new SafetyHandle()
    }

    return resolved;
    }
};

class GetSafetyConfigurationResponse {
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
        this.output = new SafetyConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSafetyConfigurationResponse
    // Serialize message field [output]
    bufferOffset = SafetyConfiguration.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSafetyConfigurationResponse
    let len;
    let data = new GetSafetyConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = SafetyConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetSafetyConfigurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f5914a096fb8f06a025903695bb5f3a5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SafetyConfiguration output
    
    ================================================================================
    MSG: kortex_driver/SafetyConfiguration
    
    SafetyHandle handle
    float32 error_threshold
    float32 warning_threshold
    SafetyEnable enable
    ================================================================================
    MSG: kortex_driver/SafetyHandle
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/SafetyEnable
    
    SafetyHandle handle
    bool enable
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSafetyConfigurationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = SafetyConfiguration.Resolve(msg.output)
    }
    else {
      resolved.output = new SafetyConfiguration()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetSafetyConfigurationRequest,
  Response: GetSafetyConfigurationResponse,
  md5sum() { return '943b22eac92db7c1fc8c0b5bbdfb1565'; },
  datatype() { return 'kortex_driver/GetSafetyConfiguration'; }
};
