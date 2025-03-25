// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SafetyConfiguration = require('../msg/SafetyConfiguration.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetSafetyConfigurationRequest {
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
        this.input = new SafetyConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetSafetyConfigurationRequest
    // Serialize message field [input]
    bufferOffset = SafetyConfiguration.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSafetyConfigurationRequest
    let len;
    let data = new SetSafetyConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = SafetyConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetSafetyConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '52bf64f2821ac8b203cdfa2e63cd3563';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SafetyConfiguration input
    
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
    const resolved = new SetSafetyConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = SafetyConfiguration.Resolve(msg.input)
    }
    else {
      resolved.input = new SafetyConfiguration()
    }

    return resolved;
    }
};

class SetSafetyConfigurationResponse {
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
        this.output = new Empty();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetSafetyConfigurationResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetSafetyConfigurationResponse
    let len;
    let data = new SetSafetyConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetSafetyConfigurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6c43d221c810050f75091660f63b0cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Empty output
    
    ================================================================================
    MSG: kortex_driver/Empty
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetSafetyConfigurationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = Empty.Resolve(msg.output)
    }
    else {
      resolved.output = new Empty()
    }

    return resolved;
    }
};

module.exports = {
  Request: SetSafetyConfigurationRequest,
  Response: SetSafetyConfigurationResponse,
  md5sum() { return 'b1a9f41cb42ab39cba6bfb32d20fea4f'; },
  datatype() { return 'kortex_driver/SetSafetyConfiguration'; }
};
