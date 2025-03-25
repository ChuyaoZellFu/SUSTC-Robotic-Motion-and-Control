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

let ToolConfiguration = require('../msg/ToolConfiguration.js');

//-----------------------------------------------------------

class GetToolConfigurationRequest {
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
    // Serializes a message object of type GetToolConfigurationRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetToolConfigurationRequest
    let len;
    let data = new GetToolConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetToolConfigurationRequest';
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
    const resolved = new GetToolConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetToolConfigurationResponse {
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
        this.output = new ToolConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetToolConfigurationResponse
    // Serialize message field [output]
    bufferOffset = ToolConfiguration.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetToolConfigurationResponse
    let len;
    let data = new GetToolConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = ToolConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetToolConfigurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e8aee88048229955a9c6e4f254f854b1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ToolConfiguration output
    
    ================================================================================
    MSG: kortex_driver/ToolConfiguration
    
    CartesianTransform tool_transform
    float32 tool_mass
    ControlConfig_Position tool_mass_center
    ================================================================================
    MSG: kortex_driver/CartesianTransform
    
    float32 x
    float32 y
    float32 z
    float32 theta_x
    float32 theta_y
    float32 theta_z
    ================================================================================
    MSG: kortex_driver/ControlConfig_Position
    
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetToolConfigurationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = ToolConfiguration.Resolve(msg.output)
    }
    else {
      resolved.output = new ToolConfiguration()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetToolConfigurationRequest,
  Response: GetToolConfigurationResponse,
  md5sum() { return '49c06fd31c2188ee0d629d1b8491b1f0'; },
  datatype() { return 'kortex_driver/GetToolConfiguration'; }
};
