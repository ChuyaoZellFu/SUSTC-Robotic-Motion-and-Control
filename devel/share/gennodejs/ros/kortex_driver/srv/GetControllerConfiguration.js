// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControllerHandle = require('../msg/ControllerHandle.js');

//-----------------------------------------------------------

let ControllerConfiguration = require('../msg/ControllerConfiguration.js');

//-----------------------------------------------------------

class GetControllerConfigurationRequest {
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
        this.input = new ControllerHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetControllerConfigurationRequest
    // Serialize message field [input]
    bufferOffset = ControllerHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetControllerConfigurationRequest
    let len;
    let data = new GetControllerConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = ControllerHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetControllerConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d6670247f140ab183942a0576f87f5f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ControllerHandle input
    
    ================================================================================
    MSG: kortex_driver/ControllerHandle
    
    uint32 type
    uint32 controller_identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetControllerConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ControllerHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new ControllerHandle()
    }

    return resolved;
    }
};

class GetControllerConfigurationResponse {
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
        this.output = new ControllerConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetControllerConfigurationResponse
    // Serialize message field [output]
    bufferOffset = ControllerConfiguration.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetControllerConfigurationResponse
    let len;
    let data = new GetControllerConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = ControllerConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ControllerConfiguration.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetControllerConfigurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b8d77785a2ac7668126fc41da85857b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ControllerConfiguration output
    
    ================================================================================
    MSG: kortex_driver/ControllerConfiguration
    
    ControllerHandle handle
    string name
    MappingHandle active_mapping_handle
    string analog_input_identifier_enum
    string digital_input_identifier_enum
    ================================================================================
    MSG: kortex_driver/ControllerHandle
    
    uint32 type
    uint32 controller_identifier
    ================================================================================
    MSG: kortex_driver/MappingHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetControllerConfigurationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = ControllerConfiguration.Resolve(msg.output)
    }
    else {
      resolved.output = new ControllerConfiguration()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetControllerConfigurationRequest,
  Response: GetControllerConfigurationResponse,
  md5sum() { return '486c0887673fbfba27c914b77b97b479'; },
  datatype() { return 'kortex_driver/GetControllerConfiguration'; }
};
