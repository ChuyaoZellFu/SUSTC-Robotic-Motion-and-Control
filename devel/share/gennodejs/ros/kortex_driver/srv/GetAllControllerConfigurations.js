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

let ControllerConfigurationList = require('../msg/ControllerConfigurationList.js');

//-----------------------------------------------------------

class GetAllControllerConfigurationsRequest {
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
    // Serializes a message object of type GetAllControllerConfigurationsRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAllControllerConfigurationsRequest
    let len;
    let data = new GetAllControllerConfigurationsRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetAllControllerConfigurationsRequest';
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
    const resolved = new GetAllControllerConfigurationsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetAllControllerConfigurationsResponse {
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
        this.output = new ControllerConfigurationList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAllControllerConfigurationsResponse
    // Serialize message field [output]
    bufferOffset = ControllerConfigurationList.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAllControllerConfigurationsResponse
    let len;
    let data = new GetAllControllerConfigurationsResponse(null);
    // Deserialize message field [output]
    data.output = ControllerConfigurationList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ControllerConfigurationList.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetAllControllerConfigurationsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4480fc05629bc4afd8fd6b76141f1133';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ControllerConfigurationList output
    
    ================================================================================
    MSG: kortex_driver/ControllerConfigurationList
    
    ControllerConfiguration[] controller_configurations
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
    const resolved = new GetAllControllerConfigurationsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = ControllerConfigurationList.Resolve(msg.output)
    }
    else {
      resolved.output = new ControllerConfigurationList()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetAllControllerConfigurationsRequest,
  Response: GetAllControllerConfigurationsResponse,
  md5sum() { return '48bc932dac09542b910dbdc1482df5ca'; },
  datatype() { return 'kortex_driver/GetAllControllerConfigurations'; }
};
