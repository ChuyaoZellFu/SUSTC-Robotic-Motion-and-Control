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

let SafetyInformation = require('../msg/SafetyInformation.js');

//-----------------------------------------------------------

class GetSafetyInformationRequest {
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
    // Serializes a message object of type GetSafetyInformationRequest
    // Serialize message field [input]
    bufferOffset = SafetyHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSafetyInformationRequest
    let len;
    let data = new GetSafetyInformationRequest(null);
    // Deserialize message field [input]
    data.input = SafetyHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetSafetyInformationRequest';
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
    const resolved = new GetSafetyInformationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = SafetyHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new SafetyHandle()
    }

    return resolved;
    }
};

class GetSafetyInformationResponse {
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
        this.output = new SafetyInformation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSafetyInformationResponse
    // Serialize message field [output]
    bufferOffset = SafetyInformation.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSafetyInformationResponse
    let len;
    let data = new GetSafetyInformationResponse(null);
    // Deserialize message field [output]
    data.output = SafetyInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 35;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetSafetyInformationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8424594fd065a6af902dd23345f3dd8f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SafetyInformation output
    
    ================================================================================
    MSG: kortex_driver/SafetyInformation
    
    SafetyHandle handle
    bool can_change_safety_state
    bool has_warning_threshold
    bool has_error_threshold
    uint32 limit_type
    float32 default_warning_threshold
    float32 default_error_threshold
    float32 upper_hard_limit
    float32 lower_hard_limit
    uint32 status
    uint32 unit
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
    const resolved = new GetSafetyInformationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = SafetyInformation.Resolve(msg.output)
    }
    else {
      resolved.output = new SafetyInformation()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetSafetyInformationRequest,
  Response: GetSafetyInformationResponse,
  md5sum() { return '03aeca1f2d2314fba73be4aeb3606ece'; },
  datatype() { return 'kortex_driver/GetSafetyInformation'; }
};
