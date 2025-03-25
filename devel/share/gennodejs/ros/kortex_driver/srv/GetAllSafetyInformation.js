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

let SafetyInformationList = require('../msg/SafetyInformationList.js');

//-----------------------------------------------------------

class GetAllSafetyInformationRequest {
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
    // Serializes a message object of type GetAllSafetyInformationRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAllSafetyInformationRequest
    let len;
    let data = new GetAllSafetyInformationRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetAllSafetyInformationRequest';
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
    const resolved = new GetAllSafetyInformationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetAllSafetyInformationResponse {
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
        this.output = new SafetyInformationList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetAllSafetyInformationResponse
    // Serialize message field [output]
    bufferOffset = SafetyInformationList.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetAllSafetyInformationResponse
    let len;
    let data = new GetAllSafetyInformationResponse(null);
    // Deserialize message field [output]
    data.output = SafetyInformationList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += SafetyInformationList.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetAllSafetyInformationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '93c578a5c13fe995b1d1d665a7cf5337';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SafetyInformationList output
    
    ================================================================================
    MSG: kortex_driver/SafetyInformationList
    
    SafetyInformation[] information
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
    const resolved = new GetAllSafetyInformationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = SafetyInformationList.Resolve(msg.output)
    }
    else {
      resolved.output = new SafetyInformationList()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetAllSafetyInformationRequest,
  Response: GetAllSafetyInformationResponse,
  md5sum() { return '6cd621443d851423fb32151d65f5f576'; },
  datatype() { return 'kortex_driver/GetAllSafetyInformation'; }
};
