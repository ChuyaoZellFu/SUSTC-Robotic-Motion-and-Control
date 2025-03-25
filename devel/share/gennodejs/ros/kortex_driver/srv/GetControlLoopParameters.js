// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let LoopSelection = require('../msg/LoopSelection.js');

//-----------------------------------------------------------

let ControlLoopParameters = require('../msg/ControlLoopParameters.js');

//-----------------------------------------------------------

class GetControlLoopParametersRequest {
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
        this.input = new LoopSelection();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetControlLoopParametersRequest
    // Serialize message field [input]
    bufferOffset = LoopSelection.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetControlLoopParametersRequest
    let len;
    let data = new GetControlLoopParametersRequest(null);
    // Deserialize message field [input]
    data.input = LoopSelection.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetControlLoopParametersRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e225a54eed96dc619f1c0c6f3145d845';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    LoopSelection input
    
    ================================================================================
    MSG: kortex_driver/LoopSelection
    
    uint32 loop_selection
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetControlLoopParametersRequest(null);
    if (msg.input !== undefined) {
      resolved.input = LoopSelection.Resolve(msg.input)
    }
    else {
      resolved.input = new LoopSelection()
    }

    return resolved;
    }
};

class GetControlLoopParametersResponse {
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
        this.output = new ControlLoopParameters();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetControlLoopParametersResponse
    // Serialize message field [output]
    bufferOffset = ControlLoopParameters.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetControlLoopParametersResponse
    let len;
    let data = new GetControlLoopParametersResponse(null);
    // Deserialize message field [output]
    data.output = ControlLoopParameters.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ControlLoopParameters.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetControlLoopParametersResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b67a1e3c817362a2b2a1aa4f643576c1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ControlLoopParameters output
    
    ================================================================================
    MSG: kortex_driver/ControlLoopParameters
    
    uint32 loop_selection
    float32 error_saturation
    float32 output_saturation
    float32[] kAz
    float32[] kBz
    float32 error_dead_band
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetControlLoopParametersResponse(null);
    if (msg.output !== undefined) {
      resolved.output = ControlLoopParameters.Resolve(msg.output)
    }
    else {
      resolved.output = new ControlLoopParameters()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetControlLoopParametersRequest,
  Response: GetControlLoopParametersResponse,
  md5sum() { return 'fe660929cab3528eb46e3ce651e5ce58'; },
  datatype() { return 'kortex_driver/GetControlLoopParameters'; }
};
