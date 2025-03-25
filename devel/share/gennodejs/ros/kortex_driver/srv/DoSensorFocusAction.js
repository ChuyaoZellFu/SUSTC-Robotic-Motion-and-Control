// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SensorFocusAction = require('../msg/SensorFocusAction.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class DoSensorFocusActionRequest {
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
        this.input = new SensorFocusAction();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DoSensorFocusActionRequest
    // Serialize message field [input]
    bufferOffset = SensorFocusAction.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DoSensorFocusActionRequest
    let len;
    let data = new DoSensorFocusActionRequest(null);
    // Deserialize message field [input]
    data.input = SensorFocusAction.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += SensorFocusAction.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/DoSensorFocusActionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7045797c32f93f031d0933c5fd38ad58';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SensorFocusAction input
    
    ================================================================================
    MSG: kortex_driver/SensorFocusAction
    
    uint32 sensor
    uint32 focus_action
    SensorFocusAction_action_parameters oneof_action_parameters
    ================================================================================
    MSG: kortex_driver/SensorFocusAction_action_parameters
    
    FocusPoint[] focus_point
    ManualFocus[] manual_focus
    ================================================================================
    MSG: kortex_driver/FocusPoint
    
    uint32 x
    uint32 y
    ================================================================================
    MSG: kortex_driver/ManualFocus
    
    uint32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DoSensorFocusActionRequest(null);
    if (msg.input !== undefined) {
      resolved.input = SensorFocusAction.Resolve(msg.input)
    }
    else {
      resolved.input = new SensorFocusAction()
    }

    return resolved;
    }
};

class DoSensorFocusActionResponse {
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
    // Serializes a message object of type DoSensorFocusActionResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DoSensorFocusActionResponse
    let len;
    let data = new DoSensorFocusActionResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/DoSensorFocusActionResponse';
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
    const resolved = new DoSensorFocusActionResponse(null);
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
  Request: DoSensorFocusActionRequest,
  Response: DoSensorFocusActionResponse,
  md5sum() { return 'a727a16574ce67bd4de8fba7da857f9f'; },
  datatype() { return 'kortex_driver/DoSensorFocusAction'; }
};
