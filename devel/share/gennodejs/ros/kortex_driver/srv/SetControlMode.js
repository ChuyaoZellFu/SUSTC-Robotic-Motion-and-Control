// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ActuatorConfig_ControlModeInformation = require('../msg/ActuatorConfig_ControlModeInformation.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetControlModeRequest {
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
        this.input = new ActuatorConfig_ControlModeInformation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetControlModeRequest
    // Serialize message field [input]
    bufferOffset = ActuatorConfig_ControlModeInformation.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetControlModeRequest
    let len;
    let data = new SetControlModeRequest(null);
    // Deserialize message field [input]
    data.input = ActuatorConfig_ControlModeInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetControlModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ca19a04230874fdc811929f351eb6d06';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ActuatorConfig_ControlModeInformation input
    
    ================================================================================
    MSG: kortex_driver/ActuatorConfig_ControlModeInformation
    
    uint32 control_mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetControlModeRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ActuatorConfig_ControlModeInformation.Resolve(msg.input)
    }
    else {
      resolved.input = new ActuatorConfig_ControlModeInformation()
    }

    return resolved;
    }
};

class SetControlModeResponse {
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
    // Serializes a message object of type SetControlModeResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetControlModeResponse
    let len;
    let data = new SetControlModeResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetControlModeResponse';
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
    const resolved = new SetControlModeResponse(null);
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
  Request: SetControlModeRequest,
  Response: SetControlModeResponse,
  md5sum() { return 'cd48317b2c3b2a87721b906a23a31b2f'; },
  datatype() { return 'kortex_driver/SetControlMode'; }
};
