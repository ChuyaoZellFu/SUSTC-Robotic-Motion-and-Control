// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointSpeed = require('../msg/JointSpeed.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SendSelectedJointSpeedJoystickCommandRequest {
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
        this.input = new JointSpeed();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SendSelectedJointSpeedJoystickCommandRequest
    // Serialize message field [input]
    bufferOffset = JointSpeed.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SendSelectedJointSpeedJoystickCommandRequest
    let len;
    let data = new SendSelectedJointSpeedJoystickCommandRequest(null);
    // Deserialize message field [input]
    data.input = JointSpeed.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SendSelectedJointSpeedJoystickCommandRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd102e2ca8cf0957c9b3ae29ee1bb2b2e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    JointSpeed input
    
    ================================================================================
    MSG: kortex_driver/JointSpeed
    
    uint32 joint_identifier
    float32 value
    uint32 duration
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SendSelectedJointSpeedJoystickCommandRequest(null);
    if (msg.input !== undefined) {
      resolved.input = JointSpeed.Resolve(msg.input)
    }
    else {
      resolved.input = new JointSpeed()
    }

    return resolved;
    }
};

class SendSelectedJointSpeedJoystickCommandResponse {
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
    // Serializes a message object of type SendSelectedJointSpeedJoystickCommandResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SendSelectedJointSpeedJoystickCommandResponse
    let len;
    let data = new SendSelectedJointSpeedJoystickCommandResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SendSelectedJointSpeedJoystickCommandResponse';
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
    const resolved = new SendSelectedJointSpeedJoystickCommandResponse(null);
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
  Request: SendSelectedJointSpeedJoystickCommandRequest,
  Response: SendSelectedJointSpeedJoystickCommandResponse,
  md5sum() { return '966eae21a761830257b0f20dfcd24c8e'; },
  datatype() { return 'kortex_driver/SendSelectedJointSpeedJoystickCommand'; }
};
