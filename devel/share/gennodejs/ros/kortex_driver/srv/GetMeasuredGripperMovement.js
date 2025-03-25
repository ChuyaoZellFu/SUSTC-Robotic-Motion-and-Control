// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GripperRequest = require('../msg/GripperRequest.js');

//-----------------------------------------------------------

let Gripper = require('../msg/Gripper.js');

//-----------------------------------------------------------

class GetMeasuredGripperMovementRequest {
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
        this.input = new GripperRequest();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetMeasuredGripperMovementRequest
    // Serialize message field [input]
    bufferOffset = GripperRequest.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetMeasuredGripperMovementRequest
    let len;
    let data = new GetMeasuredGripperMovementRequest(null);
    // Deserialize message field [input]
    data.input = GripperRequest.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetMeasuredGripperMovementRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '04a7f7e5f6dd9e64a052ae0e30bcfe9a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    GripperRequest input
    
    ================================================================================
    MSG: kortex_driver/GripperRequest
    
    uint32 mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetMeasuredGripperMovementRequest(null);
    if (msg.input !== undefined) {
      resolved.input = GripperRequest.Resolve(msg.input)
    }
    else {
      resolved.input = new GripperRequest()
    }

    return resolved;
    }
};

class GetMeasuredGripperMovementResponse {
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
        this.output = new Gripper();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetMeasuredGripperMovementResponse
    // Serialize message field [output]
    bufferOffset = Gripper.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetMeasuredGripperMovementResponse
    let len;
    let data = new GetMeasuredGripperMovementResponse(null);
    // Deserialize message field [output]
    data.output = Gripper.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Gripper.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetMeasuredGripperMovementResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b2158919272cee2c2e359c210979abae';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Gripper output
    
    ================================================================================
    MSG: kortex_driver/Gripper
    
    Finger[] finger
    ================================================================================
    MSG: kortex_driver/Finger
    
    uint32 finger_identifier
    float32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetMeasuredGripperMovementResponse(null);
    if (msg.output !== undefined) {
      resolved.output = Gripper.Resolve(msg.output)
    }
    else {
      resolved.output = new Gripper()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetMeasuredGripperMovementRequest,
  Response: GetMeasuredGripperMovementResponse,
  md5sum() { return '8a811dcc01240e06c24ef1514e8bc606'; },
  datatype() { return 'kortex_driver/GetMeasuredGripperMovement'; }
};
