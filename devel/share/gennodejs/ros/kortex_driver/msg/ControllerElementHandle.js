// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControllerHandle = require('./ControllerHandle.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');

//-----------------------------------------------------------

class ControllerElementHandle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.controller_handle = null;
      this.oneof_identifier = null;
    }
    else {
      if (initObj.hasOwnProperty('controller_handle')) {
        this.controller_handle = initObj.controller_handle
      }
      else {
        this.controller_handle = new ControllerHandle();
      }
      if (initObj.hasOwnProperty('oneof_identifier')) {
        this.oneof_identifier = initObj.oneof_identifier
      }
      else {
        this.oneof_identifier = new ControllerElementHandle_identifier();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerElementHandle
    // Serialize message field [controller_handle]
    bufferOffset = ControllerHandle.serialize(obj.controller_handle, buffer, bufferOffset);
    // Serialize message field [oneof_identifier]
    bufferOffset = ControllerElementHandle_identifier.serialize(obj.oneof_identifier, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerElementHandle
    let len;
    let data = new ControllerElementHandle(null);
    // Deserialize message field [controller_handle]
    data.controller_handle = ControllerHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [oneof_identifier]
    data.oneof_identifier = ControllerElementHandle_identifier.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ControllerElementHandle_identifier.getMessageSize(object.oneof_identifier);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControllerElementHandle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8c89bd366e7348e92cb61f477829e5ba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    ControllerHandle controller_handle
    ControllerElementHandle_identifier oneof_identifier
    ================================================================================
    MSG: kortex_driver/ControllerHandle
    
    uint32 type
    uint32 controller_identifier
    ================================================================================
    MSG: kortex_driver/ControllerElementHandle_identifier
    
    uint32[] button
    uint32[] axis
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerElementHandle(null);
    if (msg.controller_handle !== undefined) {
      resolved.controller_handle = ControllerHandle.Resolve(msg.controller_handle)
    }
    else {
      resolved.controller_handle = new ControllerHandle()
    }

    if (msg.oneof_identifier !== undefined) {
      resolved.oneof_identifier = ControllerElementHandle_identifier.Resolve(msg.oneof_identifier)
    }
    else {
      resolved.oneof_identifier = new ControllerElementHandle_identifier()
    }

    return resolved;
    }
};

module.exports = ControllerElementHandle;
