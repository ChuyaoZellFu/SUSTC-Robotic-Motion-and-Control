// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SequenceHandle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.identifier = null;
      this.permission = null;
    }
    else {
      if (initObj.hasOwnProperty('identifier')) {
        this.identifier = initObj.identifier
      }
      else {
        this.identifier = 0;
      }
      if (initObj.hasOwnProperty('permission')) {
        this.permission = initObj.permission
      }
      else {
        this.permission = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SequenceHandle
    // Serialize message field [identifier]
    bufferOffset = _serializer.uint32(obj.identifier, buffer, bufferOffset);
    // Serialize message field [permission]
    bufferOffset = _serializer.uint32(obj.permission, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SequenceHandle
    let len;
    let data = new SequenceHandle(null);
    // Deserialize message field [identifier]
    data.identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [permission]
    data.permission = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SequenceHandle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '10d841c6abc3fd3596d9b10510d50074';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SequenceHandle(null);
    if (msg.identifier !== undefined) {
      resolved.identifier = msg.identifier;
    }
    else {
      resolved.identifier = 0
    }

    if (msg.permission !== undefined) {
      resolved.permission = msg.permission;
    }
    else {
      resolved.permission = 0
    }

    return resolved;
    }
};

module.exports = SequenceHandle;
