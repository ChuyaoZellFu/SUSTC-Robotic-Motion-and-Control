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

class ControllerElementHandle_identifier {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.button = null;
      this.axis = null;
    }
    else {
      if (initObj.hasOwnProperty('button')) {
        this.button = initObj.button
      }
      else {
        this.button = [];
      }
      if (initObj.hasOwnProperty('axis')) {
        this.axis = initObj.axis
      }
      else {
        this.axis = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerElementHandle_identifier
    // Serialize message field [button]
    bufferOffset = _arraySerializer.uint32(obj.button, buffer, bufferOffset, null);
    // Serialize message field [axis]
    bufferOffset = _arraySerializer.uint32(obj.axis, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerElementHandle_identifier
    let len;
    let data = new ControllerElementHandle_identifier(null);
    // Deserialize message field [button]
    data.button = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [axis]
    data.axis = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.button.length;
    length += 4 * object.axis.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControllerElementHandle_identifier';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41dfd5b8a1f2350cdbaae4a32c67f201';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32[] button
    uint32[] axis
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerElementHandle_identifier(null);
    if (msg.button !== undefined) {
      resolved.button = msg.button;
    }
    else {
      resolved.button = []
    }

    if (msg.axis !== undefined) {
      resolved.axis = msg.axis;
    }
    else {
      resolved.axis = []
    }

    return resolved;
    }
};

module.exports = ControllerElementHandle_identifier;
