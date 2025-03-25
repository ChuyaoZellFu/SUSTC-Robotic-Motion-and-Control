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

class GpioEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input_type = null;
      this.behavior = null;
      this.input_identifier = null;
    }
    else {
      if (initObj.hasOwnProperty('input_type')) {
        this.input_type = initObj.input_type
      }
      else {
        this.input_type = 0;
      }
      if (initObj.hasOwnProperty('behavior')) {
        this.behavior = initObj.behavior
      }
      else {
        this.behavior = 0;
      }
      if (initObj.hasOwnProperty('input_identifier')) {
        this.input_identifier = initObj.input_identifier
      }
      else {
        this.input_identifier = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GpioEvent
    // Serialize message field [input_type]
    bufferOffset = _serializer.uint32(obj.input_type, buffer, bufferOffset);
    // Serialize message field [behavior]
    bufferOffset = _serializer.uint32(obj.behavior, buffer, bufferOffset);
    // Serialize message field [input_identifier]
    bufferOffset = _serializer.uint32(obj.input_identifier, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GpioEvent
    let len;
    let data = new GpioEvent(null);
    // Deserialize message field [input_type]
    data.input_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [behavior]
    data.behavior = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [input_identifier]
    data.input_identifier = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GpioEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8f7ada35fd120a9401b9d95b206763c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 input_type
    uint32 behavior
    uint32 input_identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GpioEvent(null);
    if (msg.input_type !== undefined) {
      resolved.input_type = msg.input_type;
    }
    else {
      resolved.input_type = 0
    }

    if (msg.behavior !== undefined) {
      resolved.behavior = msg.behavior;
    }
    else {
      resolved.behavior = 0
    }

    if (msg.input_identifier !== undefined) {
      resolved.input_identifier = msg.input_identifier;
    }
    else {
      resolved.input_identifier = 0
    }

    return resolved;
    }
};

module.exports = GpioEvent;
