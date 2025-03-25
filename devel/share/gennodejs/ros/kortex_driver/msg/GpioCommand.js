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

class GpioCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.port_identifier = null;
      this.pin_identifier = null;
      this.action = null;
      this.period = null;
    }
    else {
      if (initObj.hasOwnProperty('port_identifier')) {
        this.port_identifier = initObj.port_identifier
      }
      else {
        this.port_identifier = 0;
      }
      if (initObj.hasOwnProperty('pin_identifier')) {
        this.pin_identifier = initObj.pin_identifier
      }
      else {
        this.pin_identifier = 0;
      }
      if (initObj.hasOwnProperty('action')) {
        this.action = initObj.action
      }
      else {
        this.action = 0;
      }
      if (initObj.hasOwnProperty('period')) {
        this.period = initObj.period
      }
      else {
        this.period = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GpioCommand
    // Serialize message field [port_identifier]
    bufferOffset = _serializer.uint32(obj.port_identifier, buffer, bufferOffset);
    // Serialize message field [pin_identifier]
    bufferOffset = _serializer.uint32(obj.pin_identifier, buffer, bufferOffset);
    // Serialize message field [action]
    bufferOffset = _serializer.uint32(obj.action, buffer, bufferOffset);
    // Serialize message field [period]
    bufferOffset = _serializer.uint32(obj.period, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GpioCommand
    let len;
    let data = new GpioCommand(null);
    // Deserialize message field [port_identifier]
    data.port_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pin_identifier]
    data.pin_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [action]
    data.action = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [period]
    data.period = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GpioCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '407fd312655ca86f1a6125a8f767d374';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 port_identifier
    uint32 pin_identifier
    uint32 action
    uint32 period
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GpioCommand(null);
    if (msg.port_identifier !== undefined) {
      resolved.port_identifier = msg.port_identifier;
    }
    else {
      resolved.port_identifier = 0
    }

    if (msg.pin_identifier !== undefined) {
      resolved.pin_identifier = msg.pin_identifier;
    }
    else {
      resolved.pin_identifier = 0
    }

    if (msg.action !== undefined) {
      resolved.action = msg.action;
    }
    else {
      resolved.action = 0
    }

    if (msg.period !== undefined) {
      resolved.period = msg.period;
    }
    else {
      resolved.period = 0
    }

    return resolved;
    }
};

module.exports = GpioCommand;
