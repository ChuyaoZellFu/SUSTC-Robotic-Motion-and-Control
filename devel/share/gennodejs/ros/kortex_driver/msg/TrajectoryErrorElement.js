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

class TrajectoryErrorElement {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.error_type = null;
      this.error_identifier = null;
      this.error_value = null;
      this.min_value = null;
      this.max_value = null;
      this.index = null;
      this.message = null;
      this.waypoint_index = null;
    }
    else {
      if (initObj.hasOwnProperty('error_type')) {
        this.error_type = initObj.error_type
      }
      else {
        this.error_type = 0;
      }
      if (initObj.hasOwnProperty('error_identifier')) {
        this.error_identifier = initObj.error_identifier
      }
      else {
        this.error_identifier = 0;
      }
      if (initObj.hasOwnProperty('error_value')) {
        this.error_value = initObj.error_value
      }
      else {
        this.error_value = 0.0;
      }
      if (initObj.hasOwnProperty('min_value')) {
        this.min_value = initObj.min_value
      }
      else {
        this.min_value = 0.0;
      }
      if (initObj.hasOwnProperty('max_value')) {
        this.max_value = initObj.max_value
      }
      else {
        this.max_value = 0.0;
      }
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = 0;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('waypoint_index')) {
        this.waypoint_index = initObj.waypoint_index
      }
      else {
        this.waypoint_index = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryErrorElement
    // Serialize message field [error_type]
    bufferOffset = _serializer.uint32(obj.error_type, buffer, bufferOffset);
    // Serialize message field [error_identifier]
    bufferOffset = _serializer.uint32(obj.error_identifier, buffer, bufferOffset);
    // Serialize message field [error_value]
    bufferOffset = _serializer.float32(obj.error_value, buffer, bufferOffset);
    // Serialize message field [min_value]
    bufferOffset = _serializer.float32(obj.min_value, buffer, bufferOffset);
    // Serialize message field [max_value]
    bufferOffset = _serializer.float32(obj.max_value, buffer, bufferOffset);
    // Serialize message field [index]
    bufferOffset = _serializer.uint32(obj.index, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [waypoint_index]
    bufferOffset = _serializer.uint32(obj.waypoint_index, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryErrorElement
    let len;
    let data = new TrajectoryErrorElement(null);
    // Deserialize message field [error_type]
    data.error_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [error_identifier]
    data.error_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [error_value]
    data.error_value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [min_value]
    data.min_value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_value]
    data.max_value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [index]
    data.index = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [waypoint_index]
    data.waypoint_index = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/TrajectoryErrorElement';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fe19f45de474fe8a7f3de643ea88e472';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 error_type
    uint32 error_identifier
    float32 error_value
    float32 min_value
    float32 max_value
    uint32 index
    string message
    uint32 waypoint_index
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryErrorElement(null);
    if (msg.error_type !== undefined) {
      resolved.error_type = msg.error_type;
    }
    else {
      resolved.error_type = 0
    }

    if (msg.error_identifier !== undefined) {
      resolved.error_identifier = msg.error_identifier;
    }
    else {
      resolved.error_identifier = 0
    }

    if (msg.error_value !== undefined) {
      resolved.error_value = msg.error_value;
    }
    else {
      resolved.error_value = 0.0
    }

    if (msg.min_value !== undefined) {
      resolved.min_value = msg.min_value;
    }
    else {
      resolved.min_value = 0.0
    }

    if (msg.max_value !== undefined) {
      resolved.max_value = msg.max_value;
    }
    else {
      resolved.max_value = 0.0
    }

    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = 0
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.waypoint_index !== undefined) {
      resolved.waypoint_index = msg.waypoint_index;
    }
    else {
      resolved.waypoint_index = 0
    }

    return resolved;
    }
};

module.exports = TrajectoryErrorElement;
