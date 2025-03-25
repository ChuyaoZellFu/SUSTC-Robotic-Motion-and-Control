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

class Finger {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.finger_identifier = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('finger_identifier')) {
        this.finger_identifier = initObj.finger_identifier
      }
      else {
        this.finger_identifier = 0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Finger
    // Serialize message field [finger_identifier]
    bufferOffset = _serializer.uint32(obj.finger_identifier, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.float32(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Finger
    let len;
    let data = new Finger(null);
    // Deserialize message field [finger_identifier]
    data.finger_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Finger';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5bf3287ad42d5a92136c7d9cf8d0eada';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 finger_identifier
    float32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Finger(null);
    if (msg.finger_identifier !== undefined) {
      resolved.finger_identifier = msg.finger_identifier;
    }
    else {
      resolved.finger_identifier = 0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0.0
    }

    return resolved;
    }
};

module.exports = Finger;
