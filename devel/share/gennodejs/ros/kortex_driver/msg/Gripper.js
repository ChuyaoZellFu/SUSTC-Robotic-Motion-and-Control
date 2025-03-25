// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Finger = require('./Finger.js');

//-----------------------------------------------------------

class Gripper {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.finger = null;
    }
    else {
      if (initObj.hasOwnProperty('finger')) {
        this.finger = initObj.finger
      }
      else {
        this.finger = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Gripper
    // Serialize message field [finger]
    // Serialize the length for message field [finger]
    bufferOffset = _serializer.uint32(obj.finger.length, buffer, bufferOffset);
    obj.finger.forEach((val) => {
      bufferOffset = Finger.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Gripper
    let len;
    let data = new Gripper(null);
    // Deserialize message field [finger]
    // Deserialize array length for message field [finger]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.finger = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.finger[i] = Finger.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.finger.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Gripper';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd6de7f2f0e25d1f1553a193c7d388b15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new Gripper(null);
    if (msg.finger !== undefined) {
      resolved.finger = new Array(msg.finger.length);
      for (let i = 0; i < resolved.finger.length; ++i) {
        resolved.finger[i] = Finger.Resolve(msg.finger[i]);
      }
    }
    else {
      resolved.finger = []
    }

    return resolved;
    }
};

module.exports = Gripper;
