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

class Base_RotationMatrixRow {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.column1 = null;
      this.column2 = null;
      this.column3 = null;
    }
    else {
      if (initObj.hasOwnProperty('column1')) {
        this.column1 = initObj.column1
      }
      else {
        this.column1 = 0.0;
      }
      if (initObj.hasOwnProperty('column2')) {
        this.column2 = initObj.column2
      }
      else {
        this.column2 = 0.0;
      }
      if (initObj.hasOwnProperty('column3')) {
        this.column3 = initObj.column3
      }
      else {
        this.column3 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Base_RotationMatrixRow
    // Serialize message field [column1]
    bufferOffset = _serializer.float32(obj.column1, buffer, bufferOffset);
    // Serialize message field [column2]
    bufferOffset = _serializer.float32(obj.column2, buffer, bufferOffset);
    // Serialize message field [column3]
    bufferOffset = _serializer.float32(obj.column3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Base_RotationMatrixRow
    let len;
    let data = new Base_RotationMatrixRow(null);
    // Deserialize message field [column1]
    data.column1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [column2]
    data.column2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [column3]
    data.column3 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Base_RotationMatrixRow';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7749fbbc183a47c4c30d1e4e586c8b66';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 column1
    float32 column2
    float32 column3
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Base_RotationMatrixRow(null);
    if (msg.column1 !== undefined) {
      resolved.column1 = msg.column1;
    }
    else {
      resolved.column1 = 0.0
    }

    if (msg.column2 !== undefined) {
      resolved.column2 = msg.column2;
    }
    else {
      resolved.column2 = 0.0
    }

    if (msg.column3 !== undefined) {
      resolved.column3 = msg.column3;
    }
    else {
      resolved.column3 = 0.0
    }

    return resolved;
    }
};

module.exports = Base_RotationMatrixRow;
